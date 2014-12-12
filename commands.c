#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <ctl_api.h>
#include <ctl_api.h>
#include <terminal.h>
#include <ARCbus.h>
#include <UCA1_uart.h>
#include <Error.h>
#include <commandLib.h>
#include "Proxy_errors.h"
#include "flash.h"

void write_settings(FLASH_SETTINGS *set){
  int en;
  //erase address section
  //disable interrupts
  en = BUS_stop_interrupts();
  //disable watchdog
  WDT_STOP();
  //unlock flash memory
  FCTL3=FWKEY;
  //setup flash for erase
  FCTL1=FWKEY|ERASE;
  //dummy write to indicate which segment to erase
  saved_settings->magic=0;
  //enable writing
  FCTL1=FWKEY|WRT;
  //write settings
  memcpy(saved_settings,set,sizeof(FLASH_SETTINGS));
  //disable writing
  FCTL1=FWKEY;

  //lock flash
  FCTL3=FWKEY|LOCK;
  //re-enable interrupts if enabled before
  BUS_restart_interrupts(en);
}

//change the stored I2C address. this does not change the address for the I2C peripheral
int addrCmd(char **argv,unsigned short argc){
  const char *name;
  FLASH_SETTINGS tmp;
  //char *end;
  unsigned char addr;
  if(argc==0){
    addr=(~UCGCEN)&UCB0I2COA;
    name=I2C_addr_revlookup(addr,busAddrSym);
    if(name!=NULL){
        printf("I2C address = 0x%02X (%s)\r\n",addr,name);
    }else{
        printf("I2C address = 0x%02X\r\n",addr);
    }
    if(saved_settings->addr!=addr){
        name=I2C_addr_revlookup(saved_settings->addr,busAddrSym);
        if(name!=NULL){
            printf("Saved address = 0x%02X (%s)\r\n",saved_settings->addr,name);
        }else{
            printf("Saved address = 0x%02X\r\n",saved_settings->addr);
        }               
    }    
    return 0;
  }
  if(argc>1){
    printf("Error : too many arguments\r\n");
    return 1;
  }
  //copy saved settings if they are valid
  if(saved_settings->magic==SAVED_SETTINGS_MAGIC){
    memcpy(&tmp,saved_settings,sizeof(FLASH_SETTINGS));
  }else{
    //mark UART settings as invalid by zeroing the clock register
    tmp.clk=0;
  }
  //set magic value
  tmp.magic=SAVED_SETTINGS_MAGIC;
  
  //get address
  tmp.addr=getI2C_addr(argv[1],1,busAddrSym);
  if(tmp.addr==0xFF){
    return 1;
  }
  //write settings to flash
  write_settings(&tmp);
  //print out message
  printf("I2C Address Changed. Changes will not take effect until after reset.\r\n");
  return 0;
}

int printCmd(char **argv,unsigned short argc){
  unsigned char buff[40],*ptr,id;
  unsigned char addr;
  unsigned short len;
  int i,j,k;
  //check number of arguments
  if(argc<2){
    printf("Error : too few arguments.\r\n");
    return 1;
  }
  //get address
  addr=getI2C_addr(argv[1],0,busAddrSym);
  if(addr==0xFF){
    return 1;
  }
  //setup packet 
  ptr=BUS_cmd_init(buff,6);
  //copy strings into buffer for sending
  for(i=2,k=0;i<=argc && k<sizeof(buff);i++){
    j=0;
    while(argv[i][j]!=0){
      ptr[k++]=argv[i][j++];
    }
    ptr[k++]=' ';
  }
  //get length
  len=k;
  //TESTING: set pin high
  P8OUT|=BIT0;
  //send command
  BUS_cmd_tx(addr,buff,len,0,BUS_I2C_SEND_FOREGROUND);
  //TESTING: set pin low
  P8OUT&=~BIT0;
  return 0;
}

int tstCmd(char **argv,unsigned short argc){
  unsigned char buff[40],*ptr,*end;
  unsigned char addr;
  unsigned short len;
  int i,j,k;
  //check number of arguments
  if(argc<2){
    printf("Error : too few arguments.\r\n");
    return 1;
  }
  if(argc>2){
    printf("Error : too many arguments.\r\n");
    return 1;
  }
  //get address
  addr=getI2C_addr(argv[1],0,busAddrSym);
  len = atoi(argv[2]);
  /*if(len<0){
    printf("Error : bad length");
    return 2;
  }*/
  //setup packet 
  ptr=BUS_cmd_init(buff,7);
  //fill packet with dummy data
  for(i=0;i<len;i++){
    ptr[i]=i;
  }
  //TESTING: set pin high
  P8OUT|=BIT0;
  //send command
  BUS_cmd_tx(addr,buff,len,0,BUS_I2C_SEND_FOREGROUND);
  //TESTING: wait for transaction to fully complete
  while(UCB0STAT&UCBBUSY);
  //TESTING: set pin low
  P8OUT&=~BIT0;
  return 0;
}

//define a handy-dandy macro
#define ARRAY_SIZE(a)   (sizeof(a)/sizeof(a[0]))

int baudCmd(char **argv,unsigned short argc){
  enum {BAUD_SHOW,BAUD_SET,BAUD_SAVE,BAUD_LIST};
  int action=BAUD_SHOW,baud,i,idx;
  float rate;
  FLASH_SETTINGS tmp;
  const int bauds[]={9600,38400,57600};
  void (*const fp[])(void)={UCA1_BR9600,UCA1_BR38400,UCA1_BR57600};
  if(argc>0){
    if(!strcmp("show",argv[1])){
      //show baud rate
      action=BAUD_SHOW;
    }else if(!strcmp("set",argv[1])){
        if(argc<2){
          printf("Error : %s set requires one argument\r\n",argv[1]);
          return -1;
        }
        //get baud rate from argument
        baud=atoi(argv[2]);
        //check if rate is valid
        for(i=0,idx=-1;i<ARRAY_SIZE(bauds);i++){
          if(bauds[i]==baud){
            idx=i;
            break;
          }
        }
        if(idx==-1){
          printf("Error: unknown baud rate %i\r\n",baud);
          action=BAUD_LIST;
        }else{
          //set baud rate
          action=BAUD_SET;
        }
    }else if(!strcmp("save",argv[1])){
        action=BAUD_SAVE;
    }else if(!strcmp("list",argv[1])){
        action=BAUD_LIST;
    }else{
      printf("Error: unknown action %s\r\n",argv[1]);
      return -3;
    }
  }
  switch(action){
    case BAUD_SHOW:
      //check if using Oversampeling mode
      if(UCOS16&UCA1MCTL){
        //Get prescaler
        rate=16*((unsigned short)UCA1BR0)|(((unsigned short)UCA1BR1)<<8);
        rate+=((UCA1MCTL&(UCBRF3|UCBRF2|UCBRF1|UCBRF0))>>4)/(float)16;
      }else{
        //Get prescaler
        rate=((unsigned short)UCA1BR0)|(((unsigned short)UCA1BR1)<<8);
        //get modulator setting
        rate+=((UCA1MCTL&(UCBRS2|UCBRS1|UCBRS0))>>1)/(float)8;
          
      }
      //check which clock is used
      switch(UCA1CTL1&(UCSSEL1|UCSSEL0)){
         case UCSSEL_0:
            printf("External clock used. Baud Rate Unknown.\r\n");
          return 0;
         case UCSSEL_1:
            //ACLK used, this is 32.768kHz xtal
            rate=32768/rate;
         break;
         case UCSSEL_2:
         case UCSSEL_3:
            //SMCLK used this is DCO at 16MHz
            rate=16000000/rate;
         break;
      }
      printf("Baud Rate is %u\r\n",(unsigned short)rate);
    break;
    case BAUD_SET:
      printf("Setting New baud rate\r\n");
      //wait for chars to be transmitted
      while(UCA1_CheckBusy()){          
        ctl_timeout_wait(ctl_get_current_time()+100);
      }
      //double check idx
      if(idx<0 || idx>=ARRAY_SIZE(fp)){
        printf("Error: internal error bad index %i\r\n",idx);
        return -4;
      }
      //call function from array
      fp[idx]();
      //print message in new baud rate
      printf("New baud rate set.\r\n");
    break;
    case BAUD_SAVE:
      //copy saved settings if they are valid
      if(saved_settings->magic==SAVED_SETTINGS_MAGIC){
        memcpy(&tmp,saved_settings,sizeof(FLASH_SETTINGS));
      }else{
        //mark address as invalid
        tmp.addr=0xFF;
      }
      //set magic value
      tmp.magic=SAVED_SETTINGS_MAGIC;
      //copy settings from UART
      tmp.br0=UCA1BR0;
      tmp.br1=UCA1BR1;
      tmp.mctl=UCA1MCTL;
      tmp.clk=UCA1CTL1&(UCSSEL1|UCSSEL0);
      //write settings to flash
      write_settings(&tmp);
      //print conformation
      printf("UART settings written to flash.\r\n");
    break;
    case BAUD_LIST:
      printf("Available baud rates:\r\n");
      for(i=0;i<ARRAY_SIZE(bauds);i++){
        printf("%8u\r\n",bauds[i]);
      }
    break;
  }
  return 0;
}

int imagertakepic(char **argv,unsigned short argc){
  unsigned char val = 0;
  unsigned char dat[4 + BUS_I2C_HDR_LEN + BUS_I2C_CRC_LEN],*payload;
  // 14 is takepic, 15 is loadpic

  if(argc < 1)
  {
    printf("Please specify a sector.\r\n");
    return -1;
  }
  if(sscanf(argv[1],"%i",&val)!=1)
  {
     printf("Invalid sector.\r\n");
     return -1;
  }
  
  printf("Sector is \"%i\"\r\n",val);
  
  //initialize packet
  payload=BUS_cmd_init(dat,CMD_IMG_TAKE_PIC_NOW);
  //set payload
  payload[0]=val;
  payload[1]=0;
  payload[2]=0;
  payload[3]=0;
  //send packet
  BUS_cmd_tx(BUS_ADDR_IMG,dat,4,0,BUS_I2C_SEND_FOREGROUND);
  return 0;
}

int imagerloadpic(char **argv,unsigned short argc){
  unsigned char val = 0;
  unsigned char dat[4 + BUS_I2C_HDR_LEN + BUS_I2C_CRC_LEN],*payload;
  // 14 is takepic, 15 is loadpic

  if(argc < 1)
  {
    printf("Please specify a sector.\r\n");
    return -1;
  }
  if(sscanf(argv[1],"%i",&val)!=1)
  {
     printf("Invalid sector.\r\n");
     return -1;
  }
  
  printf("Sector is \"%i\"\r\n",val);
  
  //initialize packet
  payload=BUS_cmd_init(dat,CMD_IMG_READ_PIC);
  //set payload
  payload[0]=val;
  payload[1]=0;
  payload[2]=0;
  payload[3]=0;
  //send packet
  BUS_cmd_tx(BUS_ADDR_IMG,dat,4,0,BUS_I2C_SEND_FOREGROUND);
  return 0;
}

int schedulepic(char **argv,unsigned short argc){
  ticker val = 0;
  unsigned char dat[4 + BUS_I2C_HDR_LEN + BUS_I2C_CRC_LEN],*payload;
  // 14 is takepic, 15 is loadpic, 13 is schedule pic

  if(argc < 1)
  {
    printf("Please specify a time.\r\n");
    return -1;
  }
  if(sscanf(argv[1],"%lu",&val)!=1)
  {
     printf("Invalid time.\r\n");
     return -1;
  }
  
  printf("Time is \"%lu\"\r\n",val);
  
  //initialize packet
  payload=BUS_cmd_init(dat,CMD_IMG_TAKE_TIMED_PIC);
  //set payload
  payload[0]=(val >> 24) & 0xFF;;
  payload[1]=(val >> 16) & 0xFF;
  payload[2]=(val >> 8) & 0xFF;
  payload[3]=val & 0xFF;
  //send packet
  BUS_cmd_tx(BUS_ADDR_IMG,dat,4,0,BUS_I2C_SEND_FOREGROUND);
  return 0;
}



int bufferlen(char **argv,unsigned short argc){
  printf("Buffer size is %i\r\n",BUS_get_buffer_size());
  return 0;
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]\r\n\t""get a list of commands or help on a spesific command.",helpCmd},
                         CTL_COMMANDS,ARC_COMMANDS,REPLAY_ERROR_COMMAND,ERROR_LOG_LEVEL_COMMAND,ARC_ASYNC_PROXY_COMMAND,ARC_SPI_DREAD,
                         {"addr"," [addr]\r\n\t""Get/Set I2C address.",addrCmd},
                         {"baud"," [show|set|save|list] [rate]\r\n\t""Get/Set UART baud rate.",baudCmd},
                         {"tst"," addr len\r\n\t""Send test data to addr.",tstCmd},
                         {"imagertakepic","Send a message to the imager to tell it that it should take a picture",imagertakepic},
                         {"imagerloadpic","Send a message to the imager to tell it that it should load it's most recent picture and send it over SPI",imagerloadpic},
                         {"buffersize","Get the size of the buffer",bufferlen},
                         {"schedulepic","Schedule a picture to be taken",schedulepic},
                         //end of list
                         {NULL,NULL,NULL}};
