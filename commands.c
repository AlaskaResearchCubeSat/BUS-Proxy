#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <ctl_api.h>
#include <terminal.h>
#include <ARCbus.h>
#include <UCA1_uart.h>
#include <Error.h>
#include "Proxy_errors.h"

//helper function to parse I2C address
//if res is true reject reserved addresses
unsigned char getI2C_addr(char *str,short res){
  unsigned long addr;
  unsigned char tmp;
  char *end;
  //attempt to parse a numeric address
  addr=strtol(str,&end,0);
  //check for errors
  if(end==str){
    //check for symbolic matches
    if(!strcmp(str,"LEDL")){
      return BUS_ADDR_LEDL;
    }else if(!strcmp(str,"ACDS")){
      return BUS_ADDR_ACDS;
    }else if(!strcmp(str,"COMM")){
      return BUS_ADDR_COMM;
    }else if(!strcmp(str,"IMG")){
      return BUS_ADDR_IMG;
    }else if(!strcmp(str,"CDH")){
      return BUS_ADDR_CDH;
    }else if(!strcmp(str,"GC")){
      return BUS_ADDR_GC;
    }
    //not a known address, error
    printf("Error : could not parse address \"%s\".\r\n",str);
    return 0xFF;
  }
  if(*end!=0){
    printf("Error : unknown sufix \"%s\" at end of address\r\n",end);
    return 0xFF;
  }
  //check address length
  if(addr>0x7F){
    printf("Error : address 0x%04lX is not 7 bits.\r\n",addr);
    return 0xFF;
  }
  //check for reserved address
  tmp=0x78&addr;
  if((tmp==0x00 || tmp==0x78) && res){
    printf("Error : address 0x%02lX is reserved.\r\n",addr);
    return 0xFF;
  }
  //return address
  return addr;
}

//reset a MSP430 on command
int restCmd(char **argv,unsigned short argc){
  unsigned char buff[10];
  unsigned char addr;
  unsigned short all=0;
  int resp;
  //force user to pass no arguments to prevent unwanted resets
  if(argc>1){
    puts("Error : too many arguments\r");
    return -1;
  }
  if(argc!=0){
    if(!strcmp(argv[1],"all")){
      all=1;
      addr=BUS_ADDR_GC;
    }else{
      //get address
      addr=getI2C_addr(argv[1],0);
      if(addr==0xFF){
        return 1;
      }
    }
    //setup packet 
    BUS_cmd_init(buff,CMD_RESET);
    resp=BUS_cmd_tx(addr,buff,0,0,BUS_I2C_SEND_FOREGROUND);
    switch(resp){
      case 0:
        puts("Command Sent Sucussfully.\r");
      break;
      case ERR_TIMEOUT:
        puts("IIC timeout Error.\r");
      break;
    }
  }
  //reset if no arguments given or to reset all boards
  if(argc==0 || all){
    //wait for UART buffer to empty
    while(UCA1_CheckBusy());
    //write to WDTCTL without password causes PUC
    reset(ERR_LEV_INFO,PROXY_ERR_SRC_CMD,CMD_ERR_RESET,0);
    //Never reached due to reset
    puts("Error : Reset Failed!\r");
  }
  return 0;
}

//set priority for tasks on the fly
int priorityCmd(char **argv,unsigned short argc){
  extern CTL_TASK_t *ctl_task_list;
  int i,found=0;
  CTL_TASK_t *t=ctl_task_list;
  if(argc<1 || argc>2){
    printf("Error: %s takes one or two arguments, but %u are given.\r\n",argv[0],argc);
    return -1;
  }
  while(t!=NULL){
    if(!strcmp(t->name,argv[1])){
      found=1;
      //match found, break
      break;
    }
    t=t->next;
  }
  //check that a task was found
  if(found==0){
      //no task found, return
      printf("Error: could not find task named %s.\r\n",argv[1]);
      return -3;
  }
  //print original priority
  printf("\"%s\" priority = %u\r\n",t->name,t->priority);
  if(argc==2){
      unsigned char val=atoi(argv[2]);
      if(val==0){
        printf("Error: invalid priority.\r\n");
        return -2;
      }
      //set priority
      ctl_task_set_priority(t,val);
      //print original priority
      printf("new \"%s\" priority = %u\r\n",t->name,t->priority);
  }
  return 0;
}

//get/set ctl_timeslice_period
int timesliceCmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error: too many arguments.\r\n");
    return 0;
  }
  //if one argument given then set otherwise get
  if(argc==1){
    int en;
    CTL_TIME_t val=atol(argv[1]);
    //check value
    if(val==0){
      printf("Error: bad value.\r\n");
      return -1;
    }
    //disable interrupts so that opperation is atomic
    en=ctl_global_interrupts_set(0);
    ctl_timeslice_period=val;
    ctl_global_interrupts_set(en);
  }
  printf("ctl_timeslice_period = %ul\r\n",ctl_timeslice_period);
  return 0;
}

//return state name
const char *stateName(unsigned char state){
  switch(state){
    case CTL_STATE_RUNNABLE:
      return "CTL_STATE_RUNNABLE";
    case CTL_STATE_TIMER_WAIT:
      return "CTL_STATE_TIMER_WAIT";
    case CTL_STATE_EVENT_WAIT_ALL:
      return "CTL_STATE_EVENT_WAIT_ALL";
    case CTL_STATE_EVENT_WAIT_ALL_AC:
      return "CTL_STATE_EVENT_WAIT_ALL_AC";
    case CTL_STATE_EVENT_WAIT_ANY:
      return "CTL_STATE_EVENT_WAIT_ANY";
    case CTL_STATE_EVENT_WAIT_ANY_AC:
      return "CTL_STATE_EVENT_WAIT_ANY_AC";
    case CTL_STATE_SEMAPHORE_WAIT:
      return "CTL_STATE_SEMAPHORE_WAIT";
    case CTL_STATE_MESSAGE_QUEUE_POST_WAIT:
      return "CTL_STATE_MESSAGE_QUEUE_POST_WAIT";
    case CTL_STATE_MESSAGE_QUEUE_RECEIVE_WAIT:
      return "CTL_STATE_MESSAGE_QUEUE_RECEIVE_WAIT";
    case CTL_STATE_MUTEX_WAIT:
      return "CTL_STATE_MUTEX_WAIT";
    case CTL_STATE_SUSPENDED:
      return "CTL_STATE_SUSPENDED";
    default:
      return "unknown state";
  }
}

//print the status of all tasks in a table
int statsCmd(char **argv,unsigned short argc){
  extern CTL_TASK_t *ctl_task_list;
  int i;
  CTL_TASK_t *t=ctl_task_list;
  //format string
  const char *fmt="%-10s\t%u\t\t%c%-28s\t%lu\r\n";
  //print out nice header
  printf("\r\nName\t\tPriority\tState\t\t\t\tTime\r\n--------------------------------------------------------------------\r\n");
  //loop through tasks and print out info
  while(t!=NULL){
    printf(fmt,t->name,t->priority,(t==ctl_task_executing)?'*':' ',stateName(t->state),t->execution_time);
    t=t->next;
  }
  //add a blank line after table
  printf("\r\n");
  return 0;
}


//change the stored I2C address. this does not change the address for the I2C peripheral
int addrCmd(char **argv,unsigned short argc){
  //unsigned long addr;
  //unsigned char tmp;
  //char *end;
  unsigned char addr;
  if(argc==0){
    printf("I2C address = 0x%02X\r\n",*(unsigned char*)0x01000);
    return 0;
  }
  if(argc>1){
    printf("Error : too many arguments\r\n");
    return 1;
  }
  addr=getI2C_addr(argv[1],1);
  if(addr==0xFF){
    return 1;
  }
  //erase address section
  //first disable watchdog
  WDT_STOP();
  //unlock flash memory
  FCTL3=FWKEY;
  //setup flash for erase
  FCTL1=FWKEY|ERASE;
  //dummy write to indicate which segment to erase
  *((char*)0x01000)=0;
  //enable writing
  FCTL1=FWKEY|WRT;
  //write address
  *((char*)0x01000)=addr;
  //disable writing
  FCTL1=FWKEY;
  //lock flash
  FCTL3=FWKEY|LOCK;
  //Kick WDT to restart it
  WDT_KICK();
  //print out message
  printf("I2C Address Changed. Changes will not take effect until after reset.\r\n");
  return 0;
}

//transmit command over I2C
int txCmd(char **argv,unsigned short argc){
  unsigned char buff[10],*ptr,id;
  unsigned char addr;
  unsigned short len;
  unsigned int e;
  char *end;
  int i,resp,nack=BUS_CMD_FL_NACK;
  if(!strcmp(argv[1],"noNACK")){
    nack=0;
    //shift arguments
    argv[1]=argv[0];
    argv++;
    argc--;
  }
  //check number of arguments
  if(argc<2){
    printf("Error : too few arguments.\r\n");
    return 1;
  }
  if(argc>sizeof(buff)){
    printf("Error : too many arguments.\r\n");
    return 2;
  }
  //get address
  addr=getI2C_addr(argv[1],0);
  if(addr==0xFF){
    return 1;
  }
  //get packet ID
  id=strtol(argv[2],&end,0);
  if(end==argv[2]){
      printf("Error : could not parse element \"%s\".\r\n",argv[2]);
      return 2;
  }
  if(*end!=0){
    printf("Error : unknown sufix \"%s\" at end of element \"%s\"\r\n",end,argv[2]);
    return 3;
  }
  //setup packet 
  ptr=BUS_cmd_init(buff,id);
  //pares arguments
  for(i=0;i<argc-2;i++){
    ptr[i]=strtol(argv[i+3],&end,0);
    if(end==argv[i+1]){
        printf("Error : could not parse element \"%s\".\r\n",argv[i+3]);
        return 2;
    }
    if(*end!=0){
      printf("Error : unknown sufix \"%s\" at end of element \"%s\"\r\n",end,argv[i+3]);
      return 3;
    }
  }
  len=i;
  resp=BUS_cmd_tx(addr,buff,len,nack,BUS_I2C_SEND_FOREGROUND);
  switch(resp){
    case RET_SUCCESS:
      printf("Command Sent Sucussfully.\r\n");
    break;
  }
  //check if an error occured
  if(resp<0){
    printf("Error : unable to send command\r\n");
  }
  printf("Resp = %i\r\n",resp);
  return 0;
}

//Send data over SPI
int spiCmd(char **argv,unsigned short argc){
  unsigned char addr;
  char *end;
  unsigned short crc;
  //static unsigned char rx[2048+2];
  unsigned char *rx=NULL;
  int resp,i,len=100;
  if(argc<1){
    printf("Error : too few arguments.\r\n");
    return 3;
  }
  //get address
  addr=getI2C_addr(argv[1],0);
  if(addr==0xFF){
    return 1;
  }
  if(argc>=2){
    //Get packet length
    len=strtol(argv[2],&end,0);
    if(end==argv[2]){
        printf("Error : could not parse length \"%s\".\r\n",argv[2]);
        return 2;
    }
    if(*end!=0){
      printf("Error : unknown sufix \"%s\" at end of length \"%s\"\r\n",end,argv[2]);
      return 3;
    }    
    if(len+2>BUS_get_buffer_size()){
      printf("Error : length is too long. Maximum Length is %u\r\n",BUS_get_buffer_size());
      return 4;
    }
  }
  //get buffer, set a timeout of 2 secconds
  rx=BUS_get_buffer(CTL_TIMEOUT_DELAY,2048);
  //check for error
  if(rx==NULL){
    printf("Error : Timeout while waiting for buffer.\r\n");
    return -1;
  }
  //fill buffer with "random" data
  for(i=0;i<len;i++){
    rx[i]=i;
  }
  //send data
  //TESTING: set pin high
  P8OUT|=BIT0;
  //send SPI data
  resp=BUS_SPI_txrx(addr,rx,rx,len);
  //TESTING: wait for transaction to fully complete
  while(UCB0STAT&UCBBUSY);
  //TESTING: set pin low
  P8OUT&=~BIT0;
  //check return value
  if(resp==RET_SUCCESS){
      //print out data message
      printf("SPI data recived\r\n");
      //print out data
      for(i=0;i<len;i++){
        //printf("0x%02X ",rx[i]);
        printf("%03i ",rx[i]);
      }
      printf("\r\n");
  }else{
    printf("%s\r\n",BUS_error_str(resp));
  }
  //free buffer
  BUS_free_buffer();
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
  addr=getI2C_addr(argv[1],0);
  if(addr==0xFF){
    return 1;
  }
  //setup packet 
  ptr=BUS_cmd_init(buff,6);
  //coppy strings into buffer for sending
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
  addr=getI2C_addr(argv[1],0);
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

//print current time
int timeCmd(char **argv,unsigned short argc){
  printf("time ticker = %li\r\n",get_ticker_time());
  return 0;
}

int asyncCmd(char **argv,unsigned short argc){
   char c;
   int err,resp;
   CTL_EVENT_SET_t e=0,evt;
  enum {ASYNC_PROXY_EV_UART_CHAR=1<<0,ASYNC_PROXY_EV_ASYNC_CHAR=1<<1,ASYNC_PROXY_EV_CLOSE=1<<2};
   unsigned char addr;
   if(argc>1){
    printf("Error : %s takes 0 or 1 arguments\r\n",argv[0]);
    return -1;
  }
  if(argc==1){
    addr=getI2C_addr(argv[1],0);
    if(addr==0xFF){
      return -1;
    }
    printf("Using Address 0x%02X\r\n",addr);
    if((err=async_open(addr))){
      printf("Error : opening async\r\n%s\r\n",BUS_error_str(err));
      return -2;
    }
    printf("async open use ^C to force quit\r\n");
    async_setup_events(&e,0,ASYNC_PROXY_EV_ASYNC_CHAR);
    UCA1_setup_events(&e,0,ASYNC_PROXY_EV_UART_CHAR);
    async_setup_close_event(&e,1<<2);
    for(;;){
      evt=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS,&e,ASYNC_PROXY_EV_UART_CHAR|ASYNC_PROXY_EV_ASYNC_CHAR|ASYNC_PROXY_EV_CLOSE,CTL_TIMEOUT_NONE,0);
      //check for char from UART
      if(evt&(ASYNC_PROXY_EV_UART_CHAR)){
        c=UCA1_Getc();
        //check for ^C
        if(c==0x03){
          //close connection
          resp=async_close();
          if(resp==RET_SUCCESS){
            //print message
            printf("\r\nConnection terminated by user\r\n");
            //exit loop
            break;
          }else{
            printf("\r\nError terminating connection : %s\r\n",BUS_error_str(resp));
            break;
          }
        }
        async_TxChar(c);
      }
      //check for char from async
      if(evt&(ASYNC_PROXY_EV_ASYNC_CHAR)){
        c=async_Getc(); 
        UCA1_TxChar(c);
      }
      if(evt&(ASYNC_PROXY_EV_CLOSE)){
        printf("\r\nconnection closed remotely\r\n");
        break;
      }
    }
    async_setup_events(NULL,0,0);
    UCA1_setup_events(NULL,0,0);
    async_setup_close_event(NULL,0);
  }else{
    if(!async_isOpen()){
      printf("Async is not open, closing anyway.\r\n");
    }
    if(async_close()){
      printf("Error : async_close() failed.\r\n");
    }
  }
}

int sendCmd(char **argv,unsigned short argc){
  unsigned char *ptr,id;
  unsigned short len;
  int i,j,k;
  if(!async_isOpen()){
    printf("Error : Async is not open\r\n");
    return -1;
  }
  //check number of arguments
  if(argc<1){
    printf("Error : too few arguments.\r\n");
    return 1;
  }
  //Send string data
  for(i=1,k=0;i<=argc;i++){
    j=0;
    while(argv[i][j]!=0){
      async_TxChar(argv[i][j++]);
    }
    async_TxChar(' ');
  }
  return 0;
}

int recCmd(char **argv,unsigned short argc){
  int c;
  if(!async_isOpen()){
    printf("Error : Async is not open\r\n");
    return -1;
  }
  //check number of arguments
  if(argc!=0){
    printf("Error : %s takes no arguments.\r\n",argv[0]);
    return 1;
  }
  do{
    //get char
    c=async_CheckKey();
    //check for EOF
    if(c!=EOF){
      //send charecter
      UCA1_TxChar(c);
    }
  }while(c!=EOF);
  //print new line
  printf("\r\n");
  return 0;
}

int ARCsearch_Cmd(char **argv,unsigned short argc){
  unsigned char buff[BUS_I2C_CRC_LEN+BUS_I2C_HDR_LEN],*ptr,*end;
  int i,ret,found=0;
  for(i=0;i<0x7F;i++){
    //setup packet 
    ptr=BUS_cmd_init(buff,7);
    //send command
    ret=BUS_cmd_tx(i,buff,0,0,BUS_I2C_SEND_FOREGROUND);
    if(ret==RET_SUCCESS){
      printf("Device Found at ADDR = 0x%02X\r\n",i);
      found++;
    }else if(ret!=ERR_I2C_NACK){
      printf("Error sending to addr 0x%02X : %s\r\n",i,BUS_error_str(ret));
    }
  }
  if(found==0){
    printf("No devices found on the ARCbus\r\n");
  }else{
    printf("%i %s found on the ARCbus\r\n",found,found==1?"device":"devices");
  }
  return 0;
}


int replayCmd(char **argv,unsigned short argc){
  error_log_replay();
  return 0;
}

//cause an error and see how it is reported on reset
int reset_testCmd(char **argv,unsigned short argc){
  //mutex to abuse
  CTL_MUTEX_t mutex;
  //function pointer
  void (*fp)(void);
  //check what type of error to generate
  if(!strcmp(argv[1],"mutex")){
    //generate mutex unlock call error
    printf("Causing a mutex unlock call error\r\n");
    //wait for chars to clear
    ctl_timeout_wait(ctl_get_current_time()+100);
    //init mutex
    ctl_mutex_init(&mutex);
    //unlock mutex without locking this causes an error
    ctl_mutex_unlock(&mutex);
  }else if(!strcmp(argv[1],"isrCall")){
    //generate unsupported call from ISR error
    printf("Causing an unsupported call from ISR\r\n");
    //wait for chars to clear
    ctl_timeout_wait(ctl_get_current_time()+100);
    //use P2 interrupts for software defined interrupts
    P2IFG|=BIT0;
    P2IE|=BIT0;
  }else if(!strcmp(argv[1],"tasks")){
    //generate no tasks to run error
    printf("Causing a no tasks to run error\r\n");
    //wait for chars to clear
    ctl_timeout_wait(ctl_get_current_time()+100);
    while(ctl_task_executing->next!=NULL){
      ctl_task_remove(ctl_task_executing->next);
    }
    //call timer wait, this generates an error because we have killed all other tasks
    ctl_timeout_wait(ctl_get_current_time()+100);
  }else if(!strcmp(argv[1],"WDT")){
    //generate a watchdog timeout error
    printf("Causing watchdog reset\r\n");
    //wait for chars to clear
    ctl_timeout_wait(ctl_get_current_time()+100);
    WDTCTL=0;
  }else if(!strcmp(argv[1],"flash")){
    //generate a watchdog timeout error
    printf("Causing a flash security key violation\r\n");
    //wait for chars to clear
    ctl_timeout_wait(ctl_get_current_time()+100);
    FCTL1=0;
  }else if(!strcmp(argv[1],"fetch")){
    //generate a watchdog timeout error
    printf("Causing invalid instruction fetch\r\n");
    //wait for chars to clear
    ctl_timeout_wait(ctl_get_current_time()+100);
    //call a function that is located at P1IN
    fp=(void (*)(void))0x20;
    fp();
  }
  printf("Failed to generate error\r\n");
  return 0;
}
  
void error_ISR(void) __ctl_interrupt[PORT2_VECTOR]{
  unsigned char flags=P2IFG&P2IE;
  P2IFG&=~flags;
  //Check for bit zero
  if(flags&BIT0){
    //make an unsupported call from ISR
    ctl_timeout_wait(0);
  }
}




//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]\r\n\t""get a list of commands or help on a spesific command.",helpCmd},
                         {"priority"," task [priority]\r\n\t""Get/set task priority.",priorityCmd},
                         {"timeslice"," [period]\r\n\t""Get/set ctl_timeslice_period.",timesliceCmd},
                         {"stats","\r\n\t""Print task status",statsCmd},
                         {"reset","\r\n\t""reset the msp430.",restCmd},
                         {"addr"," [addr]\r\n\t""Get/Set I2C address.",addrCmd},
                         {"tx"," [noACK] [noNACK] addr ID [[data0] [data1]...]\r\n\t""send data over I2C to an address",txCmd},
                         {"SPI","addr [len]\r\n\t""Send data using SPI.",spiCmd},
                         {"print"," addr str1 [[str2] ... ]\r\n\t""Send a string to addr.",printCmd},
                         {"tst"," addr len\r\n\t""Send test data to addr.",tstCmd},
                         {"time","\r\n\t""Return current time.",timeCmd},
                         {"async","[addr]\r\n\t""Open connection if address given. otherwise close connection.",asyncCmd},
                         {"send"," str1 [[str2] ... ]\r\n\t""Send async data",sendCmd},
                         {"rec","\r\n\t""Recive async data",recCmd},
                         {"search","\r\n\t""Find devices on the bus",ARCsearch_Cmd},
                         {"replay","\r\n\t""Replay errors from log",replayCmd},
                         {"tstrst","error\r\n\t""Cause An error that causes a reset",reset_testCmd},
                         //end of list
                         {NULL,NULL,NULL}};
