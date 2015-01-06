// MODIFIED
#include <msp430.h>
#include <ctl_api.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <ARCbus.h>
#include <UCA1_uart.h>
#include <Error.h>
#include "timerA.h"
#include "Proxy_errors.h"
#include <terminal.h>
#include <crc.h>
#include "flash.h"
#include "../IMG/IMG.h"

CTL_TASK_t tasks[3];

//stacks for tasks
unsigned stack1[1+200+1];  
unsigned stack2[1+500+1];
unsigned stack3[1+150+1];  

CTL_EVENT_SET_t cmd_parse_evt;



//set printf and friends to send chars out UCA1 uart
int __putchar(int c){
  //don't print if async connection is open
  if(!async_isOpen()){
    return UCA1_TxChar(c);
  }else{
    return EOF;
  }
}

//set printf and friends to send chars out UCA1 uart
int __getchar(void){
  return UCA1_Getc();
}

//handle subsystem specific commands
int SUB_parseCmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len){
  //Return Error
  return ERR_UNKNOWN_CMD;
}

void cmd_parse(void *p) __toplevel{
  unsigned int e;
  //init event
  ctl_events_init(&cmd_parse_evt,0);
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&cmd_parse_evt,0x00,CTL_TIMEOUT_NONE,0);

  }
}

void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i;
  unsigned char buf[10],*ptr;
  IMG_DAT *block;
  unsigned short check;
  extern unsigned char async_addr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL|SUB_EV_ASYNC_OPEN|SUB_EV_ASYNC_CLOSE,CTL_TIMEOUT_NONE,0);
    if(e&SUB_EV_PWR_OFF){
      //print message
      puts("System Powering Down\r");
    }
    if(e&SUB_EV_PWR_ON){
      //print message
      puts("System Powering Up\r");
    }
    if(e&SUB_EV_SEND_STAT){
      //send status
      //puts("Sending status\r");
      //setup packet 
      //TODO: put actual command for subsystem response
      ptr=BUS_cmd_init(buf,20);
      //TODO: fill in telemetry data
      //send command
      BUS_cmd_tx(BUS_ADDR_CDH,buf,0,0,BUS_I2C_SEND_FOREGROUND);
    }
    if(e&SUB_EV_SPI_DAT){
        //get length
        len=arcBus_stat.spi_stat.len;
        //check the data type
        switch(arcBus_stat.spi_stat.rx[0]){
          case SPI_ERROR_DAT:
            //print errors
            print_spi_err(arcBus_stat.spi_stat.rx,len);
          break;
          case SPI_IMG_DAT:
              block=(IMG_DAT*)(arcBus_stat.spi_stat.rx+2);
              //check length
              if(len!=sizeof(IMG_DAT)+2){
                  printf("Incorrect image block length %i\r\n",len);
              }
              //calculate CRC
              check=crc16(block,sizeof(*block)-sizeof(block->CRC));
              //check block CRC
              if(check!=block->CRC){
                  printf("Error : incorrect CRC 0x%04X sent 0x%04X calculated\r\n",block->CRC,check);
              }
              //check block type
              switch(block->magic){
                  case BT_IMG_START:
                      // Print out data received from SD card
                      printf("Received first image block from image #%i\r\n""Total blocks = %i\r\n",block->num,block->block);
                  break;
                  case BT_IMG_BODY:
                      // Print out data received from SD card
                      printf("Received bytes from image #%i block #%i\r\n",block->num,block->block);
                  break;
                  default:
                      printf("Error : Invalid block\r\n");
                  break;
              }
              //print out data
              for(i=0,ptr=(unsigned char*)block;i<sizeof(IMG_DAT);i++){
                printf("0x%02X ",ptr[i]);
                if(i%16==15){
                    printf("\r\n");
                }
              }
          break;
          default:
              puts("Unknown SPI data recived:\r");
              //print out data
              for(i=0;i<len;i++){
                //printf("0x%02X ",rx[i]);
                printf("%03i ",arcBus_stat.spi_stat.rx[i]);
              }
              printf("\r\n");
        }
        //free buffer
        BUS_free_buffer_from_event();
    }
    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
      report_error(ERR_LEV_ERROR,PROXY_ERR_SRC_SUBSYSTEM,SUB_ERR_SPI_CRC,0);
    }
    if(e&SUB_EV_SPI_ERR_BUSY){
      puts("SPI Busy\r");
    }
    if(e&SUB_EV_ASYNC_OPEN){
      //close async connection, not supported
      async_close();
    }
  }
}

int main(void){
  unsigned char addr;
  //DO this first
  ARC_setup(); 
  
  //setup system specific peripherals

  //setup mmc interface
  //mmcInit_msp();


  //TESTING: set log level to report everything by default
  set_error_level(0);
     
  //disable timesliceing so that communication is possible without CDH
  BUS_set_test_mode(BUS_TM_NO_TIMESLICE);
 
  //setup UCA1 UART
  UCA1_init_UART();
  UCA1_BR57600();

  //setup P7 for LED's
  P7OUT=0x00;
  P7DIR=0xFF;
  
  //set default address, an unused address
  addr=0x1F;
  //check if settings are valid
  if(saved_settings->magic==SAVED_SETTINGS_MAGIC){
    //check if address is valid
    if(addr&0x7F){
      //get address
      addr=saved_settings->addr;
    }
    //check if UART settings are valid
    if(saved_settings->clk&(UCSSEL0|UCSSEL1)){
      //disable interrupts
      UC1IE&=~(UCA1TXIE|UCA1RXIE);
      //put module into reset mode
      UCA1CTL1|=UCSWRST;
      UCA1BR0=saved_settings->br0;
      UCA1BR1=saved_settings->br1;
      UCA1MCTL=saved_settings->mctl;
      //clear clock bits
      UCA1CTL1&=~(UCSSEL0|UCSSEL1);
      UCA1CTL1|=saved_settings->clk;
      //take UCA1 out of reset mode
      UCA1CTL1&=~UCSWRST;
      //enable interrupts
      UC1IE|=UCA1TXIE|UCA1RXIE;
    }
        
  }
  //setup bus interface
  initARCbus(addr);

  //initialize stacks
  memset(stack1,0xcd,sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack
  
  memset(stack2,0xcd,sizeof(stack2));  // write known values into the stack
  stack2[0]=stack2[sizeof(stack2)/sizeof(stack2[0])-1]=0xfeed; // put marker values at the words before/after the stack
    
  memset(stack3,0xcd,sizeof(stack3));  // write known values into the stack
  stack3[0]=stack3[sizeof(stack3)/sizeof(stack3[0])-1]=0xfeed; // put marker values at the words before/after the stack

  //create tasks
  ctl_task_run(&tasks[0],BUS_PRI_LOW,cmd_parse,NULL,"cmd_parse",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  ctl_task_run(&tasks[1],BUS_PRI_NORMAL,terminal,"ARC Bus Test Program","terminal",sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
  ctl_task_run(&tasks[2],BUS_PRI_HIGH,sub_events,NULL,"sub_events",sizeof(stack3)/sizeof(stack3[0])-2,stack3+1,0);
  
  mainLoop();
}

