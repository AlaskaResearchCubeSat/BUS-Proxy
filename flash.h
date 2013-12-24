#ifndef __FLASH_H
  #define __FLASH_H
  
  typedef struct{
    unsigned short magic;
    //address for bus communication
    unsigned char addr;
    //baud rate settings
    unsigned char br0,br1,mctl,clk;
  } FLASH_SETTINGS;
  
  #define saved_settings   ((FLASH_SETTINGS*)0x01000)
  
  #define SAVED_SETTINGS_MAGIC        0xA45F
  
#endif
    