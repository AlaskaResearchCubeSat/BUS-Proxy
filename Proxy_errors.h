#ifndef __BUS_ERRORS_H
  #define __BUS_ERRORS_H
  #include <commandLib.h>
  
  //error sources for BUS test program
  enum{PROXY_ERR_SRC_SUBSYSTEM=ERR_SRC_CMD+1};
    
  //subsystem errors
  enum{SUB_ERR_SPI_CRC};
#endif
  