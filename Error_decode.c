#include <Error.h>
#include "Proxy_errors.h"

//decode errors from ACDS system
char *err_decode(char buf[150], unsigned short source,int err, unsigned short argument){
  switch(source){
    case PROXY_ERR_SRC_CMD:
      switch(err){
        case CMD_ERR_RESET:
          return "Command Line : Reset Command reset";
      }
    break;
    case PROXY_ERR_SRC_SUBSYSTEM:
      switch(err){
        case SUB_ERR_SPI_CRC:
          return "Subsystem : Bad SPI CRC event";
      }
    break; 
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}
