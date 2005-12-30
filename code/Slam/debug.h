#ifndef DEBUG_H
#define DEBUG_H
//#include "DROSDebug.h"

#ifdef DEBUG
#include <stdio.h>
#define DEBUG_print(format,args...) printf(format, ##args)
#define DEBUG_error(format,args...) fprintf(stderr,format, ##args)
#else
#define DEBUG_print(format,args...)
#define DEBUG_error(format,args...)
#endif

#endif 
