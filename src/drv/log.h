#include <stdio.h>  
#include <stdarg.h> 
#include "pubDef.h"

//#define DBG_PRINT (print_def("%s:%u %s:\t", __FILE__, __LINE__, __func__), print_def)

#define DBG_PRINT DBG_POST_Print
#define DBG_WARNING_PRINT DBG_POST_WARNING_Print


#define DBG_ERR_PRINT(x, ...)				{DBG_POST_Print(ANSI_DARK_RED"ERROR "ANSI_NONE); \
																DBG_POST_Print(x, ##__VA_ARGS__);}







