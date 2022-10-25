#include "StringFormat.h"

#include <stdarg.h>


void _StringFormat(String& str,const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  size_t len = strlen(format)+75;
  char* buffer = (char*)malloc(len);
  if (!buffer) 
    return;
  size_t newlen = vsnprintf(buffer, len-1, format, arg);
  va_end(arg);
  if (newlen>=len-1)
  {
    free(buffer);
    len = newlen+5;
    buffer = (char*)malloc(len);
    if (!buffer) 
      return;
    va_start(arg, format);
    newlen = vsnprintf(buffer, len-1, format, arg);
    va_end(arg);
  }
  str=buffer;
  free(buffer);
}
void _StringAddFormat(String& str,const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  size_t len = strlen(format)+75;
  char* buffer = (char*)malloc(len);
  if (!buffer) 
    return;
  size_t newlen = vsnprintf(buffer, len-1, format, arg);
  va_end(arg);
  if (newlen>=len-1)
  {
    free(buffer);
    len = newlen+5;
    buffer = (char*)malloc(len);
    if (!buffer) 
      return;
    va_start(arg, format);
    newlen = vsnprintf(buffer, len-1, format, arg);
    va_end(arg);
  }
  str+=buffer;
  free(buffer);
}
