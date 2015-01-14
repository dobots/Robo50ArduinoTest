#ifndef _H_LOG
#define _H_LOG

#include "Arduino.h"
#include "debug.h"

extern int log_level;

// void initLogging(Stream *stream);
// void LOGi(const int loglevel, const char* fmt, ... );
// void LOGd(const int loglevel, const char* fmt, ... );

#ifdef DEBUG
	#define LOGi(loglevel, fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
	#define LOGd(loglevel, fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#else
	#define LOGi(loglevel, fmt, ...)
	#define LOGd(loglevel, fmt, ...)
#endif

#endif
