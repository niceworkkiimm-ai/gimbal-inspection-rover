#ifndef ROUTE_LOG_H
#define ROUTE_LOG_H
#include "RouteLogProtocol.h"
#include "EncoderProtocol.h"
void RouteLog_Init(void);
void RouteLog_Sample(const EncoderFrame *frame);
void RouteLog_Query(const char *line,size_t length);
int RouteLog_Next(char *line,size_t capacity);
void RouteLog_Sent(void);
#endif
