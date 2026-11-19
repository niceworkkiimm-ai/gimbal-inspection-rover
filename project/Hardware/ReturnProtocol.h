/* Q1 boot,request,action,task; Q3 boot,task,pulse; Q2 status. CRC is not auth. */
#ifndef RETURN_PROTOCOL_H
#define RETURN_PROTOCOL_H
#include "HandheldProtocol.h"
HH_INLINE int ReturnFields(const char *s,size_t n,char kind,uint32_t *out,uint8_t count){
    const char *p,*end;uint16_t crc=0;uint32_t v;uint8_t i;char c;
    if(n<9||n>=160||s[0]!='Q'||s[1]!=kind||s[2]!=','||s[n-5]!=',')return 0;
    for(i=0;i<4;i++){
        c=s[n-4+i];if(c>='0'&&c<='9')v=c-'0';else if(c>='A'&&c<='F')v=c-'A'+10;else return 0;
        crc=(uint16_t)((crc<<4)|v);
    }
    if(crc!=HH_Crc(s,n-5))return 0;
    p=s+3;end=s+n-5;
    for(i=0;i<count;i++){
        if(!HH_Number(&p,end,&out[i]))return 0;
        if(i+1<count&&(p>=end||*p++!=','))return 0;
    }
    return p==end;
}
HH_INLINE int ReturnFinish(char *s,size_t cap,int n){
    int m;if(n<0||(size_t)n>=cap)return 0;
    m=snprintf(s+n,cap-(size_t)n,",%04X\n",(unsigned)HH_Crc(s,(size_t)n));
    return m<0||(size_t)m>=cap-(size_t)n?0:n+m;
}
#endif
