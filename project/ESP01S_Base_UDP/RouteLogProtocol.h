/* V7.1: read-only route journal. CRC16 detects corruption, not authentication.
 * J1: client,request,after (UINT32_MAX means latest)
 * J2: client,request,bootClient,bootRequest,oldest,newest,config,count,first
 * J3: client,request,index,sequence,ms,x_mm,y_mm,heading_mrad,distance_mm,flags
 */
#ifndef ROUTE_LOG_PROTOCOL_H
#define ROUTE_LOG_PROTOCOL_H
#include "HandheldProtocol.h"
#define JOURNAL_BATCH 8U
#define JOURNAL_LINE_MAX 192U
#define JOURNAL_CAPACITY 1024U
typedef struct {
    uint32_t sequence,ms;
    int32_t x,y,h;
    uint32_t distance,flags;
} JournalPoint;
HH_INLINE int JournalFields(const char *s,size_t n,char kind,int64_t *out,uint8_t count){
    const char *p,*end;uint16_t crc=0;uint32_t v;uint8_t i,negative;char c;
    if(n<9||n>=JOURNAL_LINE_MAX||s[0]!='J'||s[1]!=kind||s[2]!=','||s[n-5]!=',')return 0;
    for(i=0;i<4;i++){c=s[n-4+i];if(c>='0'&&c<='9')v=c-'0';else if(c>='A'&&c<='F')v=c-'A'+10;else return 0;crc=(uint16_t)((crc<<4)|v);}
    if(crc!=HH_Crc(s,n-5))return 0;
    p=s+3;end=s+n-5;
    for(i=0;i<count;i++){
        negative=(uint8_t)(p<end&&*p=='-');if(negative)p++;
        if(!HH_Number(&p,end,&v))return 0;
        out[i]=negative?-(int64_t)v:(int64_t)v;
        if(i+1<count&&(p>=end||*p++!=','))return 0;
    }
    return p==end;
}
HH_INLINE int JournalUnsigned(int64_t v){return v>=0&&v<=UINT32_MAX;}
HH_INLINE int JournalFinish(char *s,size_t cap,int n){
    int m;if(n<0||(size_t)n>=cap)return 0;
    m=snprintf(s+n,cap-(size_t)n,",%04X\n",(unsigned)HH_Crc(s,(size_t)n));
    return m<0||(size_t)m>=cap-(size_t)n?0:n+m;
}
#endif
