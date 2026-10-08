/* E1 wheel counters: A rising edge / B direction, modulo-2^32 counters.
 * Shared by STM32 V6.5 and base ESP V4.4. CRC16 covers text before final comma.
 * E1,sequence,milliseconds,count1,count2,count3,count4,CRC16\n
 */
#ifndef ENCODER_PROTOCOL_H
#define ENCODER_PROTOCOL_H
#include "HandheldProtocol.h"
typedef struct {
    uint32_t sequence, milliseconds, count[4];
} EncoderFrame;
HH_INLINE int Encoder_Format(char *text,size_t capacity,const EncoderFrame *f)
{
    int n,m;
    n=snprintf(text,capacity,"E1,%lu,%lu,%lu,%lu,%lu,%lu",
        (unsigned long)f->sequence,(unsigned long)f->milliseconds,
        (unsigned long)f->count[0],(unsigned long)f->count[1],
        (unsigned long)f->count[2],(unsigned long)f->count[3]);
    if(n<0||(size_t)n>=capacity)return 0;
    m=snprintf(text+n,capacity-(size_t)n,",%04X\n",(unsigned)HH_Crc(text,(size_t)n));
    if(m<0||(size_t)m>=capacity-(size_t)n)return 0;
    return n+m;
}
HH_INLINE int Encoder_Parse(const char *text,size_t length,EncoderFrame *f)
{
    const char *p,*end;
    uint32_t v[6],digit;
    uint16_t crc=0;
    int i;
    char c;
    if(length<19||length>=96||memcmp(text,"E1,",3)!=0||text[length-5]!=',')return 0;
    for(i=0;i<4;i++){
        c=text[length-4+i];
        if(c>='0'&&c<='9')digit=(uint32_t)(c-'0');
        else if(c>='A'&&c<='F')digit=(uint32_t)(c-'A'+10);
        else return 0;
        crc=(uint16_t)((crc<<4)|digit);
    }
    if(crc!=HH_Crc(text,length-5))return 0;
    p=text+3;end=text+length-5;
    for(i=0;i<6;i++){
        if(!HH_Number(&p,end,&v[i]))return 0;
        if(i<5&&(p>=end||*p++!=','))return 0;
    }
    if(p!=end)return 0;
    f->sequence=v[0];f->milliseconds=v[1];
    for(i=0;i<4;i++)f->count[i]=v[i+2];
    return 1;
}
#endif

