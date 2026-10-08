/* H1 tilt-control protocol V1, shared byte-for-byte by STM32/base/NodeMCU.
 * H1,sequence,mode,epoch,ready,roll_tenths,pitch_tenths,CRC16\n
 * mode: 0 gimbal, 1 vehicle, 2 stop. CRC-16/CCITT-FALSE over text before last comma.
 * CRC detects corruption; it is NOT authentication. */
#ifndef HANDHELD_PROTOCOL_H
#define HANDHELD_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#if defined(__CC_ARM)
#define HH_INLINE static __inline
#else
#define HH_INLINE static inline
#endif
typedef struct {
    uint32_t sequence;
    uint8_t mode, epoch, ready;
    int16_t roll, pitch;
} HandheldFrame;
HH_INLINE uint16_t HH_Crc(const char *text, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t b;
    for (i = 0; i < length; i++) {
        crc ^= (uint16_t)(uint8_t)text[i] << 8;
        for (b = 0; b < 8; b++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}
HH_INLINE int HH_Number(const char **cursor, const char *end, uint32_t *out)
{
    uint32_t n=0, digit;
    const char *start=*cursor;
    while (*cursor < end && **cursor >= '0' && **cursor <= '9') {
        digit=(uint32_t)(*(*cursor)++ - '0');
        if (n > (UINT32_MAX-digit)/10U) return 0;
        n=n*10U+digit;
    }
    if (*cursor==start) return 0;
    *out=n; return 1;
}
HH_INLINE int HH_Parse(const char *text, size_t length, HandheldFrame *out)
{
    const char *p, *end;
    uint32_t fields[4], value;
    uint16_t crc=0;
    int i, negative;
    int16_t angles[2];
    char c;
    if (length < 19 || length >= 64 || memcmp(text,"H1,",3)!=0 || text[length-5]!=',') return 0;
    for (i=0;i<4;i++) {
        c=text[length-4+i];
        if (c>='0'&&c<='9') value=(uint32_t)(c-'0');
        else if(c>='A'&&c<='F') value=(uint32_t)(c-'A'+10);
        else return 0;
        crc=(uint16_t)((crc<<4)|value);
    }
    if (crc!=HH_Crc(text,length-5)) return 0;
    p=text+3;end=text+length-5;
    for(i=0;i<4;i++) {
        if(!HH_Number(&p,end,&fields[i])||p>=end||*p++!=',') return 0;
    }
    for(i=0;i<2;i++) {
        negative=(p<end&&*p=='-');
        if(negative)p++;
        if(!HH_Number(&p,end,&value)||value>1800U) return 0;
        angles[i]=(int16_t)(negative?-(int32_t)value:(int32_t)value);
        if(i==0&&(p>=end||*p++!=','))return 0;
    }
    if(p!=end||fields[1]>2U||fields[2]>255U||fields[3]>1U)return 0;
    out->sequence=fields[0];out->mode=(uint8_t)fields[1];out->epoch=(uint8_t)fields[2];out->ready=(uint8_t)fields[3];
    out->roll=angles[0];out->pitch=angles[1];return 1;
}
HH_INLINE int HH_Format(char *text,size_t capacity,const HandheldFrame *frame)
{
    int n,m;
    uint16_t crc;
    if(frame->mode>2||frame->ready>1||frame->roll < -1800||frame->roll>1800||
       frame->pitch < -1800||frame->pitch>1800)return 0;
    n=snprintf(text,capacity,"H1,%lu,%u,%u,%u,%d,%d",(unsigned long)frame->sequence,
               (unsigned)frame->mode,(unsigned)frame->epoch,(unsigned)frame->ready,(int)frame->roll,(int)frame->pitch);
    if(n<0||(size_t)n>=capacity)return 0;
    crc=HH_Crc(text,(size_t)n);
    m=snprintf(text+n,capacity-(size_t)n,",%04X\n",(unsigned)crc);
    if(m<0||(size_t)m>=capacity-(size_t)n)return 0;
    return n+m;
}
#endif
