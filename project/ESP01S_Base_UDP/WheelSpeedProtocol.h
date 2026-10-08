/* S1 wheel-speed diagnostics V1. Read-only telemetry, never motor commands.
 * signed speeds are RPM x10; PWM is signed logical duty in permille.
 * CRC-16/CCITT-FALSE detects corruption, not authentication.
 */
#ifndef WHEEL_SPEED_PROTOCOL_H
#define WHEEL_SPEED_PROTOCOL_H
#include "HandheldProtocol.h"
typedef struct {
    uint32_t sequence,milliseconds,cpr;
    uint16_t dt_ms;
    uint8_t enabled,valid,fault,fault_wheels,limited,sign_mask;
    int16_t target[4],measured[4],pwm[4];
} WheelSpeedFrame;
HH_INLINE int WheelSpeedFrame_Valid(const WheelSpeedFrame *f)
{
    uint8_t i;
    if(f->enabled>1||f->valid>1||f->fault>15||f->fault_wheels>15||
       f->limited>15||f->sign_mask>15||f->cpr<1||f->cpr>1000000)return 0;
    for(i=0;i<4;i++)if(f->target[i]<-1000||f->target[i]>1000||
        f->measured[i]<-20000||f->measured[i]>20000||f->pwm[i]<-300||f->pwm[i]>300)return 0;
    return 1;
}
HH_INLINE int WheelSpeedFrame_Format(char *text,size_t capacity,const WheelSpeedFrame *f)
{
    int n,m;
    if(!WheelSpeedFrame_Valid(f))return 0;
    n=snprintf(text,capacity,
      "S1,%lu,%lu,%u,%u,%u,%u,%u,%u,%lu,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
      (unsigned long)f->sequence,(unsigned long)f->milliseconds,(unsigned)f->enabled,
      (unsigned)f->valid,(unsigned)f->fault,(unsigned)f->fault_wheels,(unsigned)f->limited,
      (unsigned)f->dt_ms,(unsigned long)f->cpr,(unsigned)f->sign_mask,
      (int)f->target[0],(int)f->target[1],(int)f->target[2],(int)f->target[3],
      (int)f->measured[0],(int)f->measured[1],(int)f->measured[2],(int)f->measured[3],
      (int)f->pwm[0],(int)f->pwm[1],(int)f->pwm[2],(int)f->pwm[3]);
    if(n<0||(size_t)n>=capacity)return 0;
    m=snprintf(text+n,capacity-(size_t)n,",%04X\n",(unsigned)HH_Crc(text,(size_t)n));
    if(m<0||(size_t)m>=capacity-(size_t)n)return 0;
    return n+m;
}
HH_INLINE int WheelSpeedFrame_Parse(const char *text,size_t length,WheelSpeedFrame *f)
{
    const char *p,*end;
    uint32_t v[10],digit;
    int32_t values[12];
    uint16_t crc=0;
    int i,negative;
    char c;
    if(length<49||length>=192||memcmp(text,"S1,",3)!=0||text[length-5]!=',')return 0;
    for(i=0;i<4;i++){
        c=text[length-4+i];
        if(c>='0'&&c<='9')digit=(uint32_t)(c-'0');
        else if(c>='A'&&c<='F')digit=(uint32_t)(c-'A'+10);
        else return 0;
        crc=(uint16_t)((crc<<4)|digit);
    }
    if(crc!=HH_Crc(text,length-5))return 0;
    p=text+3;end=text+length-5;
    for(i=0;i<10;i++)if(!HH_Number(&p,end,&v[i])||p>=end||*p++!=',')return 0;
    for(i=0;i<12;i++){
        negative=(p<end&&*p=='-');if(negative)p++;
        if(!HH_Number(&p,end,&digit)||digit>20000U)return 0;
        values[i]=negative?-(int32_t)digit:(int32_t)digit;
        if(i<11&&(p>=end||*p++!=','))return 0;
    }
    if(p!=end||v[2]>1||v[3]>1||v[4]>15||v[5]>15||v[6]>15||v[7]>65535||
       !v[8]||v[8]>1000000||v[9]>15)return 0;
    f->sequence=v[0];f->milliseconds=v[1];f->enabled=(uint8_t)v[2];f->valid=(uint8_t)v[3];
    f->fault=(uint8_t)v[4];f->fault_wheels=(uint8_t)v[5];f->limited=(uint8_t)v[6];
    f->dt_ms=(uint16_t)v[7];f->cpr=v[8];f->sign_mask=(uint8_t)v[9];
    for(i=0;i<4;i++){f->target[i]=(int16_t)values[i];f->measured[i]=(int16_t)values[i+4];f->pwm[i]=(int16_t)values[i+8];}
    return WheelSpeedFrame_Valid(f);
}
#endif
