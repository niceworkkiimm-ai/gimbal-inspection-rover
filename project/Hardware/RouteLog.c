/* V8.2 / 2026-09-13: route-only 80ms count accumulation and mecanum
 * consistency gate. V8.9: read-only recording is independent of removed auto drive.
 * Check >=40ms, integrate/retain >=80ms (1024 ~= 81.9s minimum history).
 * A query snapshots <=8 records. Retries are idempotent reads, never movement.
 */
#include "RouteLog.h"
#include "RouteLogConfig.h"
#include "WheelSpeedConfig.h"
#include <math.h>
#include <string.h>
static JournalPoint history[JOURNAL_CAPACITY],batch[JOURNAL_BATCH];
static uint32_t newest,retained,boot_client,boot_request;
static uint32_t sample_at,record_at,counts[4],flags;
static float x,y,h,distance;
static float window_w[4];
static uint8_t window_bad;
static uint8_t initialized;
static uint32_t client,request,first,batch_oldest,batch_newest;
static uint8_t batch_count,send_index,pending;
static const int8_t signs[4]=WHEEL_ENCODER_SIGNS;
static float angle(float a){return atan2f(sinf(a),cosf(a));}
static int32_t rounded(float v){return (int32_t)(v+(v<0?-.5f:.5f));}
static void append(uint32_t ms){
    JournalPoint *p;
    /* No sequence reuse within a session, including a multi-year uptime. */
    if(newest==UINT32_MAX-1U){newest=retained=boot_client=boot_request=0;flags|=1U;}
    newest++;p=&history[(newest-1U)%JOURNAL_CAPACITY];
    p->sequence=newest;p->ms=ms;p->x=rounded(x*1000);p->y=rounded(y*1000);
    p->h=rounded(h*1000);p->distance=(uint32_t)(distance*1000+.5f);p->flags=flags;
    flags=0;record_at=ms;if(retained<JOURNAL_CAPACITY)retained++;
}
void RouteLog_Init(void){
    newest=retained=boot_client=boot_request=0;initialized=pending=0;
    x=y=h=distance=0;flags=0;
    memset(window_w,0,sizeof(window_w));window_bad=0;
}
void RouteLog_Sample(const EncoderFrame *f){
    uint32_t dt,d;uint8_t i;int64_t ticks;
    float w,dx,dy,da,a,q,bx,by,k,total,residual;
    const float tick_m=3.14159265358979323846f*ROUTE_DIAMETER_MM/(1000.0f*WHEEL_SPEED_CPR);
    if(!initialized){
        initialized=1;sample_at=f->milliseconds;memcpy(counts,f->count,sizeof(counts));
        append(f->milliseconds);return;
    }
    dt=f->milliseconds-sample_at;if(dt<40U)return;
    sample_at=f->milliseconds;
    for(i=0;i<4;i++){
        d=f->count[i]-counts[i];counts[i]=f->count[i];
        ticks=d<=0x7fffffffU?(int64_t)d:-(int64_t)(~d)-1;
        w=(float)ticks*signs[i]*tick_m;
        /* Preserve the existing 2m/s plausibility bound for route recording. */
        if(fabsf(w)>0.002f*dt)window_bad=1;
        window_w[i]+=w;
    }
    /* Never reconstruct an unknown bend from a long local sampling outage. */
    if(dt>160U)window_bad=1;
    if((uint32_t)(f->milliseconds-record_at)<80U)return;
    /* X mecanum has one redundant equation: LF+RF == LR+RR, for forward,
     * lateral, curved and pure-turn motion alike. Allow four counts for
     * independent edge quantization plus 15% of total travel for mismatch.
     * An inconsistent block is UNKNOWN, not a guessed straight-line move.
     * This cannot identify the bad wheel or detect every form of wheel slip. */
    total=0;for(i=0;i<4;i++)total+=fabsf(window_w[i]);
    residual=fabsf(window_w[0]+window_w[1]-window_w[2]-window_w[3]);
    if(residual>4.0f*tick_m+0.15f*total)window_bad=1;
    if(window_bad)flags|=1U;
    else{
        dx=(window_w[0]+window_w[1]+window_w[2]+window_w[3])/4;
        dy=(-window_w[0]+window_w[1]+window_w[2]-window_w[3])/4;
        k=(ROUTE_TRACK_MM+ROUTE_WHEELBASE_MM)/2000.0f;
        da=(-window_w[0]+window_w[1]-window_w[2]+window_w[3])/(4*k);
        a=fabsf(da)<1e-6f?1:sinf(da)/da;q=fabsf(da)<1e-6f?0:(1-cosf(da))/da;
        bx=a*dx-q*dy;by=q*dx+a*dy;
        x+=cosf(h)*bx-sinf(h)*by;y+=sinf(h)*bx+cosf(h)*by;h=angle(h+da);
        distance+=sqrtf(dx*dx+dy*dy);
        if(fabsf(x)>100000||fabsf(y)>100000||distance>1000000){
            x=y=h=distance=0;newest=retained=boot_client=boot_request=0;flags|=1U;
        }
    }
    memset(window_w,0,sizeof(window_w));window_bad=0;
    append(f->milliseconds);
}
void RouteLog_Query(const char *line,size_t length){
    int64_t v[3];uint32_t after,i,oldest;
    if(!JournalFields(line,length,'1',v,3)||!JournalUnsigned(v[0])||!v[0]||
       !JournalUnsigned(v[1])||!v[1]||!JournalUnsigned(v[2])||!retained)return;
    client=(uint32_t)v[0];request=(uint32_t)v[1];after=(uint32_t)v[2];
    /* First valid query after MCU reset assigns a fresh two-word session.
     * ESP changes request for every retry; an ESP-only reset keeps MCU session.
     */
    if(!boot_client){boot_client=client;boot_request=request;}
    oldest=newest-retained+1U;batch_oldest=oldest;batch_newest=newest;
    first=after==UINT32_MAX?newest:after<oldest-1U||after>newest?oldest:after+1U;
    batch_count=first>newest?0:(uint8_t)((newest-first+1U)>JOURNAL_BATCH?JOURNAL_BATCH:newest-first+1U);
    for(i=0;i<batch_count;i++)batch[i]=history[(first+i-1U)%JOURNAL_CAPACITY];
    send_index=0;pending=1;
}
int RouteLog_Next(char *line,size_t capacity){
    JournalPoint *p;
    if(!pending)return 0;
    if(!send_index)return JournalFinish(line,capacity,snprintf(line,capacity,
        "J2,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%lu",
        (unsigned long)client,(unsigned long)request,(unsigned long)boot_client,(unsigned long)boot_request,
        (unsigned long)batch_oldest,(unsigned long)batch_newest,RouteLog_ConfigCrc(),batch_count,(unsigned long)first));
    p=&batch[send_index-1U];
    return JournalFinish(line,capacity,snprintf(line,capacity,"J3,%lu,%lu,%u,%lu,%lu,%ld,%ld,%ld,%lu,%lu",
        (unsigned long)client,(unsigned long)request,send_index-1U,(unsigned long)p->sequence,
        (unsigned long)p->ms,(long)p->x,(long)p->y,(long)p->h,(unsigned long)p->distance,(unsigned long)p->flags));
}
void RouteLog_Sent(void){if(pending&&++send_index>batch_count)pending=0;}
