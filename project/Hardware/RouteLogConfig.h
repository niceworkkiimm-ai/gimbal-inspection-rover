/* V8.9 / 2026-09-15: read-only odometry calibration, separated from auto drive.
 * Geometry/signs/CRC remain compatible with existing saved route files.
 * Wheel diameter 97mm; left/right 202mm; front/rear 210mm. No wiring change.
 */
#ifndef ROUTE_LOG_CONFIG_H
#define ROUTE_LOG_CONFIG_H
#include "HandheldProtocol.h"
#include "WheelSpeedConfig.h"
#define ROUTE_DIAMETER_MM 97U
#define ROUTE_TRACK_MM 202U
#define ROUTE_WHEELBASE_MM 210U
HH_INLINE uint16_t RouteLog_ConfigCrc(void){
    const int8_t signs[4]=WHEEL_ENCODER_SIGNS;
    char text[80];int n;unsigned mask=0,i;
    for(i=0;i<4;i++)if(signs[i]<0)mask|=1U<<i;
    n=snprintf(text,sizeof(text),"%lu,%u,%u,%u,%u",(unsigned long)WHEEL_SPEED_CPR,
        ROUTE_DIAMETER_MM,ROUTE_TRACK_MM,ROUTE_WHEELBASE_MM,mask);
    return HH_Crc(text,(size_t)n);
}
#endif
