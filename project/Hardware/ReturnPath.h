/* Reverse traversal of locally recorded wheel positions. No map/file uploads. */
#ifndef RETURN_PATH_H
#define RETURN_PATH_H
#include <stdint.h>
#include "WheelSpeedConfig.h"
#define RETURN_CAPACITY 1024U
#define RETURN_BROWSER_TIMEOUT_MS 1500U
#define RETURN_TURN_RPM 72U
#define RETURN_SYNC_RPM 45U /* maximum while one moving wheel waits for another */
#define RETURN_PARK_GAP_MS 350U
#define RETURN_APPROACH_RPM 12.0f
#define RETURN_CRUISE_MIN_TICKS 12 /* short creep segments stay in position control */
#define RETURN_CRUISE_RAMP_RPM_S 120.0f
#define RETURN_BRAKE_RPM_S 120.0f
#define RETURN_TOLERANCE_TICKS 3
#define RETURN_LOOKAHEAD_TICKS 24 /* at most one adjacent segment, about 20 mm at the rim */
enum { RETURN_IDLE, RETURN_RECORDING, RETURN_RUNNING, RETURN_DONE, RETURN_ABORTED };
enum { RETURN_OK, RETURN_LINK, RETURN_MANUAL, RETURN_FEEDBACK, RETURN_GAP,
       RETURN_FULL, RETURN_TIMEOUT, RETURN_STOP, RETURN_BROWSER, RETURN_COUNTS };
enum { RETURN_ACCEPTED, RETURN_NOT_READY, RETURN_BAD_TASK, RETURN_NO_PATH,
       RETURN_BAD_BOOT, RETURN_OLD_REQUEST, RETURN_BUSY };
enum { RETURN_SET=1, RETURN_START=2, RETURN_CANCEL=3 };
typedef struct {
    uint32_t point[RETURN_CAPACITY][4], previous[4];
    uint16_t leg_ms[RETURN_CAPACITY]; /* outbound point k-1 to k, excluding parked time */
    uint8_t entry_rpm[RETURN_CAPACITY]; /* planned speed at a joint on the reverse route */
    uint32_t boot, task, ack, last_ms, changed_ms, saved_ms, last_motion_ms;
    uint32_t segment_ms, run_ms, settled_ms, lease_ms, pulse;
    uint32_t command_boot, command_task;
    uint16_t count, index, planned_index;
    int32_t advance[4];
    float speed_limit;
    uint8_t approach_rpm; /* capped recorded rate for isolated or terminal corrections */
    uint8_t cruise_rpm; /* geometry cap; zero selects the position approach law */
    uint8_t state, reason, result, command_action;
    uint8_t initialized, ready, settled, stop_requested, count_fault_wheels;
    int8_t travel_sign[4];
    int16_t demand[4]; /* tenths of RPM in logical motor polarity */
} ReturnPath;
void ReturnPath_Init(ReturnPath *s);
void ReturnPath_Abort(ReturnPath *s, uint8_t reason);
void ReturnPath_CountFault(ReturnPath *s, uint8_t wheels);
void ReturnPath_Command(ReturnPath *s,uint32_t boot,uint32_t request,
                        uint8_t action,uint32_t task,uint32_t now);
void ReturnPath_Heartbeat(ReturnPath *s,uint32_t boot,uint32_t task,
                          uint32_t pulse,uint32_t now);
void ReturnPath_Step(ReturnPath *s,uint32_t now,const uint32_t counts[4],
                     uint8_t healthy,uint8_t centered_ready);
#endif
