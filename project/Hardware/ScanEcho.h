#ifndef SCAN_ECHO_H
#define SCAN_ECHO_H
/* Called once per completed ping at an unchanged automatic scan angle.
 * A negative sample denotes timeout or out-of-range. */
typedef struct {
    float samples[2];
    unsigned count;
} ScanEcho;

static int ScanEcho_Add(ScanEcho *batch, float sample, float *result)
{
    unsigned i;
    for (i = 0; i < batch->count; ++i) {
        float previous = batch->samples[i];
        float difference = sample - previous;
        float nearer = sample < previous ? sample : previous;
        float tolerance = 3.0f + 0.02f * nearer;
        if (nearer >= 2.0f && difference >= -tolerance && difference <= tolerance) {
            *result = (previous + sample) * 0.5f;
            return 1;
        }
    }
    if (batch->count == 2 || (batch->count == 1 && sample < 0 && batch->samples[0] < 0)) {
        *result = -1.0f;
        return 1;
    }
    batch->samples[batch->count++] = sample;
    return 0;
}
#endif
