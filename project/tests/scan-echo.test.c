#include "ScanEcho.h"
#include <assert.h>
#include <stdio.h>
static void check(float a, float b, float c, unsigned expected_pings, float expected)
{
    ScanEcho scan = {{0}, 0};
    const float input[] = {a, b, c};
    float result = -99;
    unsigned i;
    for (i = 0; i < expected_pings; ++i)
        assert(ScanEcho_Add(&scan, input[i], &result) == (i+1 == expected_pings));
    assert(result == expected);
}
int main(void)
{
    check(2, 2, 0, 2, 2);
    check(400, 400, 0, 2, 400);
    check(-1, -1, 0, 2, -1);
    check(140, 35, 142, 3, 141);
    check(-1, 250, 252, 3, 251);
    check(35, 90, 300, 3, -1);
    check(50, -1, -1, 3, -1);
    puts("PASS stationary-angle echo confirmation, retries, outliers and no return");
    return 0;
}
