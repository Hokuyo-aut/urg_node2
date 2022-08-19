/*!
  \example timeout_test.c timeout test

  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_debug.h"
#include "open_urg_sensor.h"
#include <stdio.h>


int main(int argc, char *argv[])
{
    enum { TIMEOUT_MSEC = 3000 };
    char buffer;
    int n;
    urg_t urg;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    printf("read\n");
    n = urg_raw_read(&urg, &buffer, 1, TIMEOUT_MSEC);
    printf("n = %d\n", n);

    return 0;
}
