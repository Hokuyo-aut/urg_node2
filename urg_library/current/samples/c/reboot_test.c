/*!
  \example reboot_test.c reboot test

  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdio.h>


int main(int argc, char *argv[])
{
    enum { DATA_SIZE = 1081 };
    urg_t urg;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    urg_reboot(&urg);

    return 0;
}
