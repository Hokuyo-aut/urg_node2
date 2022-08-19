/*!
  \~japanese
  \example angle_convert_test.c Šp“x•ÏŠ·‚ÌŒ‹‰Ê‚ð•\Ž¦‚·‚é
  \~english
  \example angle_convert_test.c Some cases of angle conversion
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdio.h>


int main(int argc, char *argv[])
{
    urg_t urg;
    int min_step;
    int max_step;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    urg_step_min_max(&urg, &min_step, &max_step);

    printf("urg_step2deg(%d): %f\n", min_step, urg_step2deg(&urg, min_step));
    printf("urg_step2deg(%d): %f\n", max_step, urg_step2deg(&urg, max_step));

    printf("urg_step2rad(%d): %f\n", min_step, urg_step2rad(&urg, min_step));
    printf("urg_step2rad(%d): %f\n", max_step, urg_step2rad(&urg, max_step));

    return 0;
}
