/*!
  \~japanese
  \example get_distance.c 距離データを取得する
  \~english
  \example get_distance.c Obtains distance data
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"
#include <stdlib.h>
#include <stdio.h>


static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
#if 1
    int front_index;

    (void)data_n;

    // \~japanese 前方のデータのみを表示
    // \~english Shows only the front step
    front_index = urg_step2index(urg, 0);
    printf("%ld [mm], (%ld [msec])\n", data[front_index], time_stamp);

#else
    (void)time_stamp;

    int i;
    long min_distance;
    long max_distance;

    // \~japanese 全てのデータの X-Y の位置を表示
    // \~english Prints the X-Y coordinates for all the measurement points
    urg_distance_min_max(urg, &min_distance, &max_distance);
    for (i = 0; i < data_n; ++i) {
        long l = data[i];
        double radian;
        long x;
        long y;

        if ((l <= min_distance) || (l >= max_distance)) {
            continue;
        }
        radian = urg_index2rad(urg, i);
        x = (long)(l * cos(radian));
        y = (long)(l * sin(radian));
        printf("(%ld, %ld), ", x, y);
    }
    printf("\n");
#endif
}


int main(int argc, char *argv[])
{
    enum {
        CAPTURE_TIMES = 10,
    };
    urg_t urg;
    long *data = NULL;
    long time_stamp;
    int n;
    int i;

    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return 1;
    }

    data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    if (!data) {
        perror("urg_max_index()");
        return 1;
    }

    // \~japanese データ取得
    // \~english Gets measurement data
#if 0
    // \~japanese データの取得範囲を変更する場合
    // \~english Case where the measurement range (start/end steps) is defined
    urg_set_scanning_parameter(&urg,
                               urg_deg2step(&urg, -90),
                               urg_deg2step(&urg, +90), 0);
#endif

    urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);
    for (i = 0; i < CAPTURE_TIMES; ++i) {
        n = urg_get_distance(&urg, data, &time_stamp);
        if (n <= 0) {
            printf("urg_get_distance: %s\n", urg_error(&urg));
            free(data);
            urg_close(&urg);
            return 1;
        }
        print_data(&urg, data, n, time_stamp);
    }

    // \~japanese 切断
    // \~english Disconnects
    free(data);
    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
