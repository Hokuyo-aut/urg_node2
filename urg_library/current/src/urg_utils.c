/*!
  \file
  \~japanese
  \brief URG ƒZƒ“ƒT—p‚Ì•â•ŠÖ”
  \~english
  \brief Auxiliary functions for the sensor
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_utils.h"
#include "urg_errno.h"
#define _USE_MATH_DEFINES
#include <math.h>

#undef max
#undef min

static int max(int a, int b)
{
    return (a > b) ? a : b;
}


static int min(int a, int b)
{
    return (a < b) ? a : b;
}


const char *urg_error(const urg_t *urg)
{
    typedef struct
    {
        int no;
        const char* message;
    } error_messages_t;


    error_messages_t errors[] = {
        { URG_NO_ERROR, "no error." },
        { URG_UNKNOWN_ERROR, "unknown error." },
        { URG_NOT_CONNECTED, "not connected." },
        { URG_NOT_IMPLEMENTED, "not implemented." },
        { URG_INVALID_RESPONSE, "invalid response." },
        { URG_NO_RESPONSE, "no response." },

        { URG_SEND_ERROR, "send error." },
        { URG_RECEIVE_ERROR, "receive error." },
        { URG_CHECKSUM_ERROR, "checksum error." },
        { URG_INVALID_PARAMETER, "invalid parameter." },
        { URG_MEASUREMENT_TYPE_MISMATCH, "measurement type mismatch." },

        { URG_SERIAL_OPEN_ERROR, "could not open serial device." },
        { URG_NOT_DETECT_BAUDRATE_ERROR, "could not detect serial baudrate." },
        { URG_ETHERNET_OPEN_ERROR, "could not open ethernet port." },
        { URG_SCANNING_PARAMETER_ERROR, "scanning parameter error." },
        { URG_DATA_SIZE_PARAMETER_ERROR, "data size parameter error." },
    };

    int n = sizeof(errors) / sizeof(errors[0]);
    int i;

    for (i = 0; i < n; ++i) {
        if (errors[i].no == urg->last_errno) {
            return errors[i].message;
        }
    }

    return "Unknown error.";
}


void urg_distance_min_max(const urg_t *urg,
                          long *min_distance, long *max_distance)
{
    if (!urg->is_active) {
        *min_distance = 1;
        *max_distance = 0;
        return;
    }

    *min_distance = urg->min_distance;

    // \~japanese urg_set_measurement_data_size() ‚ğ”½‰f‚µ‚½‹——£‚ğ•Ô‚·
    // \~english returns the size configured with urg_set_measurement_data_size()
    *max_distance =
        (urg->range_data_byte == URG_COMMUNICATION_2_BYTE) ?
        max(urg->max_distance, 4095) : urg->max_distance;
}


void urg_step_min_max(const urg_t *urg, int *min_index, int *max_index)
{
    if (!urg->is_active) {
        *min_index = 1;
        *max_index = 0;
        return;
    }

    *min_index = urg->first_data_index - urg->front_data_index;
    *max_index = urg->last_data_index - urg->front_data_index;
}


long urg_scan_usec(const urg_t *urg)
{
    if (!urg->is_active) {
        return URG_NOT_CONNECTED;
    }

    return urg->scan_usec;
}


int urg_max_data_size(const urg_t *urg)
{
    if (!urg->is_active) {
        return URG_NOT_CONNECTED;
    }
    return urg->last_data_index + 1;
}


double urg_index2rad(const urg_t *urg, int index)
{
    int actual_index;
    int step;

    if (!urg->is_active) {
        return URG_NOT_CONNECTED;
    }

    actual_index = min(max(0, index), urg->last_data_index);

    // \~japanese scanning_skip_step = 0 ‚Ì‚Æ‚«‚Í scanning_skip_step = 1 ‚Æ‚İ‚È‚·
    // \~english  "scanning_skip_step = 0" is equivalent to "scanning_skip_step = 1"
    step = actual_index * max(1, urg->scanning_skip_step) - urg->front_data_index + urg->received_first_index;
    
    return urg_step2rad(urg, step);
}


double urg_index2deg(const urg_t *urg, int index)
{
    return urg_index2rad(urg, index) * 180.0 / M_PI;
}


int urg_rad2index(const urg_t *urg, double radian)
{
    int index;

    if (!urg->is_active) {
        return URG_NOT_CONNECTED;
    }

    index =
        (int)(floor((urg->area_resolution * radian / (2.0 * M_PI) + 0.5)))
        + urg->front_data_index;

    return min(max(0, index), urg->last_data_index);
}


int urg_deg2index(const urg_t *urg, double degree)
{
    return urg_rad2index(urg, degree * M_PI / 180.0);
}


int urg_rad2step(const urg_t *urg, double radian)
{
    if (!urg->is_active) {
        return URG_NOT_CONNECTED;
    }

    return urg_rad2index(urg, radian) - urg->front_data_index;
}


int urg_deg2step(const urg_t *urg, double degree)
{
    return urg_rad2step(urg, degree * M_PI / 180.0);
}


double urg_step2rad(const urg_t *urg, int step)
{
    if (!urg->is_active) {
        return URG_NOT_CONNECTED;
    }

    return (2.0 * M_PI) * step / urg->area_resolution;
}


double urg_step2deg(const urg_t *urg, int step)
{
    return urg_step2rad(urg, step) * 180.0 / M_PI;
}


int urg_step2index(const urg_t *urg, int step)
{
    int measure_step;

    if (!urg->is_active) {
        return URG_NOT_CONNECTED;
    }

    measure_step = step - urg->received_first_index;
    return min(max(0, measure_step + urg->front_data_index),
               urg->last_data_index);
}

void urg_delay(int delay_msec)
{
#if defined(URG_WINDOWS_OS)
    Sleep(delay_msec);
#else
    usleep(1000 * delay_msec);
#endif
}
