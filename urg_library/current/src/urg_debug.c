/*!
  \~japanese
  \brief URG ƒZƒ“ƒT—p‚Ì•â•ŠÖ”
  \~english Auxiliary functions (read and write raw data)
  \brief 
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_debug.h"


int urg_raw_write(urg_t *urg, const char *data, int data_size)
{
    return connection_write(&urg->connection, data, data_size);
}


int urg_raw_read(urg_t *urg, char *data, int max_data_size, int timeout)
{
    return connection_read(&urg->connection, data, max_data_size, timeout);
}


int urg_raw_readline(urg_t *urg, char *data, int max_data_size, int timeout)
{
    return connection_readline(&urg->connection, data, max_data_size, timeout);
}
