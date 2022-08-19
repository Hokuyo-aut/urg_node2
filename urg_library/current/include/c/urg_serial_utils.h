#ifndef URG_SERIAL_UTILS_H
#define URG_SERIAL_UTILS_H

/*!
  \file
  \~japanese
  \brief シリアル用の補助関数
  \~english
  \brief Auxiliary functions for serial communications
  \~
  \author Satofumi KAMIMURA

  $Id$
*/


//! \~japanese シリアルポートを検索する  \~english Finds the serial port
extern int urg_serial_find_port(void);


//! \~japanese 検索したシリアルポート名を返す  \~english Returns the name of the serial port found
extern const char *urg_serial_port_name(int index);


/*!
  \~japanese
  \brief ポートが URG かどうか

  \retval 1 URG のポート
  \retval 0 不明
  \retval <0 エラー

  \~english
  \brief Checks whether the serial port corresponds to a URG or not

  \retval 1 It is a URG
  \retval 0 Unknown
  \retval <0 Error
*/
extern int urg_serial_is_urg_port(int index);

#endif /* !URG_SERIAL_UTILS_H */
