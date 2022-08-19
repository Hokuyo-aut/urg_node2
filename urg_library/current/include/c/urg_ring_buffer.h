#ifndef URG_RING_BUFFER_H
#define URG_RING_BUFFER_H

/*!
  \file
  \~japanese
  \brief リングバッファ
  \~english
  \brief Ring buffer functions
  \~
  \author Satofumi KAMIMURA

  $Id$
*/


//! \~japanese リングバッファの管理情報  \~english Control structure of the ring buffer
typedef struct
{
    char *buffer;                 //!< \~japanese バッファへのポインタ  \~english Pointer to the data buffer
    int buffer_size;              //!< \~japanese バッファサイズ  \~english Buffer size
    int first;                    //!< \~japanese バッファの先頭位置  \~english Index of the first entry of the buffer
    int last;                     //!< \~japanese バッファの最終位置  \~english Index of the last entry of the buffer
} ring_buffer_t;


/*!
  \~japanese
  \brief 初期化

  \param[in] ring リングバッファの構造体
  \param[in] buffer 割り当てるバッファ
  \param[in] shift_length バッファサイズの 2 の乗数

  \~english
  \brief Initialization

  \param[in] ring Pointer to the ring buffer data structure
  \param[in] buffer Actual buffer to use
  \param[in] shift_length Buffer size as multiple of 2
*/
extern void ring_initialize(ring_buffer_t *ring,
                            char *buffer, const int shift_length);


/*!
  \~japanese
  \brief リングバッファのクリア

  \param[in] ring リングバッファの構造体

  \~english
  \brief Clears the ring  buffer

  \param[in] ring Pointer to the ring buffer data structure
*/
extern void ring_clear(ring_buffer_t *ring);


/*!
  \~japanese
  \brief 格納データ数を返す

  \param[in] ring リングバッファの構造体

  \~english
  \brief Returns the number of elements on the buffer

  \param[in] ring Pointer to the ring buffer data structure
*/
extern int ring_size(const ring_buffer_t *ring);


/*!
  \~japanese
  \brief 最大の格納データ数を返す

  \param[in] ring リングバッファの構造体

  \~english
  \brief Returns the maximum number of elements of the ring buffer

  \param[in] ring Pointer to the ring buffer data structure
*/
extern int ring_capacity(const ring_buffer_t *ring);


/*!
  \~japanese
  \brief データの格納

  \param[in] ring リングバッファの構造体
  \param[in] data データ
  \param[in] size データサイズ

  \return 格納したデータ数

  \~english
  \brief Stores data on the ring buffer

  \param[in] ring Pointer to the ring buffer data structure
  \param[in] data Data to store
  \param[in] size Number of elements to store

  \return The number of elements written to the ring buffer
*/
extern int ring_write(ring_buffer_t *ring, const char *data, int size);


/*!
  \~japanese
  \brief データの取り出し

  \param[in] ring リングバッファの構造体
  \param[out] buffer データ
  \param[in] size 最大のデータサイズ

  \return 取り出したデータ数

  \~english
  \brief Extracts data from the ring buffer

  \param[in] ring Pointer to the ring buffer data structure
  \param[out] buffer Buffer to hold the extracted data
  \param[in] size Maximum size of the buffer

  \return The number of elements read from the ring buffer
*/
extern int ring_read(ring_buffer_t *ring, char *buffer, int size);

#endif /* ! RING_BUFFER_H */
