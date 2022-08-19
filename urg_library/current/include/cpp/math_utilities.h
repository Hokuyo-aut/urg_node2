#ifndef QRK_MATH_UTILITIES_H
#define QRK_MATH_UTILITIES_H

/*!
  \file
  \~japanese
  \brief 数学関数の補助ファイル
  \~english
  \brief Auxiliary math functions
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "detect_os.h"
#if defined(WINDOWS_OS)
#define _USE_MATH_DEFINES
#endif
#include <math.h>


#ifndef M_PI
//! \~japanese 円周率 (Visual C++ 6.0 用)
//! \~english PI approximation (for Visual C++ 6.0)
#define M_PI 3.14159265358979323846
#endif

#if defined(MSC)
extern long lrint(double x);
#endif

#endif /* !QRK_MATH_UTILITIES_H */
