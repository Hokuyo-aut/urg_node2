#!/bin/sh

if [ "${MSYSTEM}" = "MINGW32" ] ; then
  echo "-lwsock32"
fi
