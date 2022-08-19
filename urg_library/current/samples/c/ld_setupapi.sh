#!/bin/sh

if [ "${MSYSTEM}" = "MINGW32" ] ; then
  echo "-lsetupapi"
fi
