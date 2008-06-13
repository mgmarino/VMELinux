#!/bin/sh
CURRENTOS=`uname -r | awk -F'.' '{print $NF}'`

if echo "$CURRENTOS" | grep "fc" > /dev/null ; then
  echo "rh"
elif echo "$CURRENTOS" | grep "rh" > /dev/null ; then
  echo "rh"
elif echo "$CURRENTOS" | grep "crux" > /dev/null; then
  echo "crux"
fi
