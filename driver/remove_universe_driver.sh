#!/bin/sh
module="universe"
device="vme_"
mode="666"

# invoke insmod with all arguments we were passed
# and use a pathname, as newer modutils don't look in . by default
lsmod | grep -q $module
if [ "${?}" != 0 ]; then
  printf "%s not found...\n" $module
  printf "Exiting.\n"
  exit 1
fi 
# remove stale nodes
rm -f /dev/${device}m[0-8]
rm -f /dev/${device}ctl
rm -f /dev/${device}dma

/sbin/rmmod  $module || exit $?


