#!/bin/sh
module="universe"
device="vme_"
mode="666"
version=`uname -r`
kernel=$version

# invoke insmod with all arguments we were passed
# and use a pathname, as newer modutils don't look in . by default
# Available options:
#   size_to_reserve=(Give the size of pci space to reserve)
#   reserve_from_address=(Give starting pci address from which to reserve.)
/sbin/insmod /lib/modules/$kernel/kernel/drivers/vme/$module.ko $* || exit 1


# remove stale nodes
printf "Removing stale nodes...\n"
rm -f /dev/${device}m[0-8]
rm -f /dev/${device}ctl
rm -f /dev/${device}dma

major=`awk "\\$2==\"$module\" {print \\$1}" /proc/devices`

printf "Installing nodes (major: %d)...\n" $major
mknod /dev/${device}m0 c $major 0
mknod /dev/${device}m1 c $major 1
mknod /dev/${device}m2 c $major 2
mknod /dev/${device}m3 c $major 3
mknod /dev/${device}m4 c $major 4
mknod /dev/${device}m5 c $major 5
mknod /dev/${device}m6 c $major 6
mknod /dev/${device}m7 c $major 7
mknod /dev/${device}ctl c $major 8
mknod /dev/${device}dma c $major 9

# give appropriate group/permissions, and change the group.
# Not all distributions have staff; some have "wheel" instead.
group="staff"
grep '^staff:' /etc/group > /dev/null || group="wheel"

chgrp $group /dev/${device}m[0-8]
chgrp $group /dev/${device}ctl
chgrp $group /dev/${device}dma
chmod $mode /dev/${device}m[0-8]
chmod $mode /dev/${device}ctl
chmod $mode /dev/${device}dma
#chmod 600 /dev/${device}m3
#chmod 600 /dev/${device}m7

printf "%s driver installation complete.\n" $module
