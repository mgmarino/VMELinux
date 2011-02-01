#!/bin/sh
ISSUEFILE=/etc/issue
if grep -q "Fedora" ${ISSUEFILE} > /dev/null ; then echo "rh"
elif grep -q "Red Hat" ${ISSUEFILE} > /dev/null ; then echo "rh"
elif grep -q "CRUX" ${ISSUEFILE} > /dev/null ; then echo "crux"
# interesting, slackware uses sysv style for init
elif grep -q "\\\\s \\\\r" ${ISSUEFILE} > /dev/null ; then echo "sysv"
fi
