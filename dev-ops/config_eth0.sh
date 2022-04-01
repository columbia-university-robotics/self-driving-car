#!/bin/bash

# Must be run as root. Will permanently 

cat > /etc/network/interfaces << EOF
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

auto eth0
iface eth0 inet static
        address 192.168.1.102
        netmask 255.255.255.0
EOF
