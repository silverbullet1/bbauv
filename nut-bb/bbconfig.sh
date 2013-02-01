#!/bin/bash

./configure --with-usb --without-ssl --without-cgi --without-snmp --without-serial --without-dev --enable-static --with-statepath=/var/run/nut --with-altpidpath=/var/run/nut --with-drvpath=/lib/nut --with-cgipath=/usr/lib/cgi-bin/nut --with-htmlpath=/usr/share/nut/www --with-pidpath=/var/run/nut --datadir=/usr/share/nut --with-pkgconfig-dir=/usr/lib/pkgconfig --with-user=nut --with-group=nut --sysconfdir=/etc/nut --with-udev-dir=/etc/udev

