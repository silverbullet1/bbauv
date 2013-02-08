#!/bin/sh

# Add this script to your list of startup applications

# Trigger the OpenUPS devices based on vendor and product IDs
# (because they aren't triggered at startup)
echo bb | sudo -S udevadm trigger --attr-match=idVendor=04d8 --attr-match=idProduct=d004
