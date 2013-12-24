#!/bin/bash

# Script to install extra tools that 12.04 lacks
sudo apt-add-repository ppa:kalakris/cmake # For backport of cmake 2.8.11

sudo apt-get update
sudo apt-get install cmake
