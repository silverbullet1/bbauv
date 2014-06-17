#!/bin/bash
tokill = ps -ef | grep acoustic | awk '{print $2}'
