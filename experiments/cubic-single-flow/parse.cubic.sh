#!/bin/bash

cat files-1/var/log/messages | awk '{ print $2, $3, $4, $5}' | grep cwnd | awk '{ print ($1 - 300)"."$2, $4, $5}' | uniq > tcp-cwnd.dat
