#!/bin/bash

for mode in delay repeat mimic confusion noise spoof freeze
do
    current=`date "+%Y-%m-%d_%H-%M-%S"`
    echo "start $mode at $current"
    cp $mode /usr/bin
    cd /usr/share/electrosense/
    $mode -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning 24000000 1120000000 & PID=$!
    sleep 8.5m
    kill $PID
    sleep 5
    cd ~/sensor-robin/attacks_with_output
done
