#!/bin/bash

#MODE="normal"
#OUTPUT_FILE="$MODE_$(date +%s).txt"

set -e

cmake . 
make
cp es_sensor /usr/bin
#mv /usr/bin/es_sensor /usr/bin/"$MODE"
cd /usr/share/electrosense/
pwd
strace -o ~/sensor-robin/output.txt es_sensor -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -g 32.8 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning 24000000 1766000000
#es_sensor -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning 24000000 1766000000