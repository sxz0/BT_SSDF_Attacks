#!/bin/bash
set -e

cmake . 
make
cp es_sensor /usr/bin
cd /usr/share/electrosense/
#strace -o ~/sensor-robin/output.txt es_sensor -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -g 32.8 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning 24000000 1766000000
es_sensor -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning -v normal -j 200000000 24000000 1766000000 200000000 300000000