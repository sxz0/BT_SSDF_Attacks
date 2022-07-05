#!/bin/bash
for bandwidth in 20000 20000000
do 
    for mode in normal delay repeat mimic confusion noise spoof freeze
    do
        current=`date "+%Y-%m-%d_%H-%M-%S"`
        echo "Start $mode width bandwidth=${bandwidth}Hz at $current"
        echo "---------------------------------------------------------------------------------------"
        cd /usr/share/electrosense/
        es_sensor -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning -v $mode -j $bandwidth 24000000 1766000000 200000000 300000000 & PID=$!
        sleep 60s
        kill $PID
        sleep 5
        echo "Stopped $mode"
        echo "---------------------------------------------------------------------------------------"
    done
    echo "---------------------------------------------------------------------------------------"
    echo "SWITCHING BANDWIDTH"
    echo "---------------------------------------------------------------------------------------"
done

echo "Finished"
