#!/bin/bash

# for mode in repeat mimic confusion noise spoof freeze
# do 
#     service electrosense-sensor-mqtt stop
#     cp $mode /usr/bin/es_sensor
#     sleep 5
#     service electrosense-sensor-mqtt start
#     sleep 5
#     pid=$(ps aux | grep es_sensor | grep -v sudo | grep -v grep | awk '{print $2}')
#     echo $pid
#     timeout -s 1 5m perf trace -o /data/$mode.txt -e !nanosleep -T -p $pid
# done

service electrosense-sensor-mqtt stop
cp repeat /usr/bin/es_sensor
sleep 5
service electrosense-sensor-mqtt start
sleep 5
pid=$(ps aux | grep es_sensor | grep -v sudo | grep -v grep | awk '{print $2}')
echo $pid
timeout -s 1 5m perf trace -o /data/repeat.txt -e \!nanosleep -T -p $pid