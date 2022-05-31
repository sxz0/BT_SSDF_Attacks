#!/bin/bash

echo "Start monitoring script"

for mode in normal repeat mimic confusion noise spoof freeze delay stop
do
    mount -o remount rw /
    if [ $mode == "stop" ]
		then
			#STOP MONITORING
			echo "Stopping monitoring script..."
			service electrosense-sensor-mqtt stop
            echo "Done."
    else
        #SET MALICIOUS EXECUTABLE AND START MONITORING
        echo "Restarting es_sensor executable with new behavior for $mode"
        service electrosense-sensor-mqtt stop
        cp $mode /usr/bin/es_sensor
        service electrosense-sensor-mqtt start
        sleep 10
        echo "Starting monitoring script"
        ./monitor.sh $mode &
        # wait until get trace finished
        wait
        echo "Done."
        echo "================================================================="
        sleep 10
    fi
done
echo "finished!!!"
