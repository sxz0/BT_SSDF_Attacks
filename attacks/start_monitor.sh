#!/bin/bash

for time in 600
do 
    for bandwidth in 20000 200000 2000000 20000000 200000000
    do 
        if [ "$bandwidth" -gt "2000000"]; then
            component="rtlsdr"
        else 
            component="fft"
        fi 
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
                current=`date "+%Y-%m-%d_%H-%M-%S"`;
                folder="${current}_${component}_${bandwidth}_${time}"
                mkdir /data/$folder

                #SET MALICIOUS EXECUTABLE AND START MONITORING
                echo "Restarting es_sensor executable with new behavior for $mode with a bandwidth of $bandwidth for $time seconds."
                mkdir /data/$folder/raw/
                service electrosense-sensor-mqtt stop
                sleep 5
                cd /usr/share/electrosense/
                es_sensor -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning -v $mode -j $bandwidth 24000000 1766000000 200000000 500000000 > /dev/null
                sleep 10
                cd ~/sensor-robin/attacks
                echo "Starting monitoring script"
                ./monitor.sh $mode $folder $bandwidth $time &
                # wait until get trace finished
                wait
                echo "Done."
                echo "================================================================="
                sleep 10
            fi
        done
        echo "Switching bandwidth"
    done
    echo "Switching time"
done 

echo "Finished!"