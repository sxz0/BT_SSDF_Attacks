#!/bin/bash
echo "Don't forget to start logging."
sleep 5

for time in 30
do 
    for bandwidth in 800000000
    do 
        if [[ ($bandwidth -gt 2000000) ]]; then
            component="rtlsdr"
        else 
            component="fft"
        fi 

        current=`date "+%Y-%m-%d_%H-%M-%S"`;
        folder="${current}_${component}_${bandwidth}_${time}"
        mkdir -p /data/$folder/raw
        mkdir -p /data/logs
        touch /data/logs/$folder.log

        for mode in normal repeat mimic confusion noise spoof freeze delay stop
        do
            mount -o remount rw /
            if [ $mode == "stop" ]
                then
                    #STOP MONITORING
                    echo "Stopping monitoring script..."
                    pid=$(ps aux | grep es_sensor | grep -v sudo | grep -v grep | awk '{print $2}')
                    kill $pid
                    echo "Done with configuration [time: $time, bandwidth: $bandwidth]."
            else
                #SET MALICIOUS EXECUTABLE AND START MONITORING
                pid=$(ps aux | grep es_sensor | grep -v sudo | grep -v grep | awk '{print $2}')
                if [[ ! $pid == "" ]]; then 
                    echo "es_sensor still running. Killing the process"
                    kill $pid
                fi
                sleep 5
                cd /usr/share/electrosense/
                echo "Starting es_sensor executable with new behavior for $mode with a bandwidth of $bandwidth for $time seconds."
                es_sensor -d 0 -q 0.167 -c 0 -r 0 -y sequential -s 2400000 -a 5 -b 10 -m 0 -k 3600 -t 0 -f 8 -n collector.electrosense.org:5001#certs/CA-Cert.pem#certs/Sensor-SSL-Cert.pem#certs/Sensor-SSL-SK.pem -o 128 -w hanning -v $mode -j $bandwidth 24000000 1766000000 50000000 900000000 >>/data/logs/$folder.log 2>&1 &
                $HOME/sensor-robin/attacks/monitor.sh $mode $folder $bandwidth $time 
                echo "Done with monitoring mode $mode."
                echo "================================================================="
            fi
        done
        echo "Switching bandwidth"
    done
    echo "Switching time"
done 

echo "Finished!"
