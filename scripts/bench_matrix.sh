#!/bin/bash

killall cat

FILENAME="results.txt"

stty -F /dev/ttyACM1 115200 cooked -echo -parenb cs8 -cstopb
cat /dev/ttyACM1 > $FILENAME &

cd ../pico/

NLINES=$(du -sb "../scripts/$FILENAME" | awk '{print $1}')

for i in {1..50}
do
   cargo rb concurrency &
   cargo_pid=$!
   while [ $(du -sb "../scripts/$FILENAME"  | awk '{print $1}') == $NLINES ]; do sleep 0.5; done
   NLINES=$(du -sb "../scripts/$FILENAME"  | awk '{print $1}')
   kill $cargo_pid
done

sed -i 's/[?]//g' ../scripts/$FILENAME

