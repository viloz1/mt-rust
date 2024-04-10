#!/bin/bash

killall cat

FILENAME="matrix.txt"

stty -F /dev/ttyACM0 115200 cooked -echo -parenb cs8 -cstopb
cat /dev/ttyACM0 > $FILENAME &

cd ../pico

NLINES=$(du -sb "../scripts/$FILENAME" | awk '{print $1}')

for i in {1..5}
do
   cargo rb matrix &
   cargo_pid=$!
   while [ $(du -sb "../scripts/$FILENAME"  | awk '{print $1}') == $NLINES ]; do sleep 0.5; done
   NLINES=$(du -sb "../scripts/$FILENAME"  | awk '{print $1}')
   kill $cargo_pid
done

sed -i 's/[?]//g' ../scripts/$FILENAME

