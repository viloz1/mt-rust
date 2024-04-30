
#!/bin/bash

killall cat

FILENAME="results.txt"

stty -F /dev/ttyACM0 115200 cooked -echo -parenb cs8 -cstopb
cat /dev/ttyACM0 > $FILENAME &

cd ../stm32f07/

killall probe-rs

NLINES=$(du -sb "../scripts/$FILENAME" | awk '{print $1}')

for i in {0..2}
do
   cargo rb reaction &
   cargo_pid=$!
   sleep 21
   kill $cargo_pid
done

sed -i 's/[?]//g' ../scripts/$FILENAME

