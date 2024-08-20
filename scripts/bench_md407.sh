#!/bin/bash

killall cat

FILENAME="results.txt"


cd ../md407
cargo bbr matrix
arm-none-eabi-objcopy -S -O srec "./target/thumbv7em-none-eabihf/release/matrix" app.s19 &&

stty -F /dev/ttyACM0 115200 cooked -echo -parenb cs8 -cstopb
stty -F /dev/ttyUSB0 115200 cooked -echo -parenb cs8 -cstopb

rm "../scripts/$FILENAME"
touch "../scripts/$FILENAME"

NLINES=$(du -sb "../scripts/$FILENAME" | awk '{print $1}')

for i in {1..50}
do
    echo "Loading..."
    echo 'load' > /dev/ttyUSB0 &&
    sleep 0.5 &&
    cat app.s19 > /dev/ttyUSB0 &&
    sleep 0.5
    
    cat /dev/ttyUSB0 >> "../scripts/$FILENAME" &
    read_pid=$!
    sleep 1
    echo "Loaded!"
    echo 'go' > /dev/ttyUSB0
    NLINES=$(du -sb "../scripts/$FILENAME" | awk '{print $1}')
    
    while [ $(du -sb "../scripts/$FILENAME"  | awk '{print $1}') == $NLINES ]; do sleep 0.1; done
    kill $read_pid
    NLINES=$(du -sb "../scripts/$FILENAME"  | awk '{print $1}') 
    sleep 0.8
done

sed -i 's/[?]//g' ../scripts/$FILENAME




