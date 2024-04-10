#!/bin/python3

import sys
import pandas
import multiprocessing.pool
import matplotlib.pyplot as plt

values = []
lower_limit = 0.220
upper_limit = 0.250

interval = 0.0000001

periods = []


def task(chunk):
    internal_values = []
    for index, row in chunk.iterrows():
        internal_values.append(abs(float(row["Channel 7"])))
    return internal_values

def main():
    if len(sys.argv) != 2:
        print("Invalid amount of arguments. Usage: frequncy.py <file>.csv")
        exit(0)
    filename = sys.argv[1]

    pool = multiprocessing.pool.Pool()
    
    print("before open")
    with open(filename, newline='') as file:
        print("before read")
        reader = pandas.read_csv(file, delimiter=',', chunksize=10**6)
        print("after read")
        for result in pool.map(task, reader):
            print(result)
            #for chunk in reader:

            #for index, row in chunk.iterrows():
            #values.append(abs(float(row["Channel 7"])))
    exit(1)
    is_high = False
    current_stamps = 0
    max_stamps = 0
    begin_stamp = 0

    values.pop(0)

    for value in values:
        if is_high and (float(value) < lower_limit):
            is_high = False
            periods.append((begin_stamp*interval, 1, current_stamps*interval))
            current_stamps = 0
            begin_stamp = max_stamps

        if (not is_high) and (float(value) > upper_limit):
            is_high = True
            periods.append((begin_stamp*interval, 0, current_stamps*interval))
            current_stamps = 0
            begin_stamp = max_stamps

        current_stamps += 1
        max_stamps += 1

    x_values = []
    y_values = []

    for (begin_time, _, period) in periods:
        x_values.append(begin_time)
        y_values.append(1/period)

    plt.plot(x_values, y_values)
    plt.xlabel('Seconds after start')
    plt.ylabel('Frequency')
    plt.ylim(0, 1000)
    plt.title('Frequencies over time')

    plt.show()


if __name__ == "__main__":
    main()
