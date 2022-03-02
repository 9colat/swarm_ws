#!/usr/bin/env python3
from datetime import datetime
import time
old_time = datetime.now()
old_time_timestamp = datetime.timestamp(old_time)


def test():
    global old_time_timestamp
    while True:
        time_now = datetime.now()
        time_timestamp = datetime.timestamp(time_now)
        delta_time = time_timestamp - old_time_timestamp
        print("delta time is: " + str(delta_time))
        time.sleep(3)



if __name__ == '__main__':
    test()
