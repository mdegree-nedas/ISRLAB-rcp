from noros.core import Freenove_4wd_smart_car
import random
import time


def read_another_callback():
    return [random.randint(0, 10), random.randint(0, 10), random.randint(0, 10)]


def main():
    f = Freenove_4wd_smart_car()
    f.sensors.linetracker.callback = read_another_callback
    while True:
        time.sleep(f.sensors.linetracker.time)
        msg = f.sensors.linetracker.read()
        print(msg)
        f.broker.send(f.sensors.linetracker.topic, msg)


if __name__ == "__main__":
    main()
