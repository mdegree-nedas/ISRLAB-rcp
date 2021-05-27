from noros.core import Freenove_4wd_smart_car
import random
import time


def another_callback_for_go_forward(data):
    ros_cmd = data["command"]
    twist_linear = data["data"]["linear"]
    twist_angular = data["data"]["angular"]
    print("ros command: {}\nros twist payload linear:\n- {}\nros twist payload angular:\n- {}\n".format(ros_cmd, twist_linear, twist_angular))

def another_callback_for_turn_left(data):
    ros_cmd = data["command"]
    twist_linear = data["data"]["linear"]
    twist_angular = data["data"]["angular"]
    print("ros command: {}\nros twist payload linear:\n- {}\nros twist payload angular:\n- {}\n".format(ros_cmd, twist_linear, twist_angular))

def another_callback_for_turn_right(data):
    ros_cmd = data["command"]
    twist_linear = data["data"]["linear"]
    twist_angular = data["data"]["angular"]
    print("ros command: {}\nros twist payload linear:\n- {}\nros twist payload angular:\n- {}\n".format(ros_cmd, twist_linear, twist_angular))


def read_sensor_another_callback():
    return random.choice([[0,0,0],[1,0,0],[0,1,0],[0,0,1]])

expected_ros_cmd = {repr([0,0,0]): 'idle (no command)', repr([1,0,0]): 'turn_left', repr([0,1,0]): 'go_forward', repr([0,0,1]): 'turn_right'}

def main():
    f = Freenove_4wd_smart_car()
    f.sensors.linetracker.callback = read_sensor_another_callback
    f.actuators.motion.commands.go_forward.callback = another_callback_for_go_forward
    f.actuators.motion.commands.turn_left.callback = another_callback_for_turn_left
    f.actuators.motion.commands.turn_right.callback = another_callback_for_turn_right
    f.broker.receive()
    while True:
        time.sleep(f.sensors.linetracker.time)
        msg = f.sensors.linetracker.read()
        print('sending sensor data to ros: ', msg, '(expected ros command: {})'.format(expected_ros_cmd[repr(msg)]))
        f.broker.send(f.sensors.linetracker.topic, msg)


if __name__ == "__main__":
    main()
