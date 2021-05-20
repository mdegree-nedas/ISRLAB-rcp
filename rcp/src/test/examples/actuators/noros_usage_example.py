from noros.core import Freenove_4wd_smart_car


def another_callback(data):
    print("motion.go_forward another callback")
    print(data)


def main():
    f = Freenove_4wd_smart_car()
    # f.actuators.motion.commands.go_forward.callback = print_roba
    f.broker.receive()
    input()


if __name__ == "__main__":
    main()
