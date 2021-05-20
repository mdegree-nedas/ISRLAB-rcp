from pathlib import Path
import os


class RosInterfaceGenerator:
    def __init__(self):
        self._cfg_dict = None

        self._gen_name_k = None
        self._gen_sensors_k = None
        self._gen_vector_sensors = []
        self._gen_actuators_k = None
        self._gen_vector_actuators = []
        self._gen_commands_k = None
        self._gen_vector_commands = []

    def _initialize(self, cfg_dict, cfg_parse):
        self._cfg_dict = cfg_dict

        (
            self._gen_name_k,
            self._gen_sensors_k,
            self._gen_vector_sensors,
            self._gen_actuators_k,
            self._gen_vector_actuators,
            self._gen_commands_k,
            self._gen_vector_commands,
        ) = cfg_parse[:]

        self._sep = "/"
        self._ext = ".py"

        self._tab = "    "
        self._2tab = self._tab * 2
        self._3tab = self._tab * 3
        self._4tab = self._tab * 4
        self._nl = "\n"

    def _initialize_interface(self):
        self._prefix = "."
        self._destdir = "out" + self._sep + self._gen_name_k + self._sep + "ros"
        Path(self._prefix + self._sep + self._destdir).mkdir(
            parents=True, exist_ok=True
        )

        self._filename = (
            self._prefix
            + self._sep
            + self._destdir
            + self._sep
            + "interface"
            + self._ext
        )

        payload = [
            "# rcp: INTERFACE MODULE" + self._nl,
            "# rcp: auto-generated ros foreign interface file" + self._nl,
            "# rcp: EDIT this file and insert other ros2 msg interface wrappers"
            + self._nl,
            self._nl,
        ]

        f = open(self._filename, "w")
        f.writelines(payload)
        f.close()

    def generate(self, cfg_dict, cfg_parse):
        self._initialize(cfg_dict, cfg_parse)

        self._initialize_interface()
        self._gen_interface_imports()

        self._gen_interface_twist_wrapper_class()

        self._finalize()

    # ##################################################
    # GEN EXTRA

    def _gen_interface_imports(self):
        payload = [
            self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()

    def _finalize(self):
        print("finalize >   ros > " + self._filename)
        os.system("black -q " + self._filename)

    # ##################################################
    # GEN ROS GEOMETRY_MSGS/TWIST INTERFACE

    def _gen_interface_twist_wrapper_class(self):
        payload = [
            "class GeometryMsgsTwist:" + self._nl,
            self._tab + "def __init__(self, topic, command, msg_type, msg):" + self._nl,
            self._2tab + "self.topic = topic" + self._nl,
            self._2tab + "self.command = command" + self._nl,
            self._2tab + "self.msg_type = msg_type" + self._nl,
            self._2tab + "self.data = self.GeometryMsgsTwistData(msg)" + self._nl,
            self._nl,
            self._tab + "class GeometryMsgsTwistData:" + self._nl,
            self._2tab + "def __init__(self, msg):" + self._nl,
            self._3tab
            + "self.linear = self.GeometryMsgsTwistDataLinear(msg)"
            + self._nl,
            self._3tab
            + "self.angular = self.GeometryMsgsTwistDataAngular(msg)"
            + self._nl,
            self._nl,
            self._2tab + "class GeometryMsgsTwistDataLinear:" + self._nl,
            self._3tab + "def __init__(self, msg):" + self._nl,
            self._4tab + "self.x = msg.linear.x" + self._nl,
            self._4tab + "self.y = msg.linear.y" + self._nl,
            self._4tab + "self.z = msg.linear.z" + self._nl,
            self._2tab + "class GeometryMsgsTwistDataAngular:" + self._nl,
            self._3tab + "def __init__(self, msg):" + self._nl,
            self._4tab + "self.x = msg.angular.x" + self._nl,
            self._4tab + "self.y = msg.angular.y" + self._nl,
            self._4tab + "self.z = msg.angular.z" + self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()
