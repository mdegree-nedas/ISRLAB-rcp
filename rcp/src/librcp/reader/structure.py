from itertools import chain
import re


class Structure:
    def __init__(self):
        self._initialize()

    def _initialize(self):
        self._cfg_default_classes = [
            "message_broker_ip",
            "sensors",
            "actuators",
            "commands",
        ]
        self._cfg_default_sensors_fields = ["id", "type", "address", "topic", "time"]
        self._cfg_default_actuators_fields = ["id", "address", "topic", "commands"]
        self._cfg_default_commands_fields = ["data", "time"]

        self._cfg_vector_name = []
        self._cfg_vector_classes = []
        self._cfg_vector_sensors = []
        self._cfg_vector_sensors_fields = []
        self._cfg_vector_actuators = []
        self._cfg_vector_actuators_commands = []
        self._cfg_vector_actuators_fields = []
        self._cfg_vector_commands = []
        self._cfg_vector_commands_fields = []

    def cfg_validate(self, cfg_dict):
        self._initialize()
        self._cfg_vector_name = list(cfg_dict.keys())
        assert len(self._cfg_vector_name) == 1
        name_k = self._cfg_vector_name[0]

        self._cfg_vector_classes = list(cfg_dict[name_k].keys())
        assert len(self._cfg_vector_classes) == 4
        assert self._cfg_vector_classes == self._cfg_default_classes

        mb_ip_k, sensors_k, actuators_k, commands_k = self._cfg_vector_classes[:]

        assert type(mb_ip_k) == str
        ip_regex = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$")
        assert ip_regex.fullmatch(cfg_dict[name_k][mb_ip_k]) != None

        self._cfg_vector_sensors = list(cfg_dict[name_k][sensors_k].keys())

        for sensor in self._cfg_vector_sensors:
            self._cfg_vector_sensors_fields = list(
                cfg_dict[name_k][sensors_k][sensor].keys()
            )
            assert len(self._cfg_vector_sensors_fields) == 5
            assert self._cfg_vector_sensors_fields == self._cfg_default_sensors_fields

        self._cfg_vector_actuators = list(cfg_dict[name_k][actuators_k].keys())

        for actuator in self._cfg_vector_actuators:
            self._cfg_vector_actuators_fields = list(
                cfg_dict[name_k][actuators_k][actuator].keys()
            )
            self._cfg_vector_actuators_commands.append(
                list(cfg_dict[name_k][actuators_k][actuator]["commands"])
            )
            assert len(self._cfg_vector_actuators_fields) == 4
            assert (
                self._cfg_vector_actuators_fields == self._cfg_default_actuators_fields
            )

        self._cfg_vector_actuators_commands = list(
            chain.from_iterable(self._cfg_vector_actuators_commands)
        )
        self._cfg_vector_commands = list(cfg_dict[name_k][commands_k].keys())
        assert self._cfg_vector_commands == self._cfg_vector_actuators_commands

        for command in self._cfg_vector_commands:
            self._cfg_vector_commands_fields = list(
                cfg_dict[name_k][commands_k][command].keys()
            )
            assert len(self._cfg_vector_commands_fields) == 2
            assert self._cfg_vector_commands_fields == self._cfg_default_commands_fields

        for sensor in self._cfg_vector_sensors:
            assert type(cfg_dict[name_k][sensors_k][sensor]["id"]) == str
            assert type(cfg_dict[name_k][sensors_k][sensor]["type"]) == str
            assert type(cfg_dict[name_k][sensors_k][sensor]["address"]) == str
            assert type(cfg_dict[name_k][sensors_k][sensor]["topic"]) == str
            assert type(cfg_dict[name_k][sensors_k][sensor]["time"]) == float

        for actuator in self._cfg_vector_actuators:
            assert type(cfg_dict[name_k][actuators_k][actuator]["id"]) == str
            assert type(cfg_dict[name_k][actuators_k][actuator]["address"]) == str
            assert type(cfg_dict[name_k][actuators_k][actuator]["topic"]) == str
            assert type(cfg_dict[name_k][actuators_k][actuator]["commands"]) == list

        for command in self._cfg_vector_commands:
            assert type(cfg_dict[name_k][commands_k][command]["data"]) == str
            assert type(cfg_dict[name_k][commands_k][command]["time"]) == float

        return [
            name_k,
            sensors_k,
            self._cfg_vector_sensors,
            actuators_k,
            self._cfg_vector_actuators,
            commands_k,
            self._cfg_vector_commands,
        ]
