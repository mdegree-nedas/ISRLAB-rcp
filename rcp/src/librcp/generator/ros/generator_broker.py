from pathlib import Path
import os


class RosBrokerGenerator:
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
        self._nl = "\n"

    def _initialize_broker(self):
        self._prefix = "."
        self._destdir = "out" + self._sep + self._gen_name_k + self._sep + "ros"
        Path(self._prefix + self._sep + self._destdir).mkdir(
            parents=True, exist_ok=True
        )

        self._filename = (
            self._prefix + self._sep + self._destdir + self._sep + "broker" + self._ext
        )

        payload = [
            "# rcp: BROKER MODULE" + self._nl,
            "# rcp: auto-generated ros foreign broker file" + self._nl,
            "# rcp: do not edit this file" + self._nl,
            self._nl,
        ]

        f = open(self._filename, "w")
        f.writelines(payload)
        f.close()

    def generate(self, cfg_dict, cfg_parse):
        self._initialize(cfg_dict, cfg_parse)

        self._initialize_broker()
        self._gen_broker_imports()

        self._gen_broker_redis_wrapper_common()
        self._gen_broker_redis_wrapper_class()

        self._gen_broker_redis_middleware_class()
        self._gen_broker_redis_middleware_send()

        self._gen_broker_converter_class()
        self._gen_broker_converter_GeometryMsgsTwistToJson()

        self._finalize()

    # ##################################################
    # GEN EXTRA

    def _gen_broker_imports(self):
        payload = [
            "from .interface import *" + self._nl,
            "import redis" + self._nl,
            "import json" + self._nl,
            self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()

    def _finalize(self):
        print("finalize > noros > " + self._filename)
        os.system("black -q " + self._filename)

    # ##################################################
    # GEN REDIS CONVERTER

    def _gen_broker_converter_class(self):
        payload = [
            "class _Converter:" + self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()

    def _gen_broker_converter_GeometryMsgsTwistToJson(self):
        payload = [
            self._tab + "def GeometryMsgsTwistToJson(self, msg):" + self._nl,
            self._2tab
            + "return json.dumps({'topic': msg.topic, 'command': msg.command, 'msg_type': msg.msg_type, 'data': {'linear': {'x': msg.data.linear.x, 'y': msg.data.linear.y, 'z': msg.data.linear.z}, 'angular': {'x': msg.data.angular.x, 'y': msg.data.angular.y, 'z': msg.data.angular.z}}})"
            + self._nl,
            self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()

    # ##################################################
    # GEN REDIS WRAPPER

    def _gen_broker_redis_wrapper_common(self):
        payload = [
            'SERVER_ADDR = "redis"' + self._nl,
            "SERVER_PORT = 6379" + self._nl,
            "SERVER_DRDB = 0" + self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()

    def _gen_broker_redis_wrapper_class(self):
        payload = [
            "class _RedisWrapper:" + self._nl,
            self._tab
            + "def __init__(self, host=SERVER_ADDR, port=SERVER_PORT, db=SERVER_DRDB):"
            + self._nl,
            self._2tab
            + "self._redis = redis.Redis(host=host, port=port, db=db)"
            + self._nl,
            self._2tab + "self._redis_pubsub = self._redis.pubsub()" + self._nl,
            self._nl,
            self._tab + "def subscribe(self, sensors_topics):" + self._nl,
            self._2tab + "self._redis_pubsub.subscribe(**sensors_topics)" + self._nl,
            self._2tab
            + "self._redis_pubsub.run_in_thread(sleep_time=0.01, daemon=True)"
            + self._nl,
            self._nl,
            self._tab + "def publish(self, topic, msg):" + self._nl,
            self._2tab + "return self._redis.publish(topic, msg)" + self._nl,
            self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()

    # ##################################################
    # GEN REDIS MIDDLEWARE

    def _gen_broker_redis_middleware_class(self):
        payload = [
            "class RedisMiddleware:" + self._nl,
            self._tab + "def __init__(self, sensors_topics):" + self._nl,
            self._2tab + "self._redis_wrapper = _RedisWrapper()" + self._nl,
            self._2tab + "self._converter = _Converter()" + self._nl,
            self._2tab + "self._sensors_topics = sensors_topics" + self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()

    def _gen_broker_redis_middleware_send(self):
        payload = [
            self._tab + "def send(self, topic, command, msg_type, msg):" + self._nl,
            self._2tab + 'if msg_type is "twist":' + self._nl,
            self._3tab
            + "msg = GeometryMsgsTwist(topic, command, msg_type, msg)"
            + self._nl,
            self._3tab
            + "msgJson = self._converter.GeometryMsgsTwistToJson(msg)"
            + self._nl,
            self._2tab + "else:" + self._nl,
            self._3tab + 'raise RuntimeError("incorrect ros message type")' + self._nl,
            self._2tab + "self._redis_wrapper.publish(topic, msgJson)" + self._nl,
            self._tab + "def receive(self):" + self._nl,
            self._2tab
            + "self._redis_wrapper.subscribe(self._sensors_topics)"
            + self._nl,
        ]

        f = open(self._filename, "a")
        f.writelines(payload)
        f.close()
