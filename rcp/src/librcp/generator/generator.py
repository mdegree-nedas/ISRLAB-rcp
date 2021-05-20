from librcp.generator.noros.generator_core import NoRosCoreGenerator
from librcp.generator.noros.generator_template import NoRosTemplateGenerator
from librcp.generator.noros.generator_broker import NoRosBrokerGenerator

from librcp.generator.ros.generator_core import RosCoreGenerator
from librcp.generator.ros.generator_interface import RosInterfaceGenerator
from librcp.generator.ros.generator_broker import RosBrokerGenerator


class Generator:
    def __init__(self):
        self.noros_core = NoRosCoreGenerator()
        self.noros_template = NoRosTemplateGenerator()
        self.noros_broker = NoRosBrokerGenerator()

        self.ros_core = RosCoreGenerator()
        self.ros_interface = RosInterfaceGenerator()
        self.ros_broker = RosBrokerGenerator()
