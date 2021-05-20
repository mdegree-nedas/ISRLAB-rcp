# rcp: INTERFACE MODULE
# rcp: auto-generated ros foreign interface file
# rcp: EDIT this file and insert other ros2 msg interface wrappers


class GeometryMsgsTwist:
    def __init__(self, topic, command, msg_type, msg):
        self.topic = topic
        self.command = command
        self.msg_type = msg_type
        self.data = self.GeometryMsgsTwistData(msg)

    class GeometryMsgsTwistData:
        def __init__(self, msg):
            self.linear = self.GeometryMsgsTwistDataLinear(msg)
            self.angular = self.GeometryMsgsTwistDataAngular(msg)

        class GeometryMsgsTwistDataLinear:
            def __init__(self, msg):
                self.x = msg.linear.x
                self.y = msg.linear.y
                self.z = msg.linear.z

        class GeometryMsgsTwistDataAngular:
            def __init__(self, msg):
                self.x = msg.angular.x
                self.y = msg.angular.y
                self.z = msg.angular.z
