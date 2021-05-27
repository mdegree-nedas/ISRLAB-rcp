class Controller:
    def decision(self, sensor_data):
        if sensor_data == [1,0,0]:
            return 'left'
        elif sensor_data == [0,1,0]:
            return 'forward'
        elif sensor_data == [0,0,1]:
            return 'right'
        elif sensor_data == [0,0,0]:
            return 'idle'
        else:
            return None
