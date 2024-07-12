from helmoro_motors.robotclaw import Roboclaw

class RobotHandler:

    def __init__(self, addr_left=128, addr_right=129, baud=115200, encoder_res=2797):
        
        #Parameters
        self._addr_left = addr_left
        self._addr_right = addr_right
        self._baud = baud
        self._encoder_res = encoder_res

        # Init Variables
        self._wheel_vel_cmds = [0.0, 0.0, 0.0, 0.0]
        self._wheel_pos = [0.0, 0.0, 0.0, 0.0]

        # Initialize Robotclaw Motor Controllers
        self._left_motors = Roboclaw("/dev/ttyACM1", self._baud)
        self._right_motors = Roboclaw("/dev/ttyACM0", self._baud)

        # Start Communication with Robotclaw motor controllers
        self._left_motors.Open()
        self._right_motors.Open()

    def get_wheel_positions(self):
        wheel_pos[0] = self._right_motors.ReadEncM1(self._addr_right)[2]/self._encoder_res
        wheel_pos[1] = self._left_motors.ReadEncM1(self._addr_left)[2]/self._encoder_res
        wheel_pos[2] = self._right_motors.ReadEncM2(self._addr_right)[2]/self._encoder_res
        wheel_pos[3] = self._left_motors.ReadEncM2(self._addr_left)[2]/self._encoder_res
        return wheel_pos

    def get_wheel_velocities(self):
        wheel_vel[0] = self._right_motors.ReadSpeedM1(self._addr_right)[2]/self._encoder_res
        wheel_vel[1] = self._left_motors.ReadSpeedM1(self._addr_left)[2]/self._encoder_res
        wheel_vel[2] = self._right_motors.ReadSpeedM1(self._addr_right)[2]/self._encoder_res
        wheel_vel[3] = self._left_motors.ReadSpeedM1(self._addr_left)[2]/self._encoder_res
        return wheel_vel
        
    def send_command(self, wheel_vel_cmd):
        if not self._right_motors.SpeedM1M2(self._addr_right, wheel_vel_cmd[0], wheel_vel_cmd[2]):
            return False
        if not self._left_motors.SpeedM1M2(self._addr_left, wheel_vel_cmd[1], wheel_vel_cmd[3]):
            return False
        return True
