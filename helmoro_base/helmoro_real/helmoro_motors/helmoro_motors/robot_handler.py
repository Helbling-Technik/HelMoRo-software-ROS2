from helmoro_motors.roboclaw import Roboclaw

class RobotHandler:
    # TODO: Check what correct encoder_res is, current encoder_res was approximated by hand calculations
    # Documentation says, encoder_res should be 700???
    def __init__(self, addr_left=0x81, addr_right=0x80, baud=115200, encoder_res=3500):
        
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

        version_right = self._left_motors.ReadVersion(self._addr_right)
        if not version_right[0]:
            print("GETVERSION Failed")
        else:
            print(repr(version_right[1]))

        version_left = self._left_motors.ReadVersion(self._addr_left)
        if not version_left[0]:
            print("GETVERSION Failed")
        else:
            print(repr(version_right[1]))
        

    def get_wheel_positions(self, wheel_circumference):
        wheel_pos = [0.0, 0.0, 0.0, 0.0]
        wheel_pos[0] = self._right_motors.ReadEncM1(self._addr_right)[1]/self._encoder_res*wheel_circumference
        wheel_pos[1] = self._left_motors.ReadEncM1(self._addr_left)[1]/self._encoder_res*wheel_circumference
        wheel_pos[2] = self._right_motors.ReadEncM2(self._addr_right)[1]/self._encoder_res*wheel_circumference
        wheel_pos[3] = self._left_motors.ReadEncM2(self._addr_left)[1]/self._encoder_res*wheel_circumference
        return wheel_pos

    def get_wheel_velocities(self, wheel_circumference):
        wheel_vel = [0.0, 0.0, 0.0, 0.0]
        wheel_vel[0] = self._right_motors.ReadSpeedM1(self._addr_right)[1]/self._encoder_res*wheel_circumference
        wheel_vel[1] = self._left_motors.ReadSpeedM1(self._addr_left)[1]/self._encoder_res*wheel_circumference
        wheel_vel[2] = self._right_motors.ReadSpeedM2(self._addr_right)[1]/self._encoder_res*wheel_circumference
        wheel_vel[3] = self._left_motors.ReadSpeedM2(self._addr_left)[1]/self._encoder_res*wheel_circumference
        return wheel_vel
        
    def send_command(self, wheel_vel_cmd):
        # wheel_vel_cmd = [int(x * self._encoder_res) for x in wheel_vel_cmd] # Multiply wheel speed commands with encoder resolution
        
        self._right_motors.SpeedM1M2(self._addr_right, wheel_vel_cmd[0], wheel_vel_cmd[2])
        self._left_motors.SpeedM1M2(self._addr_left, wheel_vel_cmd[1], wheel_vel_cmd[3])
        return True
