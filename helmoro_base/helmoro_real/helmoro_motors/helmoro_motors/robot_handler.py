from helmoro_motors.roboclaw import Roboclaw

class RobotHandler:
    # TODO: When calibrating the PID with motion studio, always check if max_speed changed
    def __init__(self, addr_left=0x80, addr_right=0x80, baud=115200, encoder_res=2800, max_speed=125000):
        
        #Parameters
        self._addr_left = addr_left
        self._addr_right = addr_right
        self._baud = baud
        self._encoder_res = encoder_res
        self._max_speed = max_speed # Max speed in qpps
        self._wheel_circumference = 1.0 # Default value

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
        

    def get_wheel_positions(self):
        wheel_pos = [0.0, 0.0, 0.0, 0.0]
        wheel_pos[0] = self._right_motors.ReadEncM1(self._addr_right)[1] / self._encoder_res*self._wheel_circumference
        wheel_pos[1] = self._left_motors.ReadEncM1(self._addr_left)[1] / self._encoder_res*self._wheel_circumference
        wheel_pos[2] = self._right_motors.ReadEncM2(self._addr_right)[1] / self._encoder_res*self._wheel_circumference
        wheel_pos[3] = self._left_motors.ReadEncM2(self._addr_left)[1] / self._encoder_res*self._wheel_circumference
        return wheel_pos

    def get_wheel_velocities(self):
        wheel_vel = [0.0, 0.0, 0.0, 0.0]
        wheel_vel[0] = self._right_motors.ReadSpeedM1(self._addr_right)[1] / self._encoder_res*self._wheel_circumference
        wheel_vel[1] = self._left_motors.ReadSpeedM1(self._addr_left)[1] / self._encoder_res*self._wheel_circumference
        wheel_vel[2] = self._right_motors.ReadSpeedM2(self._addr_right)[1] / self._encoder_res*self._wheel_circumference
        wheel_vel[3] = self._left_motors.ReadSpeedM2(self._addr_left)[1] / self._encoder_res*self._wheel_circumference
        return wheel_vel
        
    def send_command(self, vel_mps):
        # Transform wheel velocities in m/s to qpps
        vel_qpps = [0, 0, 0, 0]
        for i in range(len(vel_mps)):
            vel_qpps[i] = int(vel_mps[i] / self._wheel_circumference * self._encoder_res)

        self._right_motors.SpeedM1M2(self._addr_right, vel_qpps[0], vel_qpps[2])
        self._left_motors.SpeedM1M2(self._addr_left, vel_qpps[1], vel_qpps[3])
        return vel_qpps

    def set_wheel_circumference(self, wheel_circumference):
        self._wheel_circumference = wheel_circumference