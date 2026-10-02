
'''
Written by yufeng.wu0902@gmail.com

Python class that passes arguments to connected ESP32C3 controller, which
then sends commands wirelessly to the robot via ESPNow.
'''

import serial

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]
# The head turns at most this far either side of centre. The robot firmware
# clamps to the same limit.
HEAD_LIMIT_DEG = 90.0
# The controller relays at most 99 characters of each command to the robot.
MAX_COMMAND_LENGTH = 99

class q8_espnow:
    def __init__(self, port, joint_list = DEFAULT_JOINTLIST, baud = 115200):
        self.DEVICENAME = port
        self.BAUDRATE = baud
        self.JOINTS = joint_list
        self.prev_pos = [90 for i in range(8)]
        self.prev_profile = 0
        self.torque_on = False
        # Head angle in deg, positive to the left. It is sent as the 12th field
        # of every joint command, so it does not depend on the fitted legs.
        self.head_deg = 0.0
        self._last_move = None

        # Initialize serial communication with ESP32-C3
        self.serialHandler = serial.Serial(self.DEVICENAME, self.BAUDRATE)

    def enable_torque(self):
        # The robot parks the head before torque-on, then turns it to head_deg.
        self.serialHandler.write(f"0,0,0,0,0,0,0,0,0,0,1,{self.head_deg:.1f};".encode())
        self.torque_on = True
        return True
    
    def disable_torque(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,0,0,0;".encode())
        self.torque_on = False
        return True
    
    def check_battery(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,1,0,0;".encode())
        return True
    
    def record_data(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,2,0,0;".encode())
        return True
    
    def finish_recording(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,3,0,1;".encode())
        return True
    
    def send_jump(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,4,0,0;".encode())
        return True

    def send_recover(self):
        # Clears latched servo faults and re-enables torque on the robot. The
        # local flag must follow, or the next move_all would carry torque=0 and
        # the firmware would read that as a request to switch torque back off.
        self.serialHandler.write("0,0,0,0,0,0,0,0,5,0,1;".encode())
        self.torque_on = True
        return True

    def move_all(self, joints_pos, dur = 0, record = True):
        # Expects 8 positions in deg. For example: [0, 90, 0, 90, 0, 90, 0, 90]
        try:
            # If record is true, the 9th element is set to value 2. Else 0.
            cmd = ",".join(map(str, joints_pos)) + f",{record*2}," + f"{dur}," + \
                  f"{int(self.torque_on)},{self.head_deg:.1f}"
            # print(cmd)
            if len(cmd) > MAX_COMMAND_LENGTH:
                # The controller would cut it and send the rest as a command.
                return False
            self.serialHandler.write((cmd + ";").encode())
        except:
            return False
        self._last_move = (list(joints_pos), dur)
        return True

    def move_head(self, deg):
        """Turn the head to deg (positive left), clamped to +/-HEAD_LIMIT_DEG."""
        previous = self.head_deg
        self.head_deg = max(-HEAD_LIMIT_DEG, min(HEAD_LIMIT_DEG, float(deg)))
        if self._last_move is None:
            # The angle goes out with the first joint command.
            return True
        # Repeating the last joint targets and timing leaves the legs where
        # they are, also on robot firmware that predates the head field.
        joints, dur = self._last_move
        if self.move_all(joints, dur, False):
            return True
        self.head_deg = previous
        return False
    
    def move_mirror(self, joint_pos, dur = 0):
        # Expects a pair of pos for one leg, which will be mirrored 4times.
        mirrored_pos = []
        for i in range(4):
            mirrored_pos.append(joint_pos[0])
            mirrored_pos.append(joint_pos[1])
        return self.move_all(mirrored_pos, dur, False)
    
    def bulkread(self, addr, len = 4):
        value = [0 for i in range(8)]
        return value, True
    
    def joint_read4(self, joint, addr):
        value = 10
        return value
    
    def joint_read2(self, joint, addr):
        value = 10
        return value

    def check_voltage(self):
        voltage = 3.7
        return voltage

    def dxl2deg(self, angle_dxl):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_friendly = (angle_dxl - self.ZERO_OFFSET) * friendly_per_dxl
        return angle_friendly

    def deg2dxl(self, angle_friendly):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_dxl = int(angle_friendly / friendly_per_dxl + 0.5) + \
                    self.ZERO_OFFSET
        return angle_dxl
    
    #-------------------#
    # Private Functions #
    #-------------------#
    
    def _set_profile(self, dur_ms):
        return