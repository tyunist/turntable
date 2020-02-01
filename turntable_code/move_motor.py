
"""
    This is a driver program to run the EX-06+ dynamixel motor 

    Angle Convention :

    + positive angle - CW direction 
    - negative angle - CCW direction 
"""




import os
import time 
from signal import signal, SIGINT




from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_OPERATION_MODE     = 11
ADDR_MX_CW_LIMIT_L         = 6
ADDR_MX_CW_LIMIT_H         = 7
ADDR_MX_CCW_LIMIT_L        = 8 
ADDR_MX_CCW_LIMIT_H        = 9
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_MOVING_SPEED_H     = 33
ADDR_MX_TORQUE_LIMIT       = 34 
ADDR_MX_PRESENT_SPEED         = 38

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = "/dev/ttyUSB0"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
DXL_MOVING_SPEED            = 80                # Dynamixel moving speed range 0-1023
DXL_MOVING_DIRECTION        = 1                 # 1 for CW and 0 for CCW Direction


index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
current_speed = 0
current_direction = 0







# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
# if portHandler.setBaudRate(BAUDRATE):
#     print("Succeeded to change the baudrate")
# else:
#     print("Failed to change the baudrate")
#     print("Press any key to terminate...")
#     getch()
#     quit()


def execute_2byte_command(command):
    # Enable wheel mode
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, command[0], command[1], command[2])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Executed command successfully!")
    return

def enable_wheel_mode():
    command = [1, ADDR_MX_CCW_LIMIT_L, 0]
    execute_2byte_command(command)
    command[1] = ADDR_MX_CW_LIMIT_L
    command[2] = 0 
    execute_2byte_command(command)
    print('Set operation mode to wheel mode')
    return


def initialize_motor():
    global current_speed, current_direction

    enable_wheel_mode()
    
    speed = (0x50&0x3FF) | (DXL_MOVING_DIRECTION << 10)
    speed_command = [ 1, ADDR_MX_MOVING_SPEED, speed]
    execute_2byte_command(speed_command)
    current_speed = speed
    current_direction = DXL_MOVING_DIRECTION
    stop_motor()




def start_motor():
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque enable has been set")

def stop_motor():
    # # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def set_motor_dir_CW():
    stop_motor()
    speed = (DXL_MOVING_SPEED&0x3FF) | (1 << 10)
    speed_command = [ 1, ADDR_MX_MOVING_SPEED, speed]
    execute_2byte_command(speed_command)
    stop_motor()

def set_motor_dir_CCW():
    stop_motor()
    speed = (DXL_MOVING_SPEED&0x3FF) | (0 << 10)
    speed_command = [ 1, ADDR_MX_MOVING_SPEED, speed]
    execute_2byte_command(speed_command) 
    stop_motor()

#Reads two bytes of data from the address given 
def read_2byte_command(address):
    dxl_data, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, address)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return -1 
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        return -1
    print('Data', dxl_data)
    return dxl_data


def on_exit():
    stop_motor()
    # Close port
    portHandler.closePort()


def get_current_position():
    # According to data sheet the unit of measurement for this value is 0.06 degrees
    # But this is still position of the rotary encoder. 
    #TODO: Figure out how to convert rotary encoder position to angle on the surface

    return read_2byte_command(ADDR_MX_PRESENT_POSITION)*0.06 

def get_rpm():
    #The approximation as per the datasheet is the 
    #Current rpm (last 10 bits of data / 9) 

    return read_2byte_command(ADDR_MX_MOVING_SPEED)/9.0

def get_current_direction():
    # The 10 th bit of the motor speed gives the direction of the motor 
    #If 10 th bit is set to 1 - CW direction, else CCW drection

    #Returns: 1 for CW direction and 0 for CCW direction 

    global current_speed

    current_speed = read_2byte_command(ADDR_MX_MOVING_SPEED)
    current_direction = [1 if (current_speed & 0x400)>0 else 0]
    print(current_direction)

    return current_direction

def calculate_time(rpm, angle):
    #Given rpm and angle find the time to activate the motor
    time = (angle / (rpm*360))*60
    return time


def move_motor_angle(angle):
    #This is to move the motor a certain angle 
    global current_speed
    current_direction = get_current_direction()
    print(current_direction)

    if current_direction == -1 :
        print('Reading direction failed. Exiting!')
        exit()


    rpm = get_rpm()
    print(rpm)

    if current_direction and angle < 0: 
        current_speed = set_motor_dir_CCW()
    

    if not current_direction and angle > 0:
        current_speed = set_motor_dir_CW()

    motor_time = calculate_time(rpm,abs(angle))
    start_motor()
    time.sleep(motor_time)
    stop_motor()


def handler(signal_received, frame):
# Handle any cleanup hndlerre
    stop_motor()
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)



if __name__ == "__main__":
    initialize_motor()

    signal(SIGINT,handler)
    move_motor_angle(10000)
    move_motor_angle(-3000)
    # stop_motor()