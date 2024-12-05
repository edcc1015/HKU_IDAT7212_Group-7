from adafruit_servokit import ServoKit
import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import smbus
import threading

# I2C总线号，通常为1
I2C_BUS = 1
# 四路电机驱动模块的I2C地址
MOTOR_ADDR = 0x34 

# 寄存器地址定义
ADC_BAT_ADDR = 0x00
MOTOR_TYPE_ADDR = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_PWM_ADDR = 0x1F
MOTOR_FIXED_SPEED_ADDR = 0x33
MOTOR_ENCODER_TOTAL_ADDR = 0x3C

# 电机类型具体值
MOTOR_TYPE_WITHOUT_ENCODER = 0
MOTOR_TYPE_TT = 1
MOTOR_TYPE_N20 = 2
MOTOR_TYPE_JGB37_520_12V_110RPM = 3  # 磁环每转是44个脉冲   减速比:90  默认

# 实例化SMBus
bus = smbus.SMBus(I2C_BUS)

# 电机类型及编码方向极性设置
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0
button = False
running = False
select = False
roadblock = False
running = False
nose = 0
analog = False


def joy_callback(data):
    global button, select, running, analog, roadblock, keep_turning_left
    axes = list(data.axes)
    buttons = list(data.buttons)
    print("Received Axes:", axes)
    print("Received Buttons:", buttons)

    if buttons == [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("RW")
        button = True

    elif buttons == [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("RS")
        button = True

    elif buttons ==  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("RA")
        button = True

    elif buttons == [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes ==  [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("RD")
        button = True

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes ==  [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, 1.0]:
        print("LW")
        button = True
        threading.Thread(target=forward).start()

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -1.0]:
        print("LS")
        button = True
        threading.Thread(target=back).start()

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, 1.0, -0.0]:
        print("LA")
        button = True
        threading.Thread(target=turn_left).start()

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -1.0, -0.0]:
        print("LD")
        button = True
        threading.Thread(target=turn_right).start()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and (axes == [-0.0, -0.0, 0.0, -0.0, -0.0, 1.0, -0.0, -0.0] or axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]):

        button = False
        if select == False and analog == False:
            motor_init()
            print("Button stop")
        return

    elif buttons == [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("L1")
        button = True


    elif buttons == [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("R1")
        button = True


    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes[2] != 0 and -1.0 <= axes[2] < 1.0 and axes[5] == 1:
        print("L2")
        button = True

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and axes[5] != 0 and -1.0 <= axes[5] < 1.0 and axes[2] == 1:
        print("R2")
        button = True

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("START")
        motor_init() 

    elif buttons == [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("SLECT")
        button = True
        select = True
        if running == True:
            select = False
            running = False
        else:
            threading.Thread(target=human_follow).start()
        print(select)

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0] and axes == [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]:
        print("ANALOG")
        button = True
        analog = True
        if roadblock == True:
            analog = False
            roadblock = False
        else:
            threading.Thread(target=ultrasound).start()
        print(analog)

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and ((-1.0 <= axes[1] <= 1.0) or (-1.0 <= axes[0] <= 1.0)) and (axes[1] != 0 or axes[0] !=0):
        print("LQ")
        if axes[1] > 0:
            print(f"Forward:{axes[1]}")
        elif axes[1] < 0:
            print(f"Back:{axes[1]}")
        elif axes[0] > 0:
            print(f"left:{axes[0]}")
        elif axes[0] < 0:
            print(f"right:{axes[0]}")
        button = True

    elif buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] and ((-1.0 <= axes[3] <= 1.0) or (-1.0 <= axes[4] <= 1.0)) and (axes[3] != 0 or axes[4] !=0):
        print("RQ")
        if axes[4] > 0:
            print(f"Forward:{axes[4]}")
        elif axes[4] < 0:
            print(f"Back:{axes[4]}")
        elif axes[3] > 0:
            print(f"left:{axes[3]}")
        elif axes[3] < 0:
            print(f"right:{axes[3]}")
        button = True
   
    axes == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    buttons == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def motor_init():  # 电机初始化
    bus.write_byte_data(MOTOR_ADDR, MOTOR_TYPE_ADDR, MotorType)  # 设置电机类型
    bus.write_byte_data(MOTOR_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)  # 设置编码极性

def control_motors(values, mode):
    global button
    """
    同时控制多个电机。
    
    :param values: 包含四个电机的控制值的列表，对于不控制的电机使用0作为占位符
    :param mode: 控制模式 ('pwm' 或 'speed')
    """
    if mode == 'pwm':
        bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_PWM_ADDR, values)
    elif mode == 'speed':
        bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, values)


def forward():
    control_motors([-20, -20, 0, 0], 'speed')
        
def turn_left():
    control_motors([-10, -20, 0, 0], 'speed')

def turn_right():
    control_motors([-20, -10, 0, 0], 'speed')

def back():
    control_motors([20, 20, 0, 0], 'speed')

def ultrasound():
    global roadblock
    roadblock = True
    while analog:
        if dis < 60:
            control_motors([10, 10, 0, 0], 'speed')
            time.sleep(0.2)
            motor_init()
            time.sleep(0.01)


def human_follow():
    global button, select, running, nose, offset
    running = True
    while select:
        time.sleep(0.2)
        # if nose < -100:
        #     control_motors([250*offset, -50*offset, 0, 0], 'speed')
        #     time.sleep(0.1)
        #     motor_init()
        #     time.sleep(1)
        #     print("左")
        # elif nose > 100:
        #     control_motors([-50*offset, 250*offset, 0, 0], 'speed')
        #     time.sleep(0.1)
        #     motor_init()
        #     time.sleep(1)
        #     print("右")
        # elif -100 < nose < 100:
        #     forward()
        #     time.sleep(0.1)
        #     print("前")
        # nose = 0

        if nose < -60:
            control_motors([-17, -20, 0, 0], 'speed')
            #time.sleep(abs(offset)/300)
            print("左")
            #motor_init()
            # if nose < -200:
            #     control_motors([-15, -20, 0, 0], 'speed')
            #     time.sleep(0.05)
            #     motor_init()
            #     time.sleep(0.05)

        elif nose > 60:
            control_motors([-20, -17, 0, 0], 'speed')
            #time.sleep(abs(offset)/300)
            print("右")
            #motor_init()
            # if nose > 200:
            #     control_motors([-20, -15, 0, 0], 'speed')
            #     time.sleep(0.05)
            #     motor_init()
            #     time.sleep(0.05)

        elif -60 < nose < 60:
            control_motors([-20, -20, 0, 0], 'speed')
            print("前")
            # time.sleep(0.5)
        
            
        


nose = 0
nose2 = 0  
offset = 0

def nose_position_callback(data):
    global nose, offset, nose2
    nose = data.x
    offset = int((nose2 - nose) * 0.6)
    print(f"offset = {offset}")
    nose2 = data.x 
    rospy.loginfo("Received nose position: x=%s", nose)



def distance_callback(data):
    global dis 
    dis = data.data
    rospy.loginfo(f"Received distance: {dis} cm")


def listener():
    rospy.init_node('joy_listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_callback)
    rospy.Subscriber('/nose_position', Point, nose_position_callback)
    rospy.Subscriber('distance', Float32, distance_callback)
    rospy.spin()
    
if __name__ == '__main__':
    print("start the control")
    motor_init()  # 初始化电机
    listener()

    # # 示例调用：同时控制电机1和电机2，其中电机1以50的固定速度，电机2以-50的固定速度运行
    # control_motors([50, -50, 0, 0], 'speed')

