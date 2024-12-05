#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time
import RPi.GPIO as GPIO
 
# GPIO PIN配置
trigger_pin = 7
echo_pin = 18

GPIO.setmode(GPIO.BOARD)
GPIO.setup(trigger_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

def send_trigger_pulse():
    GPIO.output(trigger_pin, True)
    time.sleep(0.0001)
    GPIO.output(trigger_pin, False)

def wait_for_echo(value, timeout):
    count = timeout
    while GPIO.input(echo_pin) != value and count > 0:
        count -= 1

def get_distance():
    send_trigger_pulse()
    wait_for_echo(True, 10000)
    start = time.time()
    wait_for_echo(False, 10000)
    finish = time.time()
    pulse_len = finish - start
    distance_cm = pulse_len / 0.000058
    return distance_cm

def distance_publisher():
    # 初始化ROS节点
    rospy.init_node('distance_sensor')
    # 创建一个Publisher，发布Float32类型的消息到'distance'话题
    pub = rospy.Publisher('distance', Float32, queue_size=10)
    # 设置循环的频率
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        distance = get_distance()
        rospy.loginfo(f"Measured Distance: {distance} cm")
        pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        distance_publisher()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
