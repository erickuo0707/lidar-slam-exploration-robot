#!/usr/bin/env python3
import gpiod, time, threading, rospy
from geometry_msgs.msg import Twist

chip = gpiod.Chip('gpiochip7')
line_left  = chip.get_line(22)
line_right = chip.get_line(23)
line_left.request(consumer='servo_l',  type=gpiod.LINE_REQ_DIR_OUT)
line_right.request(consumer='servo_r', type=gpiod.LINE_REQ_DIR_OUT)

target_left  = 1500
target_right = 1400   # 右輪中立點是 1400
current_left  = 1500.0
current_right = 1400.0
last_msg_time = time.time()
lock = threading.Lock()

LEFT_NEUTRAL  = 1500
RIGHT_NEUTRAL = 1385  # 右輪中立點

SMOOTH = 0.15

def cmd_callback(msg):
    global target_left, target_right, last_msg_time
    with lock:
        last_msg_time = time.time()
        forward_offset = int(msg.linear.x * 1800)
        turn_offset    = int(msg.angular.z * 500)
        target_left  = max(1250, min(1750, LEFT_NEUTRAL  + forward_offset - turn_offset))
        target_right = max(1150, min(1650, RIGHT_NEUTRAL - forward_offset - turn_offset))

def send_pulse(line, width_us):
    line.set_value(1)
    time.sleep(width_us / 1_000_000.0)
    line.set_value(0)

def pulse_loop():
    global current_left, current_right
    while not rospy.is_shutdown():
        with lock:
            timeout = time.time() - last_msg_time > 1.0
            tl = target_left
            tr = target_right
        if timeout:
            current_left  = float(LEFT_NEUTRAL)
            current_right = float(RIGHT_NEUTRAL)
            time.sleep(0.02)
            continue
        current_left  += SMOOTH * (tl - current_left)
        current_right += SMOOTH * (tr - current_right)
        send_pulse(line_left,  int(current_left))
        send_pulse(line_right, int(current_right))
        time.sleep(0.005)

if __name__ == '__main__':
    rospy.init_node('motor_driver')
    rospy.Subscriber('/cmd_vel', Twist, cmd_callback)
    rospy.loginfo("✅ motor_driver 已上線 (右輪中立點=1400)")
    try:
        pulse_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        line_left.set_value(0);  line_right.set_value(0)
        line_left.release();     line_right.release()
        rospy.loginfo("🛑 motor_driver 已安全關閉")
