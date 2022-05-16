#!/usr/bin/env/python

from faulthandler import disable
from urllib3 import disable_warnings
import rospy
from std_msgs.msg import String
import termios, tty, sys, select

old_attr = termios.tcgetattr(sys.stdin)

def main():
    key_pub = rospy.Publisher('keys', String, queue_size=10)
    rospy.init_node('key_publisher_rem', anonymous=True, disable_signals=True)
    rate = rospy.Rate(100)

    # setting the the stdin to raw
    tty.setcbreak(sys.stdin.fileno())
    print('publishing key strokes, press Ctrl-C to exit...')

    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()
    


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('old attribute is now set.')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
        