#!/usr/bin/env python
# source for this file:
import roslib; roslib.load_manifest('joystick_keyboard_emulator')
import rospy
import threading
from sensor_msgs.msg import Joy
import sys, select, termios, tty

msg = """
Reading from the keyboard and emulating a 2 axis joystick with 10 buttons
---------------------------
Left thumbpad              Right thumpad
   q    w    e             u    i    o
   a    s    d             j    k    l
   z    x    c             m    ,    .

Buttons: nummbers

CTRL-C to quit
"""

# Move bindings associate a set of operation to execute on a set of axis
scale = 0.1
moveBindings = {
        'q': [(1,1),(0,1)],
        'w': [(1,1)],
        'e':[(1,1),(0,-1)],
        'a':[(0,1)],
        's':[(0,0),(1,0)],
        'd':[(0,-1)],
        'z':[(1,-1),(0,1)],
        'x':[(1,-1)],
        'c':[(1,-1),(0,-1)],
        'u': [(4,1),(3,1)],
        'i': [(4,1)],
        'o':[(4,1),(3,-1)],
        'j':[(3,1)],
        'k':[(4,0),(3,0)],
        'l':[(3,-1)],
        'm':[(4,-1),(3,1)],
        ',':[(4,-1)],
        '.':[(4,-1),(3,-1)],
           }
quit = False
mutex = threading.Lock()
joy = Joy()


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publisher():
    global joy, mutex, quit
    pub = rospy.Publisher('/joy', Joy)
    rate = rospy.Rate(10)
    while not quit and not rospy.is_shutdown():
        rate.sleep()
        with mutex:
            joy.header.stamp = rospy.Time.now()
            pub.publish(joy)
            


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('joystick_keyboard_emulator')
    print msg

    try:
        joy.axes = [0.0] * 10
        joy.buttons = [0] * 10
        thread = threading.Thread(target = publisher)
        thread.start()
        while not rospy.is_shutdown():
            key = getKey()
            with mutex:
                if key in moveBindings.keys():
                    for (axis,action) in moveBindings[key]:
                        if action == 0:
                            joy.axes[axis] = 0.0
                        else:
                            joy.axes[axis] += action * scale
                            joy.axes[axis] = min(max(-1,joy.axes[axis]),1)
                elif key in [str(x) for x in range(0,10)]:
                    joy.buttons[int(key)] = 1 - joy.buttons[int(key)]
                else:
                    print "%s,%X" %(key,ord(key))
                    joy.axes = [0.0] * 4
                    joy.buttons = [0] * 10
                    if (key == '\x03') or (key=='\x1B'):
                        quit = True
                        break
    except : 
        import traceback
        rospy.logerr(traceback.format_exc())

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
