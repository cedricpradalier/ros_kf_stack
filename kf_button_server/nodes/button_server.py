#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('kf_button_server')
import rospy

from kingfisher_msgs.msg import Sense
from button_server.ButtonServer import ButtonServer

class KFButtonServer(ButtonServer):
    def __init__(self):
        self.voltage = 0.0
        self.rc = 0
        ButtonServer.__init__(self)
        self.sub = rospy.Subscriber("/sense",Sense,self.sense_cb)
    
    def sense_cb(self,data):
        self.voltage = data.battery
        self.rc = data.rc

    def getHeader(self):
        # To be overloaded
        return """
        <h2> Status </h2>
        <center><p> <ul> 
            <li> Battery Voltage: %.2f V </li>
            <li> RC State: %04X </li>
        </ul></p></center><hr/>
        """ % (self.voltage,self.rc)


if __name__ == '__main__':
    try:
        server = KFButtonServer()
        server.run()
    except rospy.ROSInterruptException: 
        pass

