#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('kf_button_server')
import rospy

from kingfisher_msgs.msg import Sense
from button_server.ButtonServer import ButtonServer
from button_server.requesthdl import *
from sensor_msgs.msg import NavSatFix
from historyqueue import *

class GPSkmlRH(requesthdl):
    def __init__(self, server):
        self.server = server
        requesthdl.__init__(self,"/gps\.kml")

    def run(self, request):
        kml = self.server.getGPSHistoryKML("GPS")
        request.send_response(200)
        request.send_header('Content-type','text/xml')
        request.end_headers()
        request.wfile.write(kml)
        request.wfile.close()


class KFButtonServer(ButtonServer):
    def __init__(self):
        self.voltage = 0.0
        self.rc = 0
        self.kf_root = roslib.packages.get_pkg_dir('kf_button_server')
        ButtonServer.__init__(self)
        self.gpsHistory=HistoryQueueHD(100,2000,20)
        self.senseSub = rospy.Subscriber("/sense",Sense,self.sense_cb)
        self.gpsSub = rospy.Subscriber("/gps/fix",
                         NavSatFix, self.gpsCallback)
        self.repository.addGetHandler(GPSkmlRH(self))

    def getGPSHistoryKML(self,label):
        f = open(self.kf_root+"/src/gps_template.kml","r")
        text = f.read();
        f.close()
        history = self.gpsHistory.getall()
        if len(history) > 0:
            startlog = "%f,%f,0" % (history[0][1],history[0][2])
            endlog = "%f,%f,0" % (history[-1][1],history[-1][2])
        else:
            startlog = "0,0,0"
            endlog = "0,0,0"
        return text % (startlog,endlog,label, \
                " ".join(["%f,%f,0" % (l[1],l[2]) for l in history]))


    def gpsCallback(self,msg):
        tmsg = msg.header.stamp.to_sec()
        if (self.gpsHistory.length()>0) and (tmsg - self.gpsHistory.front()[0]>120.):
            # Erase the history if we did not receive any point for 2 min
            self.gpsHistory.clear()
        self.gpsHistory.push([tmsg,msg.longitude,msg.latitude])
    
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

