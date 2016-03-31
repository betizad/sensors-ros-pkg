#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
import math;
import numpy;
from sensor_msgs.msg import Imu,MagneticField
from std_msgs.msg import Bool
import socket
from threading import Thread
import struct
      
class IMUUDPClient:
    def __init__(self):
        self.imu_out = rospy.Publisher("imu_raw", Imu, queue_size=10)
        self.mag_out = rospy.Publisher("imu_mag", MagneticField, queue_size=10)
        
        self.mag_calib = rospy.Subscriber("mag_calib", Bool, self.on_mag_calib)
        
        
        self.sock= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('',35050))
        self.sock.setblocking(0)
        self.sock.settimeout(5)
        self.runme = True 
                
        # Magnetic calibration data
        self.calib = False
        self.mag_bias = [0]*3
        self.mag_scale = [1]*3
        self.mag_min = [20000]*3
        self.mag_max = [-20000]*3
        
        
        self.runner = Thread(target=self.run)
        self.runner.start()
        
    def on_mag_calib(self, data):
        # When calibration is turned on
        if not self.calib and data.data:
            self.calib = data.data
            self.mag_bias = [0]*3
            self.mag_scale = [1]*3
            self.mag_min = [20000]*3
            self.mag_max = [-20000]*3
            
        # When calibration is turned off 
        if self.calib and not data.data:
            self.calib = data.data
            for i in range(0,3):
                self.mag_bias[i] = (self.mag_max[i] + self.mag_min[i])/2
                #self.mag_scale[i] = (self.mag_max[i] - self.mag_min[i])/2
            
            avg_rad = sum(self.mag_scale)/3.0
            
            #for i in range(0,3):
                #self.mag_scale[i] = avg_rad/self.mag_scale[i]
                
            rospy.loginfo("New bias: {0}, new scale: {1}".format(self.mag_bias, self.mag_scale))             
            rospy.loginfo("Max: {0}, min: {1}".format(self.mag_max, self.mag_min))
        
    def mag_calibration(self, mag_temp):
        for i in range(0,3):
            if mag_temp[i] > self.mag_max[i]:
                self.mag_max[i] = mag_temp[i]
                
            if mag_temp[i] < self.mag_min[i]:
                self.mag_min[i] = mag_temp[i]
                
        
    def run(self):
        while self.runme:
            try:
                data, addr = self.sock.recvfrom(36)
                raw_data = struct.unpack('fffffffff', data)
                #rospy.loginfo("Receveid: {0}".format(raw_data))
            
                imud = Imu()
                imud.linear_acceleration.x = raw_data[0]*9.8065
                imud.linear_acceleration.y = raw_data[1]*9.8065
                imud.linear_acceleration.z = raw_data[2]*9.8065
                imud.angular_velocity.x = raw_data[3]/180*math.pi
                imud.angular_velocity.y = raw_data[4]/180*math.pi
                imud.angular_velocity.z = raw_data[5]/180*math.pi
                imud.header.stamp = rospy.Time.now()
                imud.header.frame_id = "imu_frame"
            
                if self.calib:
                    self.mag_calibration([raw_data[6], raw_data[7], raw_data[8]])
                
                magd = MagneticField()
                magd.magnetic_field.x = (raw_data[6] - self.mag_bias[0])*self.mag_scale[0]
                magd.magnetic_field.y = (raw_data[7] - self.mag_bias[1])*self.mag_scale[1]
                magd.magnetic_field.z = (raw_data[8] - self.mag_bias[2])*self.mag_scale[2]
                magd.header.stamp = imud.header.stamp
                magd.header.frame_id = imud.header.frame_id
            
                self.imu_out.publish(imud)
                self.mag_out.publish(magd)
            except socket.timeout:
                pass
            
    def stop(self):
        self.runme = False
        self.runner.join()
        self.sock.close()
                    
if __name__ == "__main__":
    rospy.init_node("reach_pub");
    imu = IMUUDPClient();
    rospy.spin();
    imu.stop()
        
        
