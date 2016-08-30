#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from nmea_msgs.msg import Sentence
from std_msgs.msg import String
      
class NMEAClient:
    def __init__(self):
        self.nmea_sentence = rospy.Publisher("nmea_sentence", Sentence, queue_size=10)
        self.nmea_string = rospy.Subscriber("gps_sentences", String, self.on_sentences, queue_size=20)
        
    def on_sentences(self, data):
        if "GGA" in data.data:
            out = Sentence()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = "gps_frame"
            out.sentence = data.data
            self.nmea_sentence.publish(out)
                                
if __name__ == "__main__":
    rospy.init_node("reach_nmea");
    client = NMEAClient()
    rospy.spin();
        
        
