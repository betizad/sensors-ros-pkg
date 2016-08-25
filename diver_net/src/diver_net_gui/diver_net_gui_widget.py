import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import *

from std_msgs.msg import Bool
from std_msgs.msg import Int16

class DiverNetGUI(Plugin):

    def __init__(self, context):
        super(DiverNetGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DiverNetGUI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('diver_net'), 'src/diver_net_gui/resource', 'DiverNetGUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DiverNetGUIUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.pubGyroMean = rospy.Publisher('calculate_gyro_mean', Int16, queue_size=1)
        self.pubPoseCal = rospy.Publisher('calibrate_pose', Int16, queue_size=1)
        self.pubMagCal = rospy.Publisher('calibrate_magnetometer', Int16, queue_size=1)

        self._widget.gyroMeanButton.clicked.connect(self.on_gyro_mean)
        self._widget.poseCalButton.clicked.connect(self.on_pose_cal)
        self._widget.magCalButton.clicked.connect(self.on_mag_cal)
        self._widget.enableGyroMean.clicked.connect(self.on_enable_gyro_mean)
        self._widget.enablePoseCal.clicked.connect(self.on_enable_pose_cal)
        self._widget.enableMagCal.clicked.connect(self.on_enable_mag_cal)

    def on_gyro_mean(self):
        self.pubGyroMean.publish(1)
        self._widget.enableGyroMean.setCheckState(Qt.Checked)

    def on_enable_gyro_mean(self):
        if self._widget.enableGyroMean.isChecked():
          self.pubGyroMean.publish(2)
        else:
          self.pubGyroMean.publish(0)

    def on_pose_cal(self):
        self.pubPoseCal.publish(1)
        self._widget.enablePoseCal.setCheckState(Qt.Checked)

    def on_enable_pose_cal(self):
        if self._widget.enablePoseCal.isChecked():
          self.pubPoseCal.publish(2)
        else:
          self.pubPoseCal.publish(0)

    def on_mag_cal(self):
        self.pubMagCal.publish(1)
        self._widget.enableMagCal.setCheckState(Qt.Checked)

    def on_enable_mag_cal(self):
        if self._widget.enableMagCal.isChecked():
          self.pubMagCal.publish(2)
        else:
          self.pubMagCal.publish(0)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
