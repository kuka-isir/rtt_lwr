import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget,QTableWidgetItem

from std_msgs.msg import ByteMultiArray,Float32MultiArray,Int32MultiArray
from std_srvs.srv import Empty,Trigger

from threading import Thread

class ServiceCallerEmpty():
    def __init__(self,service_name):
        service = rospy.ServiceProxy(service_name, Empty)
        service_name = service_name
        try:
            rospy.wait_for_service(service_name,1.0)
            try:
                service.call()
            except: pass
        except:
            print("Not Connected, please check the topics")

class ServiceCallerTrigger(Thread):
    def __init__(self,service_name,element_to_update,rate=10.0):
        Thread.__init__(self)
        self.r = rospy.Rate(rate)
        self.service = rospy.ServiceProxy(service_name, Trigger)
        self.service_name = service_name
        self.element_to_update = element_to_update
        self.stop = False
        self.start()

    def run(self):
        while not self.stop:
            try:
                rospy.wait_for_service(self.service_name,1.0)
                while not self.stop:
                    res = self.service.call()
                    self.element_to_update.setText(res.message)
                    self.r.sleep()
            except: pass
            
class ControlModeCaller():
    def setJointImpedanceControlMode(self,e):
        print("setJointImpedanceControlMode")
        ServiceCallerEmpty("set_joint_impedance_control_mode")
    def setJointTorqueControlMode(self,e):
        print("setJointTorqueControlMode")
        ServiceCallerEmpty("set_joint_torque_control_mode")
    def setCartesianImpedanceControlMode(self,e):
        print("setCartesianImpedanceControlMode")
        ServiceCallerEmpty("set_cartesian_impedance_control_mode")
    def setJointPositionControlMode(self,e):
        print("setJointPositionControlMode")
        ServiceCallerEmpty("set_joint_position_control_mode")
        

class LWRPlugin(Plugin):

    def __init__(self, context):
        super(LWRPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LWRPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_lwr'), 'resource', 'LWRPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('LWRPlugin')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        # Initialize the tabs
        self.fri_bool = [True]*16
        self.fri_int = [0]*16
        self.fri_float = [0.0]*16
        
        self.table = self._widget.tableWidget
        
        for i in range(16):
            self._widget.tableWidget.setItem(0,i,QTableWidgetItem(str(self.fri_bool[i])))
            self._widget.tableWidget.setItem(1,i,QTableWidgetItem(str(self.fri_int[i])))
            self._widget.tableWidget.setItem(2,i,QTableWidgetItem(str(self.fri_float[i])))
            
        self.b_sub = rospy.Subscriber("boolDataFromKRL",ByteMultiArray,callback=self.bool_cb,queue_size=10)
        self.i_sub = rospy.Subscriber("intDataFromKRL",Int32MultiArray,callback=self.int_cb,queue_size=10)
        self.f_sub = rospy.Subscriber("realDataFromKRL",Float32MultiArray,callback=self.float_cb,queue_size=10)
        
        
        self.cm = ControlModeCaller()
        
        self._widget.joint_impedance.clicked.connect(self.cm.setJointImpedanceControlMode)
        self._widget.joint_torque.clicked.connect(self.cm.setJointTorqueControlMode)
        self._widget.cartesian_impedance.clicked.connect(self.cm.setCartesianImpedanceControlMode)  
        self._widget.joint_position.clicked.connect(self.cm.setJointPositionControlMode)
        
        self.get_ctrl_th = ServiceCallerTrigger("get_current_control_mode",self._widget.current_control_mode)
        
    def bool_cb(self,msg):
        for i in range(16):
            self.table.item(0,i).setText(str(msg.data[i]))
            
    def int_cb(self,msg):
        for i in range(16):
            self.table.item(1,i).setText(str(msg.data[i]))
            
    def float_cb(self,msg):
        for i in range(16):
            self.table.item(2,i).setText(str(msg.data[i]))

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.get_ctrl_th.stop = True
        
        self.b_sub.unregister()
        self.i_sub.unregister()
        self.f_sub.unregister()
        
        
        
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
