#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import rospy
from corgi_msgs.msg import *

class CorgiControlPanel(QWidget):
    def __init__(self):
        super(CorgiControlPanel, self).__init__()
        self.init_ui()
        self.init_ros()

    def init_ui(self):
        ### Power Button ###
        self.label_power = QLabel('Power:', self)
        self.label_power.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_power = QButtonGroup(self)
        self.btn_group_power.setExclusive(True)
        
        self.btn_power_on = QPushButton('ON', self)
        self.btn_power_off = QPushButton('OFF', self)
        
        btn_power_list = [self.btn_power_on, self.btn_power_off]
        btn_power_style = '''QPushButton {background-color: white; color: black; text-align: center;}
                             QPushButton:checked {background-color: pink; color: black;}
                             QPushButton:hover:!checked {background-color: silver; color: black;}'''

        for btn in btn_power_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_power_style)
            self.btn_group_power.addButton(btn)
            
        self.btn_power_off.setChecked(True)

        ### Mode Switch Button ###
        self.label_mode = QLabel('Robot Mode Switch:')
        self.label_mode.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_mode = QButtonGroup(self)
        self.btn_group_mode.setExclusive(True)
        
        self.btn_rest_mode  = QPushButton('Rest Mode', self)
        self.btn_set_zero   = QPushButton('Set Zero', self)
        self.btn_hall_cal   = QPushButton('Hall Calibrate', self)
        self.btn_motor_mode = QPushButton('Motor Mode', self)
        
        btn_mode_list = [self.btn_rest_mode, self.btn_set_zero, self.btn_hall_cal, self.btn_motor_mode]
        btn_mode_style = '''QPushButton {background-color: white; color: black; text-align: center;}
                            QPushButton:checked {background-color: palegreen; color: black;}
                            QPushButton:hover:!checked {background-color: silver; color: black;}'''
        
        for btn in btn_mode_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_mode_style)
            self.btn_group_mode.addButton(btn)
        
        self.btn_rest_mode.setChecked(True)
        
            
        ### Data Output File Name ###
        self.label_output = QLabel('Output File Name (.csv):', self)
        self.label_output.setStyleSheet('color: white; font-weight: bold;')
        
        self.edit_output = QLineEdit('test', self)
        self.edit_output.setStyleSheet('''background-color: white;''')
        
            
        ### Trigger Button ###
        self.btn_trigger = QPushButton('Trigger', self)
        self.btn_trigger.setCheckable(True)
        self.btn_trigger.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center;}
                                          QPushButton:checked {background-color: skyblue; color: black;}
                                          QPushButton:hover:!checked {background-color: silver; color: black;}''')
        
        
        
        
        ### Vertical Line ###
        vline = QWidget()
        vline.setFixedWidth(5)
        vline.setStyleSheet('background-color: black;')
        
        
        ### Arrange Elements ###
        layout = QGridLayout()
        layout.setSpacing(20)
        layout.addWidget(self.label_power,    0, 0, 1, 2)
        layout.addWidget(self.btn_power_on,   1, 0, 1, 1)
        layout.addWidget(self.btn_power_off,  1, 1, 1, 1)
        layout.addWidget(self.label_mode,     2, 0, 1, 2)
        layout.addWidget(self.btn_rest_mode,  3, 0, 1, 2)
        layout.addWidget(self.btn_set_zero,   4, 0, 1, 2)
        layout.addWidget(self.btn_hall_cal,   5, 0, 1, 2)
        layout.addWidget(self.btn_motor_mode, 6, 0, 1, 2)
        layout.addWidget(vline,               0, 2, 7, 1)
        layout.addWidget(self.label_output,   0, 3, 1, 2)
        layout.addWidget(self.edit_output,    1, 3, 1, 2)
        layout.addWidget(self.btn_trigger,    6, 3, 1, 2)
        
        
        ### General Setting ###
        self.setLayout(layout)
        self.setWindowTitle('Corgi Control Panel')
        self.setStyleSheet('''background-color: dimgray; font-family: Comic Sans MS; font-size: 36px;''')
        self.show()
        

    def init_ros(self):
        rospy.init_node('corgi_control_panel')
        self.power_cmd_pub = rospy.Publisher('power/command', PowerCmdStamped, queue_size=10)
        self.trigger_pub = rospy.Publisher('trigger', TriggerStamped, queue_size=10)
        self.sensor_enable_pub = rospy.Publisher('sensor_enable', SensorEnableStamped, queue_size=10)
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CorgiControlPanel()
    sys.exit(app.exec_())
