#!/usr/bin/env python3
import os
import sys
import rospy
import subprocess
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from corgi_msgs.msg import *

class CorgiControlPanel(QWidget):
    def __init__(self):
        super(CorgiControlPanel, self).__init__()
        self.init_ui()
        self.init_ros()

    def init_ui(self):
        ### Digital Button ###
        self.label_digital = QLabel('Digital:', self)
        self.label_digital.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_digital = QButtonGroup(self)
        self.btn_group_digital.setExclusive(True)
        
        self.btn_digital_on = QPushButton('ON', self)
        self.btn_digital_off = QPushButton('OFF', self)
        
        btn_digital_list = [self.btn_digital_on, self.btn_digital_off]
        btn_digital_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px;}
                             QPushButton:checked {background-color: pink;}
                             QPushButton:hover:!checked {background-color: silver;}
                             QPushButton:hover:pressed {background-color: lightcoral; border-style: inset;}'''

        for btn in btn_digital_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_digital_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_digital.addButton(btn)
            
        self.btn_digital_off.setChecked(True)
        
        
        ### Power Button ###
        self.label_power = QLabel('Power:', self)
        self.label_power.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_power = QButtonGroup(self)
        self.btn_group_power.setExclusive(True)
        
        self.btn_power_on = QPushButton('ON', self)
        self.btn_power_off = QPushButton('OFF', self)
        
        btn_power_list = [self.btn_power_on, self.btn_power_off]
        btn_power_style = '''QPushButton {background-color: white; text-align: center; border-radius: 5px;}
                             QPushButton:checked {background-color: pink;}
                             QPushButton:hover:!checked {background-color: silver;}
                             QPushButton:hover:pressed {background-color: lightcoral; border-style: inset;}'''

        for btn in btn_power_list:
            btn.setCheckable(True)
            btn.setStyleSheet(btn_power_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_power.addButton(btn)
            
        self.btn_power_off.setChecked(True)
        

        ### Mode Switch Button ###
        self.label_mode = QLabel('Motor Mode Switch:')
        self.label_mode.setStyleSheet('color: white; font-weight: bold;')
        
        self.btn_group_mode = QButtonGroup(self)
        self.btn_group_mode.setExclusive(True)
        
        self.btn_rest_mode  = QPushButton('Rest Mode', self)
        self.btn_set_zero   = QPushButton('Set Zero', self)
        self.btn_hall_cal   = QPushButton('Hall Calibrate', self)
        self.btn_motor_mode = QPushButton('Motor Mode', self)
        
        btn_mode_list = [self.btn_rest_mode, self.btn_set_zero, self.btn_hall_cal, self.btn_motor_mode]
        btn_mode_style = '''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                            QPushButton:checked {background-color: palegreen; color: black;}
                            QPushButton:hover:!checked {background-color: silver; color: black;}
                            QPushButton:hover:pressed {background-color: mediumseagreen; border-style: inset;}'''
        
        for id, btn in enumerate(btn_mode_list):
            btn.setCheckable(True)
            btn.setStyleSheet(btn_mode_style)
            btn.clicked.connect(self.publish_power_cmd)
            self.btn_group_mode.addButton(btn, id)
        
        self.btn_rest_mode.setChecked(True)
        
            
        ### Data Output File Name ###
        self.label_output = QLabel('Output File Name (.bag):', self)
        self.label_output.setStyleSheet('color: white; font-weight: bold;')
        
        self.edit_output = QLineEdit('', self)
        self.edit_output.setStyleSheet('''background-color: white;''')
        
        
        ### CSV control Button ###
        self.label_input = QLabel('Input File Name (.csv):', self)
        self.label_input.setStyleSheet('color: white; font-weight: bold;')
        
        self.edit_input = QLineEdit('', self)
        self.edit_input.setStyleSheet('''background-color: white;''')
        
        self.btn_input = QPushButton('Select', self)
        self.btn_input.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                                        QPushButton:hover:!checked {background-color: silver; color: black;}
                                        QPushButton:hover:pressed {background-color: gray; border-style: inset;}''')
        self.btn_input.clicked.connect(self.select_orin_file)
        
        self.btn_csv = QPushButton('Run', self)
        self.btn_csv.setCheckable(True)
        self.btn_csv.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                                      QPushButton:checked {background-color: lightgray; color: black;}
                                      QPushButton:hover:!checked {background-color: lightgray; color: black;}
                                      QPushButton:hover:checked {background-color: silver; color: black;}
                                      QPushButton:hover:pressed {background-color: silver; border-style: inset;}''')
        self.btn_csv.clicked.connect(self.csv_control)
        
        
        ### Trigger Button ###
        self.btn_trigger = QPushButton('Trigger', self)
        self.btn_trigger.setCheckable(True)
        self.btn_trigger.setStyleSheet('''QPushButton {background-color: white; color: black; text-align: center; border-radius: 5px;}
                                          QPushButton:checked {background-color: skyblue; color: black;}
                                          QPushButton:hover:!checked {background-color: silver; color: black;}
                                          QPushButton:hover:checked {background-color: cornflowerblue; color: black;}
                                          QPushButton:hover:pressed {background-color: royalblue; border-style: inset;}''')
        self.btn_trigger.clicked.connect(self.publish_trigger_cmd)
        
        
        ### Reset Button ###
        self.btn_reset = QPushButton('Reset', self)
        self.btn_reset.setStyleSheet('''QPushButton {background-color: plum; color: black; text-align: center; border-radius: 5px;}
                                        QPushButton:hover:!checked {background-color: violet; color: black;}
                                        QPushButton:hover:pressed {background-color: darkviolet; border-style: inset;}''')
        self.btn_reset.clicked.connect(self.reset)
        
        
        ### Robot Status ###
        self.label_status = QLabel('Status:')
        self.label_status.setStyleSheet('color: white; font-weight: bold;')
        
        header_status = ['Digital', 'Power', 'Mode',
                         'LF Motor R', 'LF Motor L', 'RF Motor R', 'RF Motor L',
                         'RH Motor R', 'RH Motor L', 'LH Motor R', 'LH Motor L',]
        self.value_status = ['OFF', 'OFF', '-',
                            '-', '-', '-', '-',
                            '-', '-', '-', '-']
        
        self.label_status_headers = [QLabel(header) for header in header_status]
        self.label_status_values = [QLabel(value) for value in self.value_status]
        [label.setStyleSheet('color: antiquewhite;') for label in self.label_status_headers]
        [label.setStyleSheet('color: gold;') for label in self.label_status_values]
        [label.setFixedWidth(300) for label in self.label_status_values]
        [label.setAlignment(Qt.AlignCenter) for label in self.label_status_values]
        
        
        ### Vertical Line ###
        vlines = [QWidget() for i in range(5)]
        for vline in vlines:
            vline.setFixedWidth(5)
            vline.setStyleSheet('background-color: black;')
        
        
        ### Arrange Elements ###
        layout = QGridLayout()
        layout.setSpacing(20)
        layout.addWidget(self.label_digital,   0, 0, 1, 2)
        layout.addWidget(self.btn_digital_on,  1, 0, 1, 1)
        layout.addWidget(self.btn_digital_off, 1, 1, 1, 1)
        layout.addWidget(self.label_power,     2, 0, 1, 2)
        layout.addWidget(self.btn_power_on,    3, 0, 1, 1)
        layout.addWidget(self.btn_power_off,   3, 1, 1, 1)
        layout.addWidget(self.label_mode,      4, 0, 1, 2)
        layout.addWidget(self.btn_rest_mode,   5, 0, 1, 2)
        layout.addWidget(self.btn_set_zero,    6, 0, 1, 2)
        layout.addWidget(self.btn_hall_cal,    7, 0, 1, 2)
        layout.addWidget(self.btn_motor_mode,  8, 0, 1, 2)
        layout.addWidget(vlines[0],            0, 2, 12,1)
        layout.addWidget(self.label_output,    0, 3, 1, 2)
        layout.addWidget(self.edit_output,     1, 3, 1, 2)
        layout.addWidget(self.label_input,     2, 3, 1, 2)
        layout.addWidget(self.edit_input,      3, 3, 1, 1)
        layout.addWidget(self.btn_input,       3, 4, 1, 1)
        layout.addWidget(self.btn_csv,         4, 3, 1, 2)
        layout.addWidget(self.btn_trigger,     8, 3, 1, 2)
        layout.addWidget(self.btn_reset,       9, 3, 1, 2)
        layout.addWidget(vlines[1],            0, 5, 12,1)
        layout.addWidget(self.label_status,    0, 6, 1, 2)
        for i in range(len(header_status)):
            layout.addWidget(self.label_status_headers[i], i+1, 6, 1, 1)
            layout.addWidget(self.label_status_values[i],  i+1, 7, 1, 1)
        

        ### General Setting ###
        self.setLayout(layout)
        self.setWindowTitle('Corgi Control Panel')
        self.setStyleSheet('''background-color: dimgray; font-family: Ubuntu; font-size: 36px; padding: 5px''')
        self.setWindowFlag(Qt.WindowStaysOnTopHint)
        
        x = (QApplication.desktop().screenGeometry().width() - self.width())
        y = (QApplication.desktop().screenGeometry().height() - self.height())
        self.move(x//3, y//3)
        
        self.show()
        

    def init_ros(self):
        rospy.init_node('corgi_control_panel')
        self.power_cmd_pub = rospy.Publisher('power/command', PowerCmdStamped, queue_size=10)
        self.trigger_pub = rospy.Publisher('trigger', TriggerStamped, queue_size=10)
        self.sensor_enable_pub = rospy.Publisher('sensor_enable', SensorEnableStamped, queue_size=10)
        
        self.power_state_sub = rospy.Subscriber('/power/state', PowerStateStamped, self.update_status)
        
        self.recording_process = None
        
        
    def publish_power_cmd(self):
        power_cmd = PowerCmdStamped()
        
        power_cmd.header.stamp = rospy.Time.now()
        power_cmd.digital = self.btn_digital_on.isChecked()
        power_cmd.power = self.btn_power_on.isChecked()
        power_cmd.motor_mode = self.btn_group_mode.checkedId()
        
        self.power_cmd_pub.publish(power_cmd)
    
    
    def select_orin_file(self):
        pass
    
    
    def csv_control(self):
        if self.btn_csv.isChecked():
            self.btn_csv.setText('Stop')
        else:
            self.btn_csv.setText('Execute')
            
    
    def publish_trigger_cmd(self):
        trigger_cmd = TriggerStamped()
        
        trigger_cmd.header.stamp = rospy.Time.now()
        trigger_cmd.enable = self.btn_trigger.isChecked()
        trigger_cmd.output_filename = self.edit_output.text()
        
        if self.btn_trigger.isChecked() and self.edit_output.text() != '':
            rosbag_folder = f'{os.environ["HOME"]}/corgi_ws/corgi_ros_ws/recorded_data/'
            if not os.path.exists(rosbag_folder):
                os.makedirs(rosbag_folder)
            
            rosbag_filename = rosbag_folder+self.edit_output.text()
            while os.path.exists(f'{rosbag_filename}.bag'):
                rosbag_filename += '_'
            
            self.recording_process = subprocess.Popen(['rosbag', 'record', '-a', '-O', f'{rosbag_filename}.bag'])
            
            QTimer.singleShot(1000, lambda: self.trigger_pub.publish(trigger_cmd))
            
        else:
            self.trigger_pub.publish(trigger_cmd)
            
            if self.recording_process:
                self.recording_process.terminate()
                self.recording_process = None
                print('Rosbag is ternimated.')
            
    
    def reset(self):
        self.btn_digital_off.setChecked(True)
        self.btn_power_off.setChecked(True)
        self.btn_rest_mode.setChecked(True)
        self.btn_trigger.setChecked(False)
        
        self.publish_trigger_cmd()
        self.publish_power_cmd()
        
    
    def update_status(self, state):
        self.value_status[0] = 'ON' if state.digital else 'OFF'
        self.value_status[1] = 'ON' if state.power else 'OFF'
        self.value_status[2] = ['Rest Mode', 'Set Zero', 'Hall Calibrate', 'Motor Mode'][state.motor_mode]
        
        for i in range(len(self.value_status)):
            self.label_status_values[i].setText(self.value_status[i])
    
    
    def closeEvent(self, event):
        if self.recording_process:
            self.recording_process.terminate()
            self.recording_process = None
            print('Rosbag is ternimated.')
            
        self.power_state_sub.unregister()
        super(CorgiControlPanel, self).closeEvent(event)
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CorgiControlPanel()
    sys.exit(app.exec_())
