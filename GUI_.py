from PyQt5 import QtCore, QtGui, QtWidgets
import RPi.GPIO as GPIO
import time
import math
#import numpy as np

def setup_gpio():
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Ui_MainWindow.BASE_SERVO_PIN, GPIO.OUT)
	GPIO.setup(Ui_MainWindow.SECOND_SERVO_PIN, GPIO.OUT)
	GPIO.setup(Ui_MainWindow.THIRD_SERVO_PIN, GPIO.OUT)
	GPIO.setup(Ui_MainWindow.FOURTH_SERVO_PIN, GPIO.OUT)

def set_servo(servo_pin, angle):
	duty = angle/18 + 2
	GPIO.output(servo_pin, True)
	pwm = GPIO.PWM(servo_pin, 50)
	pwm.start(0)
	pwm.ChangeDutyCycle(duty)
	time.sleep(1)
	GPIO.output(servo_pin, False)
	pwm.ChangeDutyCycle(0)
	pwm.stop()


class Ui_MainWindow(object):

    #Setting up the GPIO pins
    GPIO.setmode(GPIO.BCM)
    BASE_SERVO_PIN = 17
    SECOND_SERVO_PIN = 18
    THIRD_SERVO_PIN = 22
    FOURTH_SERVO_PIN = 27

    angle_dict = {
        'A': [116, 56, 66, 90],
        'B': [69, 56, 66, 90],
        'C': [11, 56, 66, 90],
        'D': [40, 56, 66, 90],
        'E': [56, 180, 77, 90]
    }
    
    def update_coordinates_label(self, x, y, z):
        self.X_Coordinate.display(x)
        self.Y_Coordinate.display(y)
        self.Z_Coordinate.display(z)
    
    def forward_kinematics_spherical_arm(self, theta_1, theta_2, theta_3, theta_4, link_length_1, link_length_2, link_length_3, link_length_4):
         x = (link_length_2 + (link_length_3 + link_length_4*math.cos(theta_4))*math.cos(theta_3))*math.cos(theta_2) + link_length_1*math.cos(theta_1)
         y = (link_length_2 + (link_length_3 + link_length_4*math.sin(theta_4))*math.sin(theta_3))*math.sin(theta_2) + link_length_1*math.sin(theta_1)
         return x, y, theta_1
         

    def update_end_effector_coordinates(self):
        if not self.lengths_submitted:
             return
        
        theta_1 = self.BaseDial.value()
        theta_2 = self.SecondDial.value()
        theta_3 = self.ThirdDial.value()
        theta_4 = self.FinalDial.value()
        link_length_1 = self.BaseComponentLength.value()
        link_length_2 = self.SecondComponentLength.value()
        link_length_3 = self.ThirdComponentLength.value()
        link_length_4 = self.FinalComponentLength.value()
        
        end_effector_coords = self.forward_kinematics_spherical_arm(theta_1, theta_2, theta_3, theta_4, link_length_1, link_length_2, link_length_3, link_length_4)
        
        x, y, phi = end_effector_coords
        self.update_coordinates_label(x, y, phi)
    
    def on_submit_button_clicked(self):
         self.lengths_submitted = True
         self.update_end_effector_coordinates()
    
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1077, 881)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        self.AnimationFrame = QtWidgets.QFrame(self.centralwidget)
        self.AnimationFrame.setGeometry(QtCore.QRect(0, 0, 381, 861))
        self.AnimationFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.AnimationFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.AnimationFrame.setObjectName("AnimationFrame")
        
        self.ForwardKinematicsFrame = QtWidgets.QFrame(self.centralwidget)
        self.ForwardKinematicsFrame.setGeometry(QtCore.QRect(380, 0, 691, 471))
        self.ForwardKinematicsFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.ForwardKinematicsFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.ForwardKinematicsFrame.setObjectName("ForwardKinematicsFrame")
        
        self.ComponentLengthFrame = QtWidgets.QFrame(self.ForwardKinematicsFrame)
        self.ComponentLengthFrame.setGeometry(QtCore.QRect(0, 40, 691, 131))
        self.ComponentLengthFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.ComponentLengthFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.ComponentLengthFrame.setObjectName("ComponentLengthFrame")
        
        self.LabelForLengthFrame = QtWidgets.QLabel(self.ComponentLengthFrame)
        self.LabelForLengthFrame.setGeometry(QtCore.QRect(10, 0, 261, 21))
        self.LabelForLengthFrame.setObjectName("LabelForLengthFrame")
        
        self.LabelForBaseLengthBox = QtWidgets.QLabel(self.ComponentLengthFrame)
        self.LabelForBaseLengthBox.setGeometry(QtCore.QRect(10, 30, 231, 16))
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.LabelForBaseLengthBox.setFont(font)
        self.LabelForBaseLengthBox.setObjectName("LabelForBaseLengthBox")
        
        self.BaseComponentLength = QtWidgets.QDoubleSpinBox(self.ComponentLengthFrame)
        self.BaseComponentLength.setGeometry(QtCore.QRect(260, 30, 71, 22))
        self.BaseComponentLength.setObjectName("BaseComponentLength")
        
        self.LabelForSecondLengthBox = QtWidgets.QLabel(self.ComponentLengthFrame)
        self.LabelForSecondLengthBox.setGeometry(QtCore.QRect(360, 30, 241, 16))
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.LabelForSecondLengthBox.setFont(font)
        self.LabelForSecondLengthBox.setObjectName("LabelForSecondLengthBox")
        
        self.SecondComponentLength = QtWidgets.QDoubleSpinBox(self.ComponentLengthFrame)
        self.SecondComponentLength.setGeometry(QtCore.QRect(610, 30, 71, 22))
        self.SecondComponentLength.setObjectName("SecondComponentLength")
        
        self.LabelForThirdLengthBox = QtWidgets.QLabel(self.ComponentLengthFrame)
        self.LabelForThirdLengthBox.setGeometry(QtCore.QRect(10, 60, 241, 16))
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.LabelForThirdLengthBox.setFont(font)
        self.LabelForThirdLengthBox.setObjectName("LabelForThirdLengthBox")
        
        self.ThirdComponentLength = QtWidgets.QDoubleSpinBox(self.ComponentLengthFrame)
        self.ThirdComponentLength.setGeometry(QtCore.QRect(260, 60, 71, 22))
        self.ThirdComponentLength.setObjectName("ThirdComponentLength")
        
        self.LabelForFinalLengthBox = QtWidgets.QLabel(self.ComponentLengthFrame)
        self.LabelForFinalLengthBox.setGeometry(QtCore.QRect(360, 60, 241, 16))
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.LabelForFinalLengthBox.setFont(font)
        self.LabelForFinalLengthBox.setObjectName("LabelForFinalLengthBox")
        
        self.FinalComponentLength = QtWidgets.QDoubleSpinBox(self.ComponentLengthFrame)
        self.FinalComponentLength.setGeometry(QtCore.QRect(610, 60, 71, 22))
        self.FinalComponentLength.setObjectName("FinalComponentLength")
        
        self.SubmitButton = QtWidgets.QPushButton(self.ComponentLengthFrame)
        self.SubmitButton.setGeometry(QtCore.QRect(300, 100, 75, 23))
        self.SubmitButton.setObjectName("SubmitButton")
        
        self.DialFrame = QtWidgets.QFrame(self.ForwardKinematicsFrame)
        self.DialFrame.setGeometry(QtCore.QRect(-1, 169, 691, 201))
        self.DialFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.DialFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.DialFrame.setObjectName("DialFrame")
        
        self.AnglesFrameLabel = QtWidgets.QLabel(self.DialFrame)
        self.AnglesFrameLabel.setGeometry(QtCore.QRect(10, 10, 281, 16))
        self.AnglesFrameLabel.setObjectName("AnglesFrameLabel")
        
        self.BaseDial = QtWidgets.QDial(self.DialFrame)
        self.BaseDial.setGeometry(QtCore.QRect(30, 40, 131, 131))
        self.BaseDial.setObjectName("BaseDial")
        
        self.SecondDial = QtWidgets.QDial(self.DialFrame)
        self.SecondDial.setGeometry(QtCore.QRect(180, 40, 131, 131))
        self.SecondDial.setObjectName("SecondDial")
        
        self.ThirdDial = QtWidgets.QDial(self.DialFrame)
        self.ThirdDial.setGeometry(QtCore.QRect(330, 40, 131, 131))
        self.ThirdDial.setObjectName("ThirdDial")
        
        self.FinalDial = QtWidgets.QDial(self.DialFrame)
        self.FinalDial.setGeometry(QtCore.QRect(480, 40, 131, 131))
        self.FinalDial.setObjectName("FinalDial")
        
        self.LabelForBaseAngle = QtWidgets.QLCDNumber(self.DialFrame)
        self.LabelForBaseAngle.setGeometry(QtCore.QRect(60, 170, 64, 23))
        self.LabelForBaseAngle.setObjectName("LabelForBaseAngle")
        
        self.LabelForSecondAngle = QtWidgets.QLCDNumber(self.DialFrame)
        self.LabelForSecondAngle.setGeometry(QtCore.QRect(210, 170, 64, 23))
        self.LabelForSecondAngle.setObjectName("LabelForSecondAngle")
        
        self.LabelForThirdAngle = QtWidgets.QLCDNumber(self.DialFrame)
        self.LabelForThirdAngle.setGeometry(QtCore.QRect(360, 170, 64, 23))
        self.LabelForThirdAngle.setObjectName("LabelForThirdAngle")
        
        self.LabelForFinalAngle = QtWidgets.QLCDNumber(self.DialFrame)
        self.LabelForFinalAngle.setGeometry(QtCore.QRect(510, 170, 64, 23))
        self.LabelForFinalAngle.setObjectName("LabelForFinalAngle")
        
        self.LabelForPointReachedFrame = QtWidgets.QLabel(self.ForwardKinematicsFrame)
        self.LabelForPointReachedFrame.setGeometry(QtCore.QRect(10, 380, 121, 16))
        self.LabelForPointReachedFrame.setObjectName("LabelForPointReachedFrame")
        
        self.LabelForXCoordinate = QtWidgets.QLabel(self.ForwardKinematicsFrame)
        self.LabelForXCoordinate.setGeometry(QtCore.QRect(20, 420, 91, 21))
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.LabelForXCoordinate.setFont(font)
        self.LabelForXCoordinate.setObjectName("LabelForXCoordinate")
        
        self.X_Coordinate = QtWidgets.QLCDNumber(self.ForwardKinematicsFrame)
        self.X_Coordinate.setGeometry(QtCore.QRect(110, 420, 64, 21))
        self.X_Coordinate.setObjectName("X_Coordinate")
        
        self.LabelForYCoordinate = QtWidgets.QLabel(self.ForwardKinematicsFrame)
        self.LabelForYCoordinate.setGeometry(QtCore.QRect(250, 420, 91, 21))
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.LabelForYCoordinate.setFont(font)
        self.LabelForYCoordinate.setObjectName("LabelForYCoordinate")
        
        self.Y_Coordinate = QtWidgets.QLCDNumber(self.ForwardKinematicsFrame)
        self.Y_Coordinate.setGeometry(QtCore.QRect(340, 420, 64, 21))
        self.Y_Coordinate.setObjectName("Y_Coordinate")
        
        self.LabelForForwardKinematicsFrame = QtWidgets.QLabel(self.ForwardKinematicsFrame)
        self.LabelForForwardKinematicsFrame.setGeometry(QtCore.QRect(0, 0, 691, 41))
        self.LabelForForwardKinematicsFrame.setAlignment(QtCore.Qt.AlignCenter)
        self.LabelForForwardKinematicsFrame.setObjectName("LabelForForwardKinematicsFrame")
        
        self.LabelForZCoordinate = QtWidgets.QLabel(self.ForwardKinematicsFrame)
        self.LabelForZCoordinate.setGeometry(QtCore.QRect(460, 420, 91, 21))
        font = QtGui.QFont()
        font.setPointSize(7)
        font.setBold(True)
        font.setWeight(75)
        self.LabelForZCoordinate.setFont(font)
        self.LabelForZCoordinate.setObjectName("LabelForZCoordinate")
        
        self.Z_Coordinate = QtWidgets.QLCDNumber(self.ForwardKinematicsFrame)
        self.Z_Coordinate.setGeometry(QtCore.QRect(550, 420, 64, 21))
        self.Z_Coordinate.setObjectName("Z_Coordinate")
        
        self.InverseKinematicsFrame = QtWidgets.QFrame(self.centralwidget)
        self.InverseKinematicsFrame.setGeometry(QtCore.QRect(379, 469, 691, 391))
        self.InverseKinematicsFrame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.InverseKinematicsFrame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.InverseKinematicsFrame.setObjectName("InverseKinematicsFrame")
        
        self.LabelForInverseKinematicsFrame = QtWidgets.QLabel(self.InverseKinematicsFrame)
        self.LabelForInverseKinematicsFrame.setGeometry(QtCore.QRect(0, 0, 691, 41))
        self.LabelForInverseKinematicsFrame.setAlignment(QtCore.Qt.AlignCenter)
        self.LabelForInverseKinematicsFrame.setObjectName("LabelForInverseKinematicsFrame")
        
        self.LabelForInverseKinematicsBox = QtWidgets.QLabel(self.InverseKinematicsFrame)
        self.LabelForInverseKinematicsBox.setGeometry(QtCore.QRect(6, 50, 221, 20))
        self.LabelForInverseKinematicsBox.setObjectName("LabelForInverseKinematicsBox")
        
        self.RadioButtonForA = QtWidgets.QRadioButton(self.InverseKinematicsFrame)
        self.RadioButtonForA.setGeometry(QtCore.QRect(50, 110, 71, 41))
        self.RadioButtonForA.setObjectName("RadioButtonForA")
        
        self.RadioButtonForB = QtWidgets.QRadioButton(self.InverseKinematicsFrame)
        self.RadioButtonForB.setGeometry(QtCore.QRect(160, 110, 71, 41))
        self.RadioButtonForB.setObjectName("RadioButtonForB")
        
        self.RadioButtonForC = QtWidgets.QRadioButton(self.InverseKinematicsFrame)
        self.RadioButtonForC.setGeometry(QtCore.QRect(270, 110, 71, 41))
        self.RadioButtonForC.setObjectName("RadioButtonForC")
        
        self.RadioButtonForD = QtWidgets.QRadioButton(self.InverseKinematicsFrame)
        self.RadioButtonForD.setGeometry(QtCore.QRect(380, 110, 71, 41))
        self.RadioButtonForD.setObjectName("RadioButtonForD")
        
        self.RadioButtonForE = QtWidgets.QRadioButton(self.InverseKinematicsFrame)
        self.RadioButtonForE.setGeometry(QtCore.QRect(490, 110, 71, 41))
        self.RadioButtonForE.setObjectName("RadioButtonForE")
        
        MainWindow.setCentralWidget(self.centralwidget)
        
        self.BaseDial.setRange(0, 180)
        self.SecondDial.setRange(0, 180)
        self.ThirdDial.setRange(0, 180)
        self.FinalDial.setRange(0, 180)
        
        self.BaseDial.setSingleStep(1)
        self.SecondDial.setSingleStep(1)
        self.ThirdDial.setSingleStep(1)
        self.FinalDial.setSingleStep(1)
        
        self.BaseDial.setWrapping(True)
        self.SecondDial.setWrapping(True)
        self.ThirdDial.setWrapping(True)
        self.FinalDial.setWrapping(True)

        self.BaseDial.valueChanged.connect(self.updateBaseAngle)
        self.SecondDial.valueChanged.connect(self.updateSecondAngle)
        self.ThirdDial.valueChanged.connect(self.updateThirdAngle)
        self.FinalDial.valueChanged.connect(self.updateFinalAngle)
        
        self.BaseDial.valueChanged.connect(self.update_end_effector_coordinates)
        self.SecondDial.valueChanged.connect(self.update_end_effector_coordinates)
        self.ThirdDial.valueChanged.connect(self.update_end_effector_coordinates)
        self.FinalDial.valueChanged.connect(self.update_end_effector_coordinates)
        
        self.BaseDial.valueChanged.connect(self.updateBaseAngleLabel)
        self.SecondDial.valueChanged.connect(self.updateSecondAngleLabel)
        self.ThirdDial.valueChanged.connect(self.updateThirdAngleLabel)
        self.FinalDial.valueChanged.connect(self.updateFinalAngleLabel)
        
        # Connect radio buttons to the function
        self.RadioButtonForA.clicked.connect(lambda: self.moveServosForRadioButton('A'))
        self.RadioButtonForB.clicked.connect(lambda: self.moveServosForRadioButton('B'))
        self.RadioButtonForC.clicked.connect(lambda: self.moveServosForRadioButton('C'))
        self.RadioButtonForD.clicked.connect(lambda: self.moveServosForRadioButton('D'))
        self.RadioButtonForE.clicked.connect(lambda: self.moveServosForRadioButton('E'))
        
        self.retranslateUi(MainWindow)
        
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        self.SubmitButton.clicked.connect(self.on_submit_button_clicked)
        
        self.lengths_submitted = False

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        
        self.LabelForLengthFrame.setText(_translate("MainWindow", "LENGTH OF THE MANIPULATOR COMPONENTS:"))
        self.LabelForBaseLengthBox.setText(_translate("MainWindow", "LENGTH OF THE BASE COMPONENT (IN cm):"))
        self.LabelForSecondLengthBox.setText(_translate("MainWindow", "LENGTH OF THE SECOND COMPONENT (IN cm):"))
        self.LabelForThirdLengthBox.setText(_translate("MainWindow", "LENGTH OF THE THIRD COMPONENT (IN cm):"))
        self.LabelForFinalLengthBox.setText(_translate("MainWindow", "LENGTH OF THE FINAL COMPONENT (IN cm):"))
        
        self.SubmitButton.setText(_translate("MainWindow", "SUBMIT"))
        
        self.AnglesFrameLabel.setText(_translate("MainWindow", "ANGLES OF THE RESPECTIVE SERVOS:"))
        
        self.LabelForPointReachedFrame.setText(_translate("MainWindow", "POINT REACHED:"))
        
        self.LabelForXCoordinate.setText(_translate("MainWindow", "X-COORDINATE:"))
        
        self.LabelForYCoordinate.setText(_translate("MainWindow", "Y-COORDINATE:"))
        
        self.LabelForForwardKinematicsFrame.setText(_translate("MainWindow", "FORWARD KINEMATICS"))
        
        self.LabelForZCoordinate.setText(_translate("MainWindow", "Z-COORDINATE:"))
        
        self.LabelForInverseKinematicsFrame.setText(_translate("MainWindow", "INVERSE KINEMATICS"))
        
        self.LabelForInverseKinematicsBox.setText(_translate("MainWindow", "CHOOSE THE POINT YOU WANT TO GO TO:"))
        
        self.RadioButtonForA.setText(_translate("MainWindow", "Point A"))
        self.RadioButtonForB.setText(_translate("MainWindow", "Point B"))
        self.RadioButtonForC.setText(_translate("MainWindow", "Point C"))
        self.RadioButtonForD.setText(_translate("MainWindow", "Point D"))
        self.RadioButtonForE.setText(_translate("MainWindow", "Point E"))

    def updateBaseAngle(self):
        # Update the angle of the Base Servo based on the dial value
        # Also update the calculated point reached using calculatePointReached
        base_angle = float(self.BaseDial.value())
        set_servo(Ui_MainWindow.BASE_SERVO_PIN, base_angle)
        
        second_angle = float(self.SecondDial.value())
        third_angle = float(self.ThirdDial.value())
        fourth_angle = float(self.FinalDial.value())
        
        #self.calculatePointReached()
        
    def updateSecondAngle(self):
        # Update the angle of the Second Servo based on the dial value
        # Also update the calculated point reached using calculatePointReached
        second_angle = float(self.SecondDial.value())
        set_servo(Ui_MainWindow.SECOND_SERVO_PIN, second_angle)
        
        base_angle = float(self.BaseDial.value())
        third_angle = float(self.ThirdDial.value())
        fourth_angle = float(self.FinalDial.value())
        
        #self.calculatePointReached()

    def updateThirdAngle(self, value):
        # Update the angle of the Third Servo based on the dial value
        third_angle = float(self.ThirdDial.value())
        set_servo(Ui_MainWindow.THIRD_SERVO_PIN, third_angle)
        
        base_angle = float(self.BaseDial.value())
        second_angle = float(self.SecondDial.value())
        fourth_angle = float(self.FinalDial.value())
        

    def updateFinalAngle(self, value):
        # Update the angle of the Final Servo based on the dial value
        # Also update the calculated point reached using calculatePointReached
        fourth_angle = float(self.FinalDial.value())
        set_servo(Ui_MainWindow.FOURTH_SERVO_PIN, fourth_angle)
        
        base_angle = float(self.BaseDial.value())
        second_angle = float(self.SecondDial.value())
        third_angle = float(self.ThirdDial.value())
        
        #self.calculatedPointReached()
    
    def updateBaseAngleLabel(self, value):
        self.LabelForBaseAngle.display(value)

    def updateSecondAngleLabel(self, value):
        self.LabelForSecondAngle.display(value)

    def updateThirdAngleLabel(self, value):
        self.LabelForThirdAngle.display(value)

    def updateFinalAngleLabel(self, value):
        self.LabelForFinalAngle.display(value)
        
    
    def moveServosForRadioButton(self, key):
        if key in Ui_MainWindow.angle_dict:
            angles = Ui_MainWindow.angle_dict[key]
            set_servo(Ui_MainWindow.BASE_SERVO_PIN, angles[0])
            set_servo(Ui_MainWindow.SECOND_SERVO_PIN, angles[1])
            set_servo(Ui_MainWindow.THIRD_SERVO_PIN, angles[2])
            set_servo(Ui_MainWindow.FOURTH_SERVO_PIN, angles[3])


if __name__ == "__main__":
	import sys
	app = QtWidgets.QApplication(sys.argv)
	setup_gpio() #Calling GPIO setup function before creating the UI
	MainWindow = QtWidgets.QMainWindow()
	ui = Ui_MainWindow()
	ui.setupUi(MainWindow)
	MainWindow.show()
	sys.exit(app.exec_())
