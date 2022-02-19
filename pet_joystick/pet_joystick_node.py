#!/usr/bin/env python3'
# coding = utf-8
########################################################################################
###
### Maintainer: stefan.kull@gmail.com
### Inspired by: https://github.com/somervda/ourbotmanager_ros.git
### 
### Input: Analog X + Y + Twist(pot.on the stick) from analog joystick
### Output: micro-ROS node (ROS2) that publish topic /cmd_vel with msg.type twist_stamped
###    Angular = X-axis = Pull stick Left/Right
###    Linear  = Y-axis = Pull stick Up/Down
###    Twist   = Z-axis = Turn/Twist stick  (Not used right now)
###
### Behaviour:
### 1) Once: Read/Set all the parameters
### 2) Repeatedly: Read analog joystick via ADC
### 2) Repeatedly: Transform indata to a +/-100% values
### 3) Repeatedly: Map where the stick are => Depending om location, then adjust behivaiur.
### 4) Repeatedly: Publish ros-topic
###
### Prerequisite:
### $ sudo apt install i2c-tools
### $ sudo apt install python3-pip
### $ sudo pip3 install smbus2
### $ sudo pip3 install adafruit-ads1x15
### $ sudo i2cdetect -y 1
### $ sudo chmod a+rw /dev/i2c-1
###
### Hardware: KY-053 Analog Digital Converter (ADS1115, 16-bit) via default I2C adr.=0x48
### Hardware: Joystick with analog 10K resistors for X, Y and Twist
### Host: Raspberry Pi 4(Ubuntu) via I2C
###
### Launch sequence:
### 1) $ ros2 run pet_mk_viii_joystick joystick_node 
###
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from smbus2 import SMBus
import sys
import time
import signal
# Import the ADS1x15 module 
import Adafruit_ADS1x15
#from gpiozero import LED


class JoystickNode(Node): 
    
    #rosRunLED = LED(13)
    
    # Keep track of last joystick values. Used due to reducing communication of equal values.
    last_value_x = 0.0
    last_value_y = 0.0


    
    def __init__(self):
        super().__init__("joystick_node")
        
        # Set default topic-name for publishing. Accessed via ROS Parameters...
        self.declare_parameter( 'ros_topic', 'cmd_vel', ParameterDescriptor(description='ROS-topc name. Publish velocity from Joystick [default "cmd_vel"]') )
        self.ROS_TOPIC = self.get_parameter('ros_topic').get_parameter_value().string_value

        # Set default ADC-I2C address. Accessed via ROS Parameters...
        self.declare_parameter( 'adc_i2c_address', "0x48", ParameterDescriptor(description='ADC I2C address [default "0x48"]') )
        self.ADC_I2C_ADDRESS = self.get_parameter('adc_i2c_address').get_parameter_value().string_value

        # Set default mapping for ADC-channel vs. X-axis/Angular, Y-axis/linear, Twist address. Accessed via ROS Parameters...
        self.declare_parameter( 'adc_x_channel', 3, ParameterDescriptor(description='ADC channel used by X-axis (Angular=Left/Right) [default 3]') )
        self.ADC_X_CHANNEL = self.get_parameter('adc_x_channel').value
        self.declare_parameter( 'adc_y_channel', 1, ParameterDescriptor(description='ADC channel used by Y-axis (Linear=Up/Down) [default 1]') )
        self.ADC_Y_CHANNEL = self.get_parameter('adc_y_channel').value
        self.declare_parameter( 'adc_twist_channel', 2, ParameterDescriptor(description='ADC channel used by Twist-axis.[default 2]') )
        self.ADC_TWIST_CHANNEL = self.get_parameter('adc_twist_channel').value

        # Set valid range of values to treat as a "zero" (mid position) value for the joystick. Accessed via ROS Parameters...
        self.declare_parameter( 'zero_range_min', -5, ParameterDescriptor(description='Joystick "Zero" range [default -5]') )
        self.ZERO_RANGE_MIN = self.get_parameter( 'zero_range_min' ).value
        self.declare_parameter( 'zero_range_max', +5, ParameterDescriptor(description='Joystick "Zero" range [default +5]') )
        self.ZERO_RANGE_MAX = self.get_parameter( 'zero_range_max' ).value

        # Set +/- linear "dead range" when performaing Angular-only (when only turn/twist and no linear movement). Accessed via ROS Parameters...
        self.declare_parameter( 'angular_only', 30, ParameterDescriptor(description='Joystick "Zero" range, when only turning  [default +/-30%]') )
        self.ANGULAR_ONLY = self.get_parameter( 'angular_only' ).value

        # Use polatity to change if values increase or decrease going left to right or up to down. Accessed via ROS Parameters...
        self.declare_parameter( 'angular_polarity', -1, ParameterDescriptor(description='Joystick polarity in Angular(X=Left/Right) direction [default=-1]') )
        self.X_RANGE_POLARITY = self.get_parameter( 'angular_polarity' ).value
        self.declare_parameter( 'linear_polarity', -1, ParameterDescriptor(description='Joystick polarity in Linear(Y=Up/Down) direction[default=-1]') )
        self.Y_RANGE_POLARITY = self.get_parameter( 'linear_polarity' ).value 

        # Granularity is the values stepsize between -100 and +100. Accessed via ROS Parameters...
        self.declare_parameter( 'granularity', 5, ParameterDescriptor(description='Joystick value step-size [default 5].') )
        self.GRANULARITY = self.get_parameter( 'granularity' ).value

        # Scaling factors for converting joystick values to twist magnitudes
        self.declare_parameter( 'angular_scaling', 0.01, ParameterDescriptor(description='Joystick scaling factor in Angular(X=Left/Right) direction [default 0.01*100rad/s]') )
        self.ANGULAR_SCALING = self.get_parameter( 'angular_scaling' ).value
        self.declare_parameter( 'linear_scaling', 0.02, ParameterDescriptor(description='Joystick scaling factor in Linear(Y=Up/Down) direction [default 0.02*100m/s]') )
        self.LINEAR_SCALING = self.get_parameter( 'linear_scaling' ).value

        # Cycle time - How often to read joystick position via A/D converter
        self.declare_parameter( 'cycle_timer', 0.1, ParameterDescriptor(description='Cycle time how often to read joystick position [default 0.1 sec(10Hz)]') )
        self.CYCLE_TIMER = self.get_parameter( 'cycle_timer' ).value

        # Republish every n number of cycles to make sure base got last value
        self.declare_parameter( 'cycles_publish', 10, ParameterDescriptor(description='Number of cycles/timer before publish [default 10]') )
        self.CYCLES_COUNTER = self.get_parameter( 'cycles_publish' ).value
        self.republish_counter = self.CYCLES_COUNTER

        exit = False
        # Check we can open the analog/digitala converter, ads1115, via I2C-interface.
        try:
            self.adc = Adafruit_ADS1x15.ADS1115( busnum=1, address=int(self.ADC_I2C_ADDRESS,0) )  # "0" works for both Dec and Hex strings...
            self.pub = self.create_publisher(Twist, self.ROS_TOPIC ,10)
            # Set cycle time (Hz) how often to read the joystick position.
            self.joy_timer = self.create_timer(self.CYCLE_TIMER , self.process_joy) 
            #self.rosRunLED.on()
            self.get_logger().info("joystick_node has started")
            self.get_logger().info("- A/D: " + self.ADC_I2C_ADDRESS + ", X-chn: " + str(self.ADC_X_CHANNEL)+ ", Y-chn: " + str(self.ADC_Y_CHANNEL)+ ", Twist-chn: " + str(self.ADC_TWIST_CHANNEL) )
            self.get_logger().info("- A/D sampling: " + str(100 *self.CYCLE_TIMER) +"Hz, Topic: " + self.ROS_TOPIC )
        except:
            # Note: a permission error can be fixed with a "sudo chmod a+rw /dev/i2c-1"
            self.get_logger().error("joystick_node initialization:"  + str(sys.exc_info()[1]) )
            self.exit = True


    def process_joy(self):
        # Read values from potetinometers X, Y and Twist(pot. in the stick)
        # My ads1115 needed a little sleep between measuerments to settle on a good value
        joyRaw_y= self.adc.read_adc(self.ADC_Y_CHANNEL, gain=1, data_rate=128) # ADS1115 channel 1 (Gain="2/3" or "1"??)
        time.sleep(0.01) 
        joyRaw_Twist= self.adc.read_adc(self.ADC_TWIST_CHANNEL, gain=2/3, data_rate=128) # ADS1115 channel 2
        time.sleep(0.01) 
        joyRaw_x= self.adc.read_adc(self.ADC_X_CHANNEL, gain=1, data_rate=128) # ADS1115 channel 3 (Gain="2/3" or "1"??)
        time.sleep(0.01) 

        # Convert to a value bettween -100 and +100 and number of distict values set by the granularity
        value_x = (round(((round(joyRaw_x/132) - 100) * self.X_RANGE_POLARITY) / self.GRANULARITY)) * self.GRANULARITY
        value_y = (round(((round(joyRaw_y/132) - 100) * self.Y_RANGE_POLARITY) / self.GRANULARITY)) * self.GRANULARITY
        if value_x <= self.ZERO_RANGE_MAX and value_x >= self.ZERO_RANGE_MIN :
            value_x = 0
        if value_y <= self.ZERO_RANGE_MAX and value_y >= self.ZERO_RANGE_MIN :
            value_y = 0
        
        # Only output a twist message when the joystick values change,
        # Expose "Twist only" when the stick is within Y=+/-30% = ANGULAR_ONLY
        # ...otherwise 
        # If nothing published for N seconds then send the last value again.
        doPublish = False
        msg = Twist()
        self.republish_counter -= 1

        if  (abs(value_y) >= self.ANGULAR_ONLY):
            # State of stick = "High Linear speed". Updating both Twist & Linear velocity...
            # debugg/ self.get_logger().info("Both Twist & Shout x=" + str(value_x) + " y=" + str(value_y))
            msg.linear.x  = round(value_y * self.LINEAR_SCALING , 2)
            msg.angular.z = round(value_x * self.ANGULAR_SCALING, 2)
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if self.last_value_y != value_y:
                # Change in y value
                doPublish = True

        elif (abs(value_x) == 0.0) and (5.0 < abs(value_y) < self.ANGULAR_ONLY ) :
            # State of stick = "Low Linear speed". Update Only Linear and set Twist to Zero
            # debugg/ self.get_logger().info("Linear only x=" + str(value_x) + " y=" + str(value_y))
            msg.linear.x  = round(value_y * self.LINEAR_SCALING , 2)
            msg.angular.z = 0.0
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if self.last_value_y != value_y:
                # Change in y value
                doPublish = True

        elif (value_x != 0.0) and (0.0 <= abs(value_y) < self.ANGULAR_ONLY):
            # State of stick = "No Linear speed - Twist only".
            # debugg/ self.get_logger().info("Twist Only x=" + str(value_x) + " y=" + str(value_y))
            msg.linear.x = 0.0
            msg.angular.z = round(value_x * self.ANGULAR_SCALING, 2)
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if self.last_value_x != value_x or self.last_value_y != value_y:
                # Change in x value
                doPublish = True

        else :
            # State of stick = In neutral/middle position (aka. "Ground Zero")
            # debugg/ self.get_logger().info("Ground Zero x=" + str(value_x) + " y=" + str(value_y))
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if self.last_value_y != 0 or self.last_value_x != 0:
                doPublish = True
        
        if doPublish or self.republish_counter == 0:
            self.republish_counter = self.CYCLES_COUNTER
            self.pub.publish(msg)

        # Save current value, so we can see if the stick have been moved during next lap.
        self.last_value_x = value_x
        self.last_value_y = value_y
             
def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("**** ☠️ Ctrl-C detected...")
    
    finally:
        print("**** 🪦 joystick_node ending... " + str(sys.exc_info()[1]) )
        # Time to clean up stuff!
        #node.rosRunLED.off()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
