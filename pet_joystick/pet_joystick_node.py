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
### Hardware: Joystick with analog 10K resistors for X, Y and Z
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
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from smbus2 import SMBus
import sys
import time
from math import modf
import signal
# Import the ADS1x15 module 
import Adafruit_ADS1x15
#from gpiozero import LED


class JoystickNode(Node): 
    
    #rosRunLED = LED(13)
    
    # Keep track of last joystick values. Used due to reducing communication of equal values.
    last_value_x = 0.0
    last_value_y = 0.0
    last_value_z = 0.0


    
    def __init__(self):
        super().__init__("joystick_node")
        
        # Set default topic-name for publishing. Accessed via ROS Parameters...
        self.declare_parameter( 'ros_topic_twist', 'twist/cmd_vel', ParameterDescriptor(description='ROS-topc name. Publish twist/velocity from Joystick [default "twist/cmd_vel"]') )
        self.ROS_TOPIC_TWIST = self.get_parameter('ros_topic_twist').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_twist_stamped', 'twist_stamped/cmd_vel', ParameterDescriptor(description='ROS-topc name. Publish twist/velocity from Joystick [default "twist_stamped/cmd_vel"]') )
        self.ROS_TOPIC_TWIST_STAMPED = self.get_parameter('ros_topic_twist_stamped').get_parameter_value().string_value

        self.declare_parameter( 'ros_topic_raw', 'raw/joystick', ParameterDescriptor(description='ROS-topc name. Publish twist/velocity from Joystick [default "raw/joystick"]') )
        self.ROS_TOPIC_RAW = self.get_parameter('ros_topic_raw').get_parameter_value().string_value

        # Set default ADC-I2C address. Accessed via ROS Parameters...
        self.declare_parameter( 'adc_i2c_address', "0x48", ParameterDescriptor(description='ADC I2C address [default "0x48"]') )
        self.ADC_I2C_ADDRESS = self.get_parameter('adc_i2c_address').get_parameter_value().string_value

        # Set default mapping for ADC-channel vs. X-axis/Angular, Y-axis/linear, Z-axis/Twist address. Accessed via ROS Parameters...
        self.declare_parameter( 'adc_x_channel', 3, ParameterDescriptor(description='ADC channel used by X-axis (Angular=Left/Right) [default 3]') )
        self.ADC_X_CHANNEL = self.get_parameter('adc_x_channel').value

        self.declare_parameter( 'adc_y_channel', 1, ParameterDescriptor(description='ADC channel used by Y-axis (Linear=Up/Down) [default 1]') )
        self.ADC_Y_CHANNEL = self.get_parameter('adc_y_channel').value
        
        self.declare_parameter( 'adc_z_channel', 2, ParameterDescriptor(description='ADC channel used by Z-axis (Twist the stick).[default 2]') )
        self.ADC_Z_CHANNEL = self.get_parameter('adc_z_channel').value

        # Set valid range of values to treat as a "zero" (mid position) value for the joystick. Accessed via ROS Parameters...
        self.declare_parameter( 'zero_range_min', -5, ParameterDescriptor(description='Joystick "Zero" range [default -5]') )
        self.ZERO_RANGE_MIN = self.get_parameter( 'zero_range_min' ).value

        self.declare_parameter( 'zero_range_max', +5, ParameterDescriptor(description='Joystick "Zero" range [default +5]') )
        self.ZERO_RANGE_MAX = self.get_parameter( 'zero_range_max' ).value

        # Set +/- linear "dead range" when performaing Angular-only (when only turn/twist and no linear movement). Accessed via ROS Parameters...
        self.declare_parameter( 'angular_only', 30, ParameterDescriptor(description='Joystick "Zero" range, when only turning  [default +/-30%]') )
        self.ANGULAR_ONLY = self.get_parameter( 'angular_only' ).value

        # Use polarity to change if values increase or decrease going left to right or up to down. Accessed via ROS Parameters...
        self.declare_parameter( 'x_polarity', -1, ParameterDescriptor(description='Joystick polarity in Angular(X=Left/Right) direction [default=-1]') )
        self.X_POLARITY = self.get_parameter( 'x_polarity' ).value

        self.declare_parameter( 'y_polarity', 1, ParameterDescriptor(description='Joystick polarity in Linear(Y=Up/Down) direction[default=1]') )
        self.Y_POLARITY = self.get_parameter( 'y_polarity' ).value
        
        self.declare_parameter( 'z_polarity', -1, ParameterDescriptor(description='Joystick polarity in Linear(Y=Up/Down) direction[default=-1]') )
        self.Z_POLARITY = self.get_parameter( 'z_polarity' ).value 

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

            # Create topic publisher
            self.pub_Twist = self.create_publisher(Twist, self.ROS_TOPIC_TWIST ,10)
            self.pub_TwistStamped = self.create_publisher(TwistStamped, self.ROS_TOPIC_TWIST_STAMPED ,10)
            self.pub_Joy = self.create_publisher(Joy, self.ROS_TOPIC_RAW ,10)

            # Set cycle time (Hz) how often to read the joystick position.
            self.joy_timer = self.create_timer(self.CYCLE_TIMER , self.process_joy) 
            # TODO: self.rosRunLED.on()

            # Some basic information on the console
            self.get_logger().info("joystick_node has started")
            self.get_logger().info("- A/D: " + self.ADC_I2C_ADDRESS + ", X-chn: " + str(self.ADC_X_CHANNEL)+ ", Y-chn: " + str(self.ADC_Y_CHANNEL)+ ", Z-chn: " + str(self.ADC_Z_CHANNEL) )
            self.get_logger().info("- A/D sampling: " + str(100 *self.CYCLE_TIMER) +"Hz, Topic: " + self.ROS_TOPIC_TWIST )
        except:
            # Note: a permission error can be fixed with a "sudo chmod a+rw /dev/i2c-1"
            self.get_logger().error("joystick_node initialization:"  + str(sys.exc_info()[1]) )
            self.exit = True

       
        #########################################################################################################
        # 1) Initiate & assign start values for the ros-msg.
        # 2) Publish a first time
        
        # cmd_vel
        # Shared/Common Twist.msg (from geometry_msgs.msg)        
        self.msg_twist = Twist()
        self.msg_twist.linear.x = 0.0
        self.msg_twist.linear.y = 0.0
        self.msg_twist.linear.z = 0.0
        self.pub_Twist.publish(self.msg_twist)

        # Shared/Common Header.msg (from std_msgs.msg)
        current_time = modf(time.time())
        self.msg_common_stamped_header = Header()
        self.msg_common_stamped_header.frame_id = 'Joystick#1'
        self.msg_common_stamped_header.stamp.sec = int(current_time[1])
        self.msg_common_stamped_header.stamp.sec = nanosec = int(current_time[0] * 1000000000) & 0xffffffff

        # cmd_vel
        # TwistStamped.msg (from geometry_msgs.msg) share/reuse the common Twist.msg & Header.msg
        self.msg_twistStamped = TwistStamped()
        self.msg_twistStamped.header = self.msg_common_stamped_header
        self.msg_twistStamped.twist = self.msg_twist
        self.pub_TwistStamped.publish(self.msg_twistStamped)

        # Joystick "raw-data" publisher (-100% <-> +100%)
        # Joy.msg (from sensor_msgs.msg) share/reuse common Header.msg
        self.msg_joy = Joy()
        self.msg_joy.header = self.msg_common_stamped_header
        self.msg_joy.axes = [0.0, 0.0, 0.0]
        self.msg_joy.buttons = [0]
        self.pub_Joy.publish(self.msg_joy)

    def process_joy(self):
        # Read values from potetinometers X, Y and Z/Twist(pot. in the stick)
        # With gain=1 the "raw"-value(1...26400) and middle 13200
        # With gain=2/3 the "raw"-value(1...17600) and middle 8900
        # My ads1115 needed a little sleep between measuerments to settle on a good value
        joyRaw_x= self.adc.read_adc(self.ADC_X_CHANNEL, gain=1, data_rate=128) # ADS1115 channel 3. 
        time.sleep(0.01)

        joyRaw_y= self.adc.read_adc(self.ADC_Y_CHANNEL, gain=1, data_rate=128) # ADS1115 channel 1
        time.sleep(0.01)

        joyRaw_z= self.adc.read_adc(self.ADC_Z_CHANNEL, gain=1, data_rate=128) # ADS1115 channel 2
        time.sleep(0.01) 
 
        # self.get_logger().info("Raw: x=" + str(joyRaw_x) + " y=" + str(joyRaw_y) + " z=" + str(joyRaw_z))

        # Convert to a value bettween -100 and +100 and number of distict values set by the granularity
        value_x = (round(((round(joyRaw_x/132) - 100) * self.X_POLARITY) / self.GRANULARITY)) * self.GRANULARITY
        value_y = (round(((round(joyRaw_y/132) - 100) * self.Y_POLARITY) / self.GRANULARITY)) * self.GRANULARITY
        value_z = (round(((round(joyRaw_z/132) - 100) * self.Z_POLARITY) / self.GRANULARITY)) * self.GRANULARITY
        
        # "Dead-zone"-Rule no.1: IF Joystick in neutral/middle. THEN let the value in between ZERO_RANGE_MAX...ZERO_RANGE_MIN be forced to Zero!
        if value_x <= self.ZERO_RANGE_MAX and value_x >= self.ZERO_RANGE_MIN :
            value_x = 0
        if value_y <= self.ZERO_RANGE_MAX and value_y >= self.ZERO_RANGE_MIN :
            value_y = 0
        if value_z <= self.ZERO_RANGE_MAX and value_z >= self.ZERO_RANGE_MIN :
            value_z = 0
        
        # Only output a twist message when the joystick values change,
        # "Dead-zone"-Rule no.2: IF stick is within Y=+/-30%. THEN expose only angular value and set linear to Zero.
        # ...otherwise 
        # If nothing published for N loops/iterations - Then send the last value again.
        doPublish = False


        self.republish_counter -= 1

        if  (abs(value_y) >= self.ANGULAR_ONLY):
            # State of stick = "High Linear speed". Updating both Angular & Linear velocity...
            # self.get_logger().info("Both Angular & Linear x=" + str(value_x) + " y=" + str(value_y) + " z=" + str(value_z))
            self.msg_twist.linear.x  = round(value_y * self.LINEAR_SCALING , 2)
            self.msg_twist.angular.z = round(value_x * self.ANGULAR_SCALING, 2)
            self.msg_joy.axes[0] = value_x
            self.msg_joy.axes[1] = value_y
            self.msg_joy.axes[2] = value_z
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if (self.last_value_y != value_y) or (self.last_value_x != value_x) or (self.last_value_z != value_z):
                # Change in y value
                # self.get_logger().info("-----Publish!")
                doPublish = True

        elif (abs(value_x) == 0.0) and (self.ZERO_RANGE_MAX < abs(value_y) < self.ANGULAR_ONLY ) :
            # State of stick = "Low Linear speed(Y)". Update Only Linear and set Angular to Zero
            # self.get_logger().info("Linear only x=" + str(value_x) + " y=" + str(value_y) + " z=" + str(value_z))
            self.msg_twist.linear.x  = round(value_y * self.LINEAR_SCALING , 2)
            self.msg_twist.angular.z = 0.0
            self.msg_joy.axes[0] = value_x
            self.msg_joy.axes[1] = 0.0
            self.msg_joy.axes[2] = value_z
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if (self.last_value_y != value_y) or (self.last_value_x != value_x) or (self.last_value_z != value_z):
                # Change in y value
                # self.get_logger().info("-----Publish!")
                doPublish = True

        elif (value_x != 0.0) and (0.0 <= abs(value_y) < self.ANGULAR_ONLY):
            # State of stick = "No Linear speed - Angular only".
            # self.get_logger().info("Angular Only x=" + str(value_x) + " y=" + str(value_y) + " z=" + str(value_z))
            self.msg_twist.linear.x  = 0.0
            self.msg_twist.angular.z = round(value_x * self.ANGULAR_SCALING, 2)
            self.msg_joy.axes[0] = value_x
            self.msg_joy.axes[1] = 0.0
            self.msg_joy.axes[2] = value_z
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if (self.last_value_y != value_y) or (self.last_value_x != value_x) or (self.last_value_z != value_z):
                # Change in x value
                # self.get_logger().info("-----Publish!")
                doPublish = True

        else :
            # State of stick = In neutral/middle position (aka. "Ground Zero")
            # self.get_logger().info("Ground Zero x=" + str(value_x) + " y=" + str(value_y) + " z=" + str(value_z))
            self.msg_twist.linear.x  = 0.0
            self.msg_twist.angular.z = 0.0
            self.msg_joy.axes[0] = 0.0
            self.msg_joy.axes[1] = 0.0
            self.msg_joy.axes[2] = value_z
            # Have the stick been moved since last time? Yes=Then Publish new messages
            if (self.last_value_y != value_y) or (self.last_value_x != value_x) or (self.last_value_z != value_z):
                # self.get_logger().info("-----Publish!")
                doPublish = True

        if doPublish or self.republish_counter == 0:
            # Update shared/common stamped header
            current_time = modf(time.time())
            self.msg_common_stamped_header.stamp.sec = int(current_time[1])
            self.msg_common_stamped_header.stamp.sec = nanosec = int(current_time[0] * 1000000000) & 0xffffffff

            self.pub_Twist.publish(self.msg_twist)
            self.pub_TwistStamped.publish(self.msg_twistStamped)
            self.pub_Joy.publish(self.msg_joy)

        # Save current value, so we can see if the stick have been moved during next lap.
        self.last_value_x = value_x
        self.last_value_y = value_y
        self.last_value_z = value_z
             
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
        # TODO: node.rosRunLED.off()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
