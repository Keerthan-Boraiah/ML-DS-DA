
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        'right':  find_nearest(msg.ranges[85:95]),
        'fright': find_nearest (msg.ranges[130:140]),
        'front':  find_nearest (msg.ranges[175:185]),
        'fleft':  find_nearest (msg.ranges[220:230]),
        'left':   find_nearest (msg.ranges[265:275]),
        
    }    
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=1), 1)

global listofSum
#listOfSum = [0]

def pid(dis):

    A = 0.9 #Initial Desigred Distance
    p = 0.9 #Proportion
    i = 0 #Integral
    d = 0 #Derivative
    
    listOfSum = [0] #List for calculating total error sum
    
    e = A - dis #Error Calculation
    
    listOfSum.append(e) #Appending all errors to listofSum so we can sum in final 
     
    ei = sum(listOfSum) #Totalling all errors of list
    
    ed = listOfSum[-1] - listOfSum[-2] #Last Error - The Previous Error
    
    final = (p*e + ei * i + d * ed) #PID FORMULA (Proportion*Error + Total Error*Integral + Derivative*(Last Error - The Previous Error) )
    
    print('final velocity: ',final)
    return final

#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    
    print("Min distance in Right region: ", regions['right'])
    angValright = pid(regions['right']) #DEFINING OUR pid FUNCTION
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()
    
    #If an obstacle is found to be within 0.25 of the LiDAR sensors front region the linear velocity is set to 0 (turtlebot stops)
    msg.linear.x = -0.05
    msg.angular.z = angValright #FEEDING THE PID VALUE
    return msg
    #if there is no obstacle in front of the robot, it continues to move forward
    #else:
        #if (regions['front']) < 0.25:
            #for i in range(5):
                #msg.linear.x = -0.1
                #msg.angular.z = angValright
            #return msg
    #else:
        #msg.linear.x = -0.1
        #msg.angular.z = 0.0
        #return msg

#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()
        # stop the robot
    except:
        stop()
        # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    



 	
