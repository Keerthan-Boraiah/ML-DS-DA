
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
        'front':  find_nearest (msg.ranges[175:185]),
        'right':  find_nearest(msg.ranges[85:95]),
        'fright': find_nearest (msg.ranges[110:120]),
        'fleft':  find_nearest (msg.ranges[220:230]),
        'left':   find_nearest (msg.ranges[265:275]),
        'back' : find_nearest (msg.ranges[5:355]),
        'bleft' : find_nearest (msg.ranges[310:320]),
        'bright' : find_nearest (msg.ranges[60:70])
    }    
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=1), 1)

#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    print("Min distance in Front Right region: ", regions['fright'])
    print("Min distance in Back Right region: ", regions['bright'])
    
#*******************************************************************************************************************************************************************************************************************************    
    global LinearX,AngularZ
    
    X1 = regions['fright']
    X2 = regions['bright']

    FRS = []
    BRS = []

    #Create Function to Define FRONT RIGHT SENSOR
    def FRS_Membership():
        Near = [0,0.4,0.6]
        Middle = [0.4,0.6,0.8]
        Far = [0.6,0.8,1.001]        
        return Near,Middle,Far

    #Create Function to Define BACK RIGHT SENSOR
    def BRS_Membership():
        Near = [0,0.4,0.6]
        Middle = [0.4,0.6,0.8]
        Far = [0.6,0.8,1.001]
        return Near,Middle,Far

    #Create Function to Define SPEED
    def Speed():
        Slow = [0,0.1,0.2]
        Medium = [0.2,0.3,0.4]
        Fast = [0.4,0.5,0.6]
        return Slow,Medium,Fast

    #Create Function to Define TURNING
    def Turning():
        Right = [-1.5,-1.2,-0.5]
        No = [-0.5, 0, 0.5]
        Left = [0.5, 1.2, 1.5]
        return Right,No,Left

    def memberships(x,y):
        
        #************************FRS Right or left or Both Checker of FIRING RULES*****************************
        if ((x >=FRS_Membership()[0][0] and x<= FRS_Membership()[0][-1]) and (x >=FRS_Membership()[1][0] and x<= FRS_Membership()[1][-1])):
            FRS.append('between Near and Middle')
        elif ((x >=FRS_Membership()[1][0] and x<= FRS_Membership()[1][-1]) and (x >=FRS_Membership()[2][0] and x<= FRS_Membership()[2][-1])):
            FRS.append('between Middle and Far')
        elif (x >=FRS_Membership()[0][0] and x<= FRS_Membership()[0][-1]):
            FRS.append('Near')
        elif (x >=FRS_Membership()[1][0] and x<= FRS_Membership()[1][-1]):
            FRS.append('Middle')
        else:
            FRS.append('Far')
            
        #************************BRS Right or left or Both Checker of FIRING RULES*****************************
        if ((y >=BRS_Membership()[0][0] and y<= BRS_Membership()[0][-1]) and (y >=BRS_Membership()[1][0] and y<= BRS_Membership()[1][-1])):
            BRS.append('between Near and Middle')
        elif ((y >=BRS_Membership()[1][0] and y<= BRS_Membership()[1][-1]) and (y >=BRS_Membership()[2][0] and y<= BRS_Membership()[2][-1])):
            BRS.append('between Middle and Far')
        elif (y >=BRS_Membership()[0][0] and y<= BRS_Membership()[0][-1]):
            BRS.append('Near')
        elif (y >=BRS_Membership()[1][0] and y<= BRS_Membership()[1][-1]):
            BRS.append('Middle')
        else:
            BRS.append('Far')    

    memberships(X1,X2)

    #*************************Create Function for FRS RIGHT and LEFT for fired rules****************************
    def fuzzyvalues1(x,aP,bP,cP,aA,bA,cA,aG,bG,cG):
        global FRS_Near,FRS_Middle,FRS_Far
        FRS_Near = 0
        FRS_Middle = 0
        FRS_Far = 0
        
        #if Condition for getting right and left value of Near and Middle
        if FRS[0] == 'between Near and Middle':
            
            if min([FRS_Membership()[0][0],FRS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                FRS_Near += LeftPoor
                #a.append(['Near',LeftPoor])
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                FRS_Near += RightPoor
                
            if min([FRS_Membership()[1][0],FRS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FRS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FRS_Middle += RightAverage
        
        #if Condition for getting right and left value of Middle and far
        elif FRS[0] == 'between Middle and Far':
            
            if min([FRS_Membership()[1][0],FRS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FRS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FRS_Middle += RightAverage
                
            if min([FRS_Membership()[2][0],FRS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                FRS_Far += LeftGood 
            else:
                RightGood = abs((cG-x)/(cG-bG))
                FRS_Far += RightGood
        
        #if Condition for getting right and left value of Near
        elif FRS[0] == 'Near':
            
            if min([FRS_Membership()[0][0],FRS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                FRS_Near += LeftPoor
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                FRS_Near += RightPoor
        
        #if Condition for getting right and left value of Middle
        elif FRS[0] == 'Middle':
            
            if min([FRS_Membership()[1][0],FRS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FRS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FRS_Middle += RightAverage
        
        #if Condition for getting right and left value of Far
        else:
            
            if min([FRS_Membership()[2][0],FRS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                FRS_Far += LeftGood
            else:
                RightGood = abs((cG-x)/(cG-bG))
                FRS_Far += RightGood

    fuzzyvalues1(X1,FRS_Membership()[0][0],FRS_Membership()[0][1],FRS_Membership()[0][2],FRS_Membership()[1][0],FRS_Membership()[1][1],FRS_Membership()[1][2],FRS_Membership()[2][0],FRS_Membership()[2][1],FRS_Membership()[2][2])

    #***********************Create Function for BRS RIGHT and LEFT for fired rules****************************
    def fuzzyvalues2(x,aP,bP,cP,aA,bA,cA,aG,bG,cG):
        global BRS_Near,BRS_Middle,BRS_Far
        BRS_Near = 0
        BRS_Middle = 0
        BRS_Far = 0
        
        #IF Condition for getting right and left values of between Near and Middle
        if BRS[0] == 'between Near and Middle':
            
            if min([BRS_Membership()[0][0],BRS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == BRS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                BRS_Near += LeftPoor
                #a.append(['Near',LeftPoor])
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                BRS_Near += RightPoor
                
            if min([BRS_Membership()[1][0],BRS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == BRS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                BRS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                BRS_Middle += RightAverage
        
        #if Condition for getting right and left values of between Middle and Far
        elif BRS[0] == 'between Middle and Far':
            
            if min([BRS_Membership()[1][0],BRS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == BRS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                BRS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                BRS_Middle += RightAverage
                
            if min([BRS_Membership()[2][0],BRS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == BRS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                BRS_Far += LeftGood 
            else:
                RightGood = abs((cG-x)/(cG-bG))
                BRS_Far += RightGood
        
        #if Condition for getting right and left value of Near
        elif BRS[0] == 'Near':
            
            if min([BRS_Membership()[0][0],BRS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == BRS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                BRS_Near += LeftPoor
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                BRS_Near += RightPoor
        
        #if Condition for getting right and left value of Middle
        elif BRS[0] == 'Middle':
            
            if min([BRS_Membership()[1][0],BRS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == BRS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                BRS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                BRS_Middle += RightAverage
        
        #if Condition for getting right and left value of Far
        else:
            
            if min([BRS_Membership()[2][0],BRS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == BRS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                BRS_Far += LeftGood
            else:
                RightGood = abs((cG-x)/(cG-bG))
                BRS_Far += RightGood

    fuzzyvalues2(X2,BRS_Membership()[0][0],BRS_Membership()[0][1],BRS_Membership()[0][2],BRS_Membership()[1][0],BRS_Membership()[1][1],BRS_Membership()[1][2],BRS_Membership()[2][0],BRS_Membership()[2][1],BRS_Membership()[2][2])

    #RULES********************************RULES**********************************RULES

    #Add all Numerator and Denominator for Defuzzification to list and final sum
    Speed_Numerator = []
    Speed_Denominator = []

    Turning_Numerator = []
    Turning_Denominator = []

    #NEAR STARTS
    #FRS-NEAR & BRS-NEAR = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if FRS_Near>0 and BRS_Near>0:
        
        Speed_Denominator.append(min(FRS_Near,BRS_Near))
        Turning_Denominator.append(min(FRS_Near,BRS_Near))
        
        Speed_Numerator.append([min(FRS_Near,BRS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,BRS_Near) * Turning()[2][1]])

    #FRS-NEAR & BRS-MIDDLE = SPEED_SLOW(0.1) & TURNING_LEFT(1)    
    if FRS_Near>0 and BRS_Middle>0:
        
        Speed_Denominator.append(min(FRS_Near,BRS_Middle))
        Turning_Denominator.append(min(FRS_Near,BRS_Middle))
        
        Speed_Numerator.append([min(FRS_Near,BRS_Middle) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,BRS_Middle) * Turning()[2][1]])

    #FRS-NEAR & BRS-FAR = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if FRS_Near>0 and BRS_Far>0:
        
        Speed_Denominator.append(min(FRS_Near,BRS_Far))
        Turning_Denominator.append(min(FRS_Near,BRS_Far))
        
        Speed_Numerator.append([min(FRS_Near,BRS_Far) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,BRS_Far) * Turning()[2][1]])
    #NEAR ENDS

    #MIDDLE STARTS
    #FRS-MIDDLE & BRS-NEAR = SPEED_MEDIUM(0.3) & TURNING_RIGHT(-1)
    if FRS_Middle>0 and BRS_Near>0:
        
        Speed_Denominator.append(min(FRS_Middle,BRS_Near))
        Turning_Denominator.append(min(FRS_Middle,BRS_Near))
        
        Speed_Numerator.append([min(FRS_Middle,BRS_Near) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Middle,BRS_Near) * Turning()[0][1]])
        
    #FRS-MIDDLE & BRS-MIDDLE = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if FRS_Middle>0 and BRS_Middle>0:
        
        Speed_Denominator.append(min(FRS_Middle,BRS_Middle))
        Turning_Denominator.append(min(FRS_Middle,BRS_Middle))
        
        Speed_Numerator.append([min(FRS_Middle,BRS_Middle) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Middle,BRS_Middle) * Turning()[1][1]])
        
    #FRS-MIDDLE & BRS-FAR = SPEED_MEDIUM(0.3) & TURNING_LEFT(1)
    if FRS_Middle>0 and BRS_Far>0:
        
        Speed_Denominator.append(min(FRS_Middle,BRS_Far))
        Turning_Denominator.append(min(FRS_Middle,BRS_Far))
        
        Speed_Numerator.append([min(FRS_Middle,BRS_Far) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Middle,BRS_Far) * Turning()[2][1]])
    #MIDDLE ENDS

    #FAR STARTS
    #FRS-FAR & BRS-NEAR = SPEED_FAST(0.5) & TURNING_RIGHT(-1)
    if FRS_Far>0 and BRS_Near>0:
        
        Speed_Denominator.append(min(FRS_Far,BRS_Near))
        Turning_Denominator.append(min(FRS_Far,BRS_Near))
        
        Speed_Numerator.append([min(FRS_Far,BRS_Near) * Speed()[2][1]])
        Turning_Numerator.append([min(FRS_Far,BRS_Near) * Turning()[0][1]])

    #FRS-FAR & BRS-MIDDLE = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if FRS_Far>0 and BRS_Middle>0:
        
        Speed_Denominator.append(min(FRS_Far,BRS_Middle))
        Turning_Denominator.append(min(FRS_Far,BRS_Middle))
        
        Speed_Numerator.append([min(FRS_Far,BRS_Middle) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Far,BRS_Middle) * Turning()[0][1]])

    #FRS-FAR & BRS-FAR = SPEED_SLOW(0.1) & TURNING_NO(0)
    if FRS_Far>0 and BRS_Far>0:
        
        Speed_Denominator.append(min(FRS_Far,BRS_Far))
        Turning_Denominator.append(min(FRS_Far,BRS_Far))
        
        Speed_Numerator.append([min(FRS_Far,BRS_Far) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Far,BRS_Far) * Turning()[1][1]])
    #FRS_FAR ENDS

    #RULES END*********************************RULES END******************************RULES END

    #Converting Multiple list of lists to single list for summing later
    Speed_Numerator = sum(Speed_Numerator, [])
    Turning_Numerator = sum(Turning_Numerator, [])

    #DEFUZZIFICATION FORMULA NUmerator/Denomiantor

    global Angular_Z, Linear_X

    Linear_X = sum(Speed_Numerator)/sum(Speed_Denominator)
    Angular_Z = sum(Turning_Numerator)/sum(Turning_Denominator)
    
    print('Linear_X = ',Linear_X)
    print('Angular_Z = ',Angular_Z)
    print('FRS = ',FRS)
    print('BRS = ',BRS)

#******************************************************************************************************************************************************************************************************************
    
    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()
    
    #If an obstacle is found to be within 0.25 of the LiDAR sensors front region the linear velocity is set to 0 (turtlebot stops)
    #if (regions['front'])< 0.25:
    msg.linear.x = -Linear_X
    msg.angular.z = Angular_Z
    return msg
    #if there is no obstacle in front of the robot, it continues to move forward
    #else:
        #msg.linear.x = 0.1
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
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
