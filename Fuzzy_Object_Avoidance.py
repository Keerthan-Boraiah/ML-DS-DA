
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

#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    print("Min distance in left front region: ", regions['fleft'])
    print("Min distance in front region: ", regions['front'])
    print("Min distance in right front region: ", regions['fright'])
    
#*********************************************************************************************************************************************************************************************

    global X1,X2,X3,LinearX,AngularZ
    X1 = regions['fright']
    X2 = regions['fleft']
    X3 = regions['front']

    FRS = []
    FLS = []
    FS = []

    #Create Function to Define FRONT RIGHT SENSOR
    def FRS_Membership():
        Near = [0,0.15,0.33]
        Middle = [0.15,0.33,0.66]
        Far = [0.33,0.66,1.0001]        
        return Near,Middle,Far

    #Create Function to Define FRONT LEFT SENSOR
    def FLS_Membership():
        Near = [0,0.15,0.33]
        Middle = [0.15,0.33,0.66]
        Far = [0.33,0.66,1.0001]         
        return Near,Middle,Far

    #Create Function to Define FRONT SENSOR
    def FS_Membership():
        Near = [0,0.15,0.33]
        Middle = [0.15,0.33,0.66]
        Far = [0.33,0.66,1.0001]        
        return Near,Middle,Far

    #Create Function to Define Speed
    def Speed():
        Slow = [0,0.01,0.020001]
        Medium = [0.02,0.03,0.040001]
        Fast = [0.04,0.05,0.060001]
        return Slow,Medium,Fast

    #Create Function to Define Turning
    def Turning():
        Right = [-1.5,-1.2,-0.5]
        No = [-0.5, 0, 0.5]
        Left = [0.5, 1.2, 1.5]
        return Right,No,Left

    def memberships(x,y,z):
        
        #***********************FRS Right or left or Both Checker of FIRING RULES*****************************
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
            
        #***********************FLS Right or left or Both Checker of FIRING RULES****************************
        if ((y >=FLS_Membership()[0][0] and y<= FLS_Membership()[0][-1]) and (y >=FLS_Membership()[1][0] and y<= FLS_Membership()[1][-1])):
            FLS.append('between Near and Middle')
        elif ((y >=FLS_Membership()[1][0] and y<= FLS_Membership()[1][-1]) and (y >=FLS_Membership()[2][0] and y<= FLS_Membership()[2][-1])):
            FLS.append('between Middle and Far')
        elif (y >=FLS_Membership()[0][0] and y<= FLS_Membership()[0][-1]):
            FLS.append('Near')
        elif (y >=FLS_Membership()[1][0] and y<= FLS_Membership()[1][-1]):
            FLS.append('Middle')
        else:
            FLS.append('Far')
            
        #***********************FS Right or left or Both Checker of FIRING RULES*****************************
        if ((z >=FS_Membership()[0][0] and z<= FS_Membership()[0][-1]) and (z >=FS_Membership()[1][0] and z<= FS_Membership()[1][-1])):
            FS.append('between Near and Middle')
        elif ((z >=FS_Membership()[1][0] and z<= FS_Membership()[1][-1]) and (z >=FS_Membership()[2][0] and z<= FS_Membership()[2][-1])):
            FS.append('between Middle and Far')
        elif (z >=FS_Membership()[0][0] and z<= FS_Membership()[0][-1]):
            FS.append('Near')
        elif (z >=FS_Membership()[1][0] and z<= FS_Membership()[1][-1]):
            FS.append('Middle')
        else:
            FS.append('Far')
            
    memberships(X1,X2,X3)
    print('FRS:- ',FRS)
    print('FLS:- ',FLS)
    print('FS:- ',FS)

    #************************Create Function for FRS RIGHT and LEFT for fired rules************************* 
    def fuzzyvalues1(x,aP,bP,cP,aA,bA,cA,aG,bG,cG):
        global FRS_Near,FRS_Middle,FRS_Far
        FRS_Near = 0
        FRS_Middle = 0
        FRS_Far = 0
        
        #IF Condition for getting right and left values of between Near and Middle
        if FRS[0] == 'between Near and Middle':
            
            if min([FRS_Membership()[0][0],FRS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                FRS_Near += LeftPoor
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                FRS_Near += RightPoor
                
            if min([FRS_Membership()[1][0],FRS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FRS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FRS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FRS_Middle += RightAverage
        
        #if Condition for getting right and left values of between Middle and Far
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
    
    print('FRS_Near:- ',FRS_Near)
    print('FRS_Middle:- ',FRS_Middle)
    print('FRS_Far:- ',FRS_Far)

    #***********************Create Function for FLS RIGHT and LEFT for fired rules*************************
    def fuzzyvalues2(x,aP,bP,cP,aA,bA,cA,aG,bG,cG):
        global FLS_Near,FLS_Middle,FLS_Far
        FLS_Near = 0
        FLS_Middle = 0
        FLS_Far = 0
        
        #IF Condition for getting right and left values of between Near and Middle
        if FLS[0] == 'between Near and Middle':
            
            if min([FLS_Membership()[0][0],FLS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == FLS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                FLS_Near += LeftPoor
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                FLS_Near += RightPoor
                
            if min([FLS_Membership()[1][0],FLS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FLS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FLS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FLS_Middle += RightAverage
        
        #if Condition for getting right and left values of between Middle and Far
        elif FLS[0] == 'between Middle and Far':
            
            if min([FLS_Membership()[1][0],FLS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FLS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FLS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FLS_Middle += RightAverage
                
            if min([FLS_Membership()[2][0],FLS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == FLS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                FLS_Far += LeftGood 
            else:
                RightGood = abs((cG-x)/(cG-bG))
                FLS_Far += RightGood
        
        #if Condition for getting right and left value of Near
        elif FLS[0] == 'Near':
            
            if min([FLS_Membership()[0][0],FLS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == FLS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                FLS_Near += LeftPoor
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                FLS_Near += RightPoor
        
        #if Condition for getting right and left value of Middle
        elif FLS[0] == 'Middle':
            
            if min([FLS_Membership()[1][0],FLS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FLS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FLS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FLS_Middle += RightAverage
        
        #if Condition for getting right and left value of Far
        else:
            
            if min([FLS_Membership()[2][0],FLS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == FLS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                FLS_Far += LeftGood
            else:
                RightGood = abs((cG-x)/(cG-bG))
                FLS_Far += RightGood

    fuzzyvalues2(X2,FLS_Membership()[0][0],FLS_Membership()[0][1],FLS_Membership()[0][2],FLS_Membership()[1][0],FLS_Membership()[1][1],FLS_Membership()[1][2],FLS_Membership()[2][0],FLS_Membership()[2][1],FLS_Membership()[2][2])
    
    print('FLS_Near:- ',FLS_Near)
    print('FLS_Middle:- ',FLS_Middle)
    print('FLS_Far:- ',FLS_Far)

    #*************************Create Function for FS RIGHT and LEFT for fired rules***************************
    def fuzzyvalues3(x,aP,bP,cP,aA,bA,cA,aG,bG,cG):
        global FS_Near,FS_Middle,FS_Far
        FS_Near = 0
        FS_Middle = 0
        FS_Far = 0
        
        #IF Condition for getting right and left values of between Near and Middle
        if FS[0] == 'between Near and Middle':
            
            if min([FS_Membership()[0][0],FS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == FS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                FS_Near += LeftPoor
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                FS_Near += RightPoor
                
            if min([FS_Membership()[1][0],FS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FS_Middle += RightAverage
        
        #if Condition for getting right and left values of between Middle and Far
        elif FS[0] == 'between Middle and Far':
            
            if min([FS_Membership()[1][0],FS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FS_Middle += RightAverage
                
            if min([FS_Membership()[2][0],FS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == FS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                FS_Far += LeftGood 
            else:
                RightGood = abs((cG-x)/(cG-bG))
                FS_Far += RightGood
        
        #if Condition for getting right and left value of Near
        elif FS[0] == 'Near':
            
            if min([FS_Membership()[0][0],FS_Membership()[0][-1]], key=lambda x:abs(x-X1)) == FS_Membership()[0][0]:
                LeftPoor = abs((x-aP)/(bP-aP))
                FS_Near += LeftPoor
            else:
                RightPoor = abs((cP-x)/(cP-bP))
                FS_Near += RightPoor
        
        #if Condition for getting right and left value of Middle
        elif FS[0] == 'Middle':
            
            if min([FS_Membership()[1][0],FS_Membership()[1][-1]], key=lambda x:abs(x-X1)) == FS_Membership()[1][0]:
                LeftAverage = abs((x-aA)/(bA-aA))
                FS_Middle += LeftAverage
            else:
                RightAverage = abs((cA-x)/(cA-bA))
                FS_Middle += RightAverage
        
        #if Condition for getting right and left value of Far
        else:
            
            if min([FS_Membership()[2][0],FS_Membership()[2][-1]], key=lambda x:abs(x-X1)) == FS_Membership()[2][0]:
                LeftGood = abs((x-aG)/(bG-aG))
                FS_Far += LeftGood
            else:
                RightGood = abs((cG-x)/(cG-bG))
                FS_Far += RightGood

    fuzzyvalues3(X3,FS_Membership()[0][0],FS_Membership()[0][1],FS_Membership()[0][2],FS_Membership()[1][0],FS_Membership()[1][1],FS_Membership()[1][2],FS_Membership()[2][0],FS_Membership()[2][1],FS_Membership()[2][2])
    #print(FS_Near)
    
    print('FS_Near:- ',FS_Near)
    print('FS_Middle:- ',FS_Middle)
    print('FS_Far:- ',FS_Far)

    #RULES********************************RULES**********************************RULES

    #Add all Numerator and Denominator for Defuzzification to list and final sum
    Speed_Numerator = []
    Speed_Denominator = []

    Turning_Numerator = []
    Turning_Denominator = []

    #NEAR STARTS
    #1st FRS-NEAR & FLS-NEAR & FS-NEAR = SPEED_SLOW(0.1) & TURNING_RIGHT(1)
    if (FRS_Near>0 and FLS_Near>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Near,FS_Near))
        Turning_Denominator.append(min(FRS_Near,FLS_Near,FS_Near))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Near,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Near,FS_Near) * Turning()[0][1]])

    #2nd FRS-NEAR & FLS-NEAR & FS-MEDIUM = SPEED_SLOW(0.1) & TURNING_NO(0)    
    if (FRS_Near>0 and FLS_Near>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Near,FS_Middle))
        Turning_Denominator.append(min(FRS_Near,FLS_Near,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Near,FS_Middle) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Near,FS_Middle) * Turning()[1][1]])

    #3rd FRS-NEAR & FLS-Near & FS_Far = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Near>0 and FLS_Near>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Near,FS_Far))
        Turning_Denominator.append(min(FRS_Near,FLS_Near,FS_Far))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Near,FS_Far) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Near,FS_Far) * Turning()[1][1]])
        
    #4th FRS_NEAR & FLS_MEDIUM & FS_NEAR = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if (FRS_Near>0 and FLS_Middle>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Middle,FS_Near))
        Turning_Denominator.append(min(FRS_Near,FLS_Middle,FS_Near))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Middle,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Middle,FS_Near) * Turning()[2][1]])
        
    #5th FRS_NEAR & FLS_MEDIUM & FS_MEDIUM = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if (FRS_Near>0 and FLS_Middle>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Middle,FS_Middle))
        Turning_Denominator.append(min(FRS_Near,FLS_Middle,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Middle,FS_Middle) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Middle,FS_Middle) * Turning()[2][1]])

    #6th FRS_NEAR & FLS_MEDIUM & FS_FAR = SPEED_SLOW(0.3) & TURNING_LEFT(0)
    if (FRS_Near>0 and FLS_Middle>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Middle,FS_Far))
        Turning_Denominator.append(min(FRS_Near,FLS_Middle,FS_Far))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Middle,FS_Far) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Middle,FS_Far) * Turning()[2][1]])

    #7th FRS_NEAR & FLS_FAR & FS_NEAR = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if (FRS_Near>0 and FLS_Far>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Far,FS_Near))
        Turning_Denominator.append(min(FRS_Near,FLS_Far,FS_Near))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Far,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Far,FS_Near) * Turning()[2][1]])
        
    #8th FRS_NEAR & FLS_FAR & FS_MEDIUM = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if (FRS_Near>0 and FLS_Far>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Far,FS_Middle))
        Turning_Denominator.append(min(FRS_Near,FLS_Far,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Far,FS_Middle) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Far,FS_Middle) * Turning()[2][1]])
        
    #9th FRS_NEAR &FLS_FAR & FS_FAR = SPEED_SLOW(0.3) & TURNING_LEFT(1)
    if (FRS_Near>0 and FLS_Far>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Near,FLS_Far,FS_Far))
        Turning_Denominator.append(min(FRS_Near,FLS_Far,FS_Far))
        
        Speed_Numerator.append([min(FRS_Near,FLS_Far,FS_Far) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Near,FLS_Far,FS_Far) * Turning()[2][1]])
    #NEAR ENDS
    
    #MIDDLE STARTS
    #10th FRS_Middle &FLS_Near & FS_Near = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if (FRS_Middle>0 and FLS_Near>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Near,FS_Near))
        Turning_Denominator.append(min(FRS_Middle,FLS_Near,FS_Near))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Near,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Near,FS_Near) * Turning()[0][1]])
        
    #11th FRS_Middle &FLS_Near & FS_MIDDLE = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if (FRS_Middle>0 and FLS_Near>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Near,FS_Middle))
        Turning_Denominator.append(min(FRS_Middle,FLS_Near,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Near,FS_Middle) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Near,FS_Middle) * Turning()[0][1]])
        
    #12th FRS_Middle &FLS_Near & FS_FAR = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if (FRS_Middle>0 and FLS_Near>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Near,FS_Far))
        Turning_Denominator.append(min(FRS_Middle,FLS_Near,FS_Far))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Near,FS_Far) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Near,FS_Far) * Turning()[0][1]])

    #13th FRS_Middle &FLS_Middle & FS_Near = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if (FRS_Middle>0 and FLS_Middle>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Middle,FS_Near))
        Turning_Denominator.append(min(FRS_Middle,FLS_Middle,FS_Near))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Middle,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Middle,FS_Near) * Turning()[2][1]])
        
    #14th FRS_Middle &FLS_MIDDLE & FS_MIDDLE = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Middle>0 and FLS_Middle>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Middle,FS_Middle))
        Turning_Denominator.append(min(FRS_Middle,FLS_Middle,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Middle,FS_Middle) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Middle,FS_Middle) * Turning()[1][1]])
        
    #15th FRS_Middle &FLS_MIDDLE & FS_FAR = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Middle>0 and FLS_Middle>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Middle,FS_Far))
        Turning_Denominator.append(min(FRS_Middle,FLS_Middle,FS_Far))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Middle,FS_Far) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Middle,FS_Far) * Turning()[1][1]])
        
    #16th FRS_Middle &FLS_FAR & FS_Near = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if (FRS_Middle>0 and FLS_Far>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Far,FS_Near))
        Turning_Denominator.append(min(FRS_Middle,FLS_Far,FS_Near))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Far,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Far,FS_Near) * Turning()[2][1]])
        
    #17th FRS_Middle &FLS_FAR & FS_MIDDLE = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Middle>0 and FLS_Far>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Far,FS_Middle))
        Turning_Denominator.append(min(FRS_Middle,FLS_Far,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Far,FS_Middle) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Far,FS_Middle) * Turning()[1][1]])
        
    #18th FRS_Middle &FLS_FAR & FS_FAR = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Middle>0 and FLS_Far>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Middle,FLS_Far,FS_Far))
        Turning_Denominator.append(min(FRS_Middle,FLS_Far,FS_Far))
        
        Speed_Numerator.append([min(FRS_Middle,FLS_Far,FS_Far) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Middle,FLS_Far,FS_Far) * Turning()[1][1]])
    #MIDDLE ENDS

    #FAR STARTS
    #19th FRS_FAR &FLS_Near & FS_Near = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if (FRS_Far>0 and FLS_Near>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Near,FS_Near))
        Turning_Denominator.append(min(FRS_Far,FLS_Near,FS_Near))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Near,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Near,FS_Near) * Turning()[0][1]])
        
    #20th FRS_FAR &FLS_Near & FS_MIDDLE = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if (FRS_Far>0 and FLS_Near>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Near,FS_Middle))
        Turning_Denominator.append(min(FRS_Far,FLS_Near,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Near,FS_Middle) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Near,FS_Middle) * Turning()[0][1]])
        
    #21th FRS_FAR &FLS_Near & FS_FAR = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if (FRS_Far>0 and FLS_Near>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Near,FS_Far))
        Turning_Denominator.append(min(FRS_Far,FLS_Near,FS_Far))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Near,FS_Far) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Near,FS_Far) * Turning()[0][1]])

    #22th FRS_FAR &FLS_Middle & FS_Near = SPEED_SLOW(0.1) & TURNING_RIGHT(-1)
    if (FRS_Far>0 and FLS_Middle>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Middle,FS_Near))
        Turning_Denominator.append(min(FRS_Far,FLS_Middle,FS_Near))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Middle,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Middle,FS_Near) * Turning()[0][1]])
        
    #23th FRS_FAR &FLS_MIDDLE & FS_MIDDLE = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Far>0 and FLS_Middle>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Middle,FS_Middle))
        Turning_Denominator.append(min(FRS_Far,FLS_Middle,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Middle,FS_Middle) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Middle,FS_Middle) * Turning()[1][1]])
        
    #24th FRS_FAR &FLS_MIDDLE & FS_FAR = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Far>0 and FLS_Middle>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Middle,FS_Far))
        Turning_Denominator.append(min(FRS_Far,FLS_Middle,FS_Far))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Middle,FS_Far) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Middle,FS_Far) * Turning()[1][1]])
        
    #25th FRS_FAR &FLS_FAR & FS_Near = SPEED_SLOW(0.1) & TURNING_LEFT(1)
    if (FRS_Far>0 and FLS_Far>0 and FS_Near>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Far,FS_Near))
        Turning_Denominator.append(min(FRS_Far,FLS_Far,FS_Near))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Far,FS_Near) * Speed()[0][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Far,FS_Near) * Turning()[2][1]])
        
    #26th FRS_FAR &FLS_FAR & FS_MIDDLE = SPEED_MEDIUM(0.3) & TURNING_NO(0)
    if (FRS_Far>0 and FLS_Far>0 and FS_Middle>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Far,FS_Middle))
        Turning_Denominator.append(min(FRS_Far,FLS_Far,FS_Middle))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Far,FS_Middle) * Speed()[1][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Far,FS_Middle) * Turning()[1][1]])
        
    #27th FRS_FAR &FLS_FAR & FS_FAR = SPEED_FAR(0.5) & TURNING_NO(0)
    if (FRS_Far>0 and FLS_Far>0 and FS_Far>0):
        
        Speed_Denominator.append(min(FRS_Far,FLS_Far,FS_Far))
        Turning_Denominator.append(min(FRS_Far,FLS_Far,FS_Far))
        
        Speed_Numerator.append([min(FRS_Far,FLS_Far,FS_Far) * Speed()[2][1]])
        Turning_Numerator.append([min(FRS_Far,FLS_Far,FS_Far) * Turning()[1][1]])
    #FAR ENDS
    #RULES END*********************************RULES END******************************RULES END

    #Converting Multiple list of lists to single list for summing later
    Speed_Numerator = sum(Speed_Numerator, [])
    Turning_Numerator = sum(Turning_Numerator, [])

    #DEFUZZIFICATION FORMULA NUmerator/Denomiantor
    
    print('Speed_Numerator:- ',sum(Speed_Numerator), ' Speed_Denominator:- ',sum(Speed_Denominator))
    print('Turning_Numerator:- ',sum(Turning_Numerator), ' Turning_Denominator:- ',sum(Turning_Denominator))
	
    #print(sum(Speed_Denominator))
    Linear_X = sum(Speed_Numerator)/sum(Speed_Denominator)
    Angular_Z = sum(Turning_Numerator)/sum(Turning_Denominator)
    
    print('Linear_X:- ',Linear_X)
    print('Angular_Z:- ',Angular_Z)
    print('***********************************************************************')


#*********************************************************************************************************************************************************************************************
    
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