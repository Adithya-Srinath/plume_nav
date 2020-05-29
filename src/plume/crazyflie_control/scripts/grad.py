#!/usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from olfaction_msgs.msg import anemometer, gas_sensor
from tf.transformations import euler_from_quaternion
import sys
import math

# Global variables 
windspeed = 0.0
wind_dir = 0.0
conc = 0.0
x = 0.0; y =0.0; theta = 0.0 #change this to x = [] and see if you can remove sleep
vel_msg = Twist()
check = 0
initial_wind_check = 0
past_wind_speed = []
past_wind_dir = []
past_conc = []


# Function (3) to get the wind data
def wind_callbck(msg):
    global windspeed
    global wind_dir
    windspeed = msg.wind_speed
    wind_dir = msg.wind_direction
    
    #rospy.loginfo("wind speed: %s" , windspeed)
    #rospy.loginfo("wind direction: %s", wind_dir)

# Function (4) to get the gas concentration, 
def conc_callbck(msg):
    global conc
    conc = msg.raw
    #rospy.loginfo("conc: %f",conc)

# Function (5) to get the current location and speed 
def loc_callbck(msg):

    global x 
    global y
    global theta 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch,theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w]) # not sure how this works, copied from mover.py
    #rospy.loginfo("x: %f",x)
    #rospy.loginfo("y: %f",y)
    #rospy.loginfo("Theta: %f",theta)

# Function (6) to get velocity co-ordinates (used in conjuction with the function below)
def moving_controller(goal_x, goal_y):
        dist2goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
        if dist2goal > 0.05:
            angle_to_goal = math.atan2((goal_y - y), (goal_x - x))
            if abs(angle_to_goal - theta) > 0.05:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 4* angdiff(theta, angle_to_goal)
            else:
                vel_msg.linear.x = 0.7 * dist2goal
                vel_msg.angular.z = 0
            return False
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            return True
        

def angdiff(a, b):

    direction = (a-b)/abs(a-b)
    normdeg = (a-b)%(2*math.pi)
    diff = min(2*math.pi-normdeg, normdeg)
    return diff

# Function (7) to move the robot to desired co-ordinates,calls the function above (moving_controller)
def do_move(a,b):
    #rospy.on_shutdown(h)
    done = False
    
    #rospy.wait_for_message("/cmd_vel", Twist)
    r = rospy.Rate(10)
    i = 0
    goal = Point()
    goal.x = a
    goal.y = b
    print("moving to", a,b)
    while not rospy.is_shutdown():
        if not done:
            done = moving_controller(goal.x, goal.y)
        if done:
            print("Reached. Shutting down")
            break
        pub_speed.publish(vel_msg)
        #rospy.loginfo("[%f,%f], %f", x,y, theta)
        r.sleep()
    
# Function to move around surrounding area and gets concentration, windspeed and direction: 
# USED 
#   1) in beginning 
#   2) If plume is lost  
# Calculates a score based on conc, degree of downwind and distance from env boundary, based on which direction is decided.
# Returns either the direction to head in or returns false

def look_around(step_size,clueless):
    global past_wind
    global past_conc
    global check
    global avg_wind_dir

    # had to sleep first time x and y are called, otherwise x1 and y1 were input as 0. Can probably reduce sleep time
    if(check == 0):
            rospy.sleep(2) 
            print("I'm taking a nap")
            check = check +1
    x1 = x
    y1 = y 
    score = []
    x2 = []
    y2 = []
    
    # Record wind and conc where I start 
    past_wind_speed.append(windspeed)
    past_wind_dir.append(wind_dir)
    past_conc.append(conc)
    conc_counter = 0 # to check if any of the conc values are non zero 
    k =0 # counter

    if clueless == 1: 
        a1 = 0
        a2= 361
        a3 = 45
    if clueless == 0:
        a1 = int(avg_wind_dir*180/math.pi)+90
        a2 = int(avg_wind_dir*180/math.pi)-90
        a3 = 29

    # Loop to move it around, points of a circle - every 45 degrees. Collect speed, direction and conc. Get the average of dir 
    for i in range(0,361,45): # Run a loop from 0 to 360 degrees 
        ang = i/180.0*math.pi # converting to rads
        

        print(i,ang)

        # Co-ordinates to move to
        x2.append(x1+(step_size*math.cos(ang)))
        y2.append(y1+(step_size*math.sin(ang)))
        do_move(x2[k],y2[k]) 

        # appending the data to lists at each point 
        past_wind_speed.append(windspeed) 
        past_wind_dir.append(wind_dir)
        past_conc.append(conc) 
         
        # Condition to increase concentration counter if conc at any of the points is larger than 0
        if conc > 0:
            conc_counter = conc_counter+1
        rospy.sleep(1)
        k = k+1
    
    do_move(x1,y1) # going back to where it started 
    print("Past wind data:",past_wind_dir[-9:]) 
    avg_wind_dir = sum(past_wind_dir[-9:])/9 # average of the last 9 wind directions 
    print("Average:",avg_wind_dir)
    avg_wind_dir_up = math.pi + avg_wind_dir
    

    # scoring, must be done outside loop cause avg wind speed required 

    # Defining the score weights 
    a = 1.0 # Concentration score weight # started with 10 but it had a huge weight at low conc so
    b = 1.0 # angle score weight 
    # set a weight based on how close to boundary as well, this will be a varying weight 
    # set a gradient score as well- the higher the gradient with the point directly behind, the higher the score 

    for i in range (0,8): 
        my_dir_now = math.atan2((y2[i]-y1),(x2[i]-x1))
        conc_score =  a*past_conc[i-8]
        ang_score = abs(b/angdiff(avg_wind_dir_up,my_dir_now))
        print("i:",i)
        print("Conc",past_conc[i-8])
        print("conc_score", conc_score)
        print("my_dir_now", my_dir_now)
        print("ang_score",ang_score)
        print("total_score", conc_score+ang_score)
        print(" ")
        score.append(conc_score + ang_score)  # high conc = high score; closer to downwind = high score; 
    Index_max = score.index(max(score))
    print("high score is:",score[Index_max])
    print("Index is:", Index_max)
    print("x2,y2  = ",x2[Index_max],y2[Index_max])

    if conc_counter > 0: 
        head_to = 0.0 
        head_to_dir = 0.0
        head_to =  (y2[Index_max]-y1)/(x2[Index_max]-x1)
        if x2[Index_max] > x1:
            head_to_dir = 1.0
        if x2[Index_max] < x1:
            head_to_dir = -1.0
        if x2[Index_max] == x1:
            head_to_dir = 0
        print("x1,y1",x1,y1)
        print("head_to",head_to,"head to dir",head_to_dir)
        return(head_to,head_to_dir)
    else:
        print("All concentrations are 0")
        return(False, False) 

def follow_it(ang,head_to_dir_factor):
    print("Called follow_it()")
    m = ang # converting angle to slope
    conc1 = conc 
    conc2 = conc1
    while(conc2 >= conc1): # (conc1 -x), x is to prevent it from going off track for small values 
        x1 = x
        y1 = y
        print(x1,y1,wind_dir) 
        x2 = (head_to_dir_factor)*(2/(math.sqrt(m**2 + 1))) + x1 # to get the same step size every time: controlled by the numerator of the first term, CAN MAKE VARIABLE- closer to source small steps
        y2 = m*(x2-x1) + y1  # line equation 
        print(x1,y1,"going to",x2,y2,"slope",m)
        
        #Averaging out the conc cause of fluctuations, can remove if required once we fix plume 
        conc1 =0
        for i in range (4):
            conc1 = conc1+conc
            print(i,"conc1",conc)
            rospy.sleep(2)
        conc1 = conc1/5

        # move to the next point 
        do_move(x2,y2)

        # Average out conc at that point as well
        conc2 =0
        for i in range (4):
            conc2 = conc2+conc
            print(i,"conc2",conc)
            rospy.sleep(2)
        conc2 = conc2/5
    print("conc started decreasing")
    return (False)

#check if going up the line or going down is longer, take the longer path
def raster_find_longer(x1,y1,m):
    print("Called raster_find_longer")
    x_int_0=(0.0-y1+m*x1)/m
    y_int_0= m*(0.0-x1)+m
    x_int_20=(20.0-y1+m*x1)/m
    y_int_20= m*(20.0-x1)+m

    intercept = [x_int_0, y_int_0, x_int_20, y_int_20]
    conj = [0.0,0.0,20.0,20.0]

    d1 = math.sqrt((x1 - x_int_0)**2 + (y1 - 0)**2)
    d2 = math.sqrt((x1 - 0)**2 + (y1 - y_int_0)**2)
    d3 = math.sqrt((x1 - x_int_20)**2 + (y1 - 20)**2)
    d4 = math.sqrt((x1 - 20)**2 + (y1 - y_int_20)**2)

    if(x_int_0 < 0 or x_int_0 > 20):
        d1 =0
    if(y_int_0 < 0 or y_int_0 > 20):
        d2=0
    if(x_int_20 < 0 or x_int_20 > 20):
        d3 =0
    if(y_int_20 < 0 or y_int_20 > 20):
        d4=0
    
    d = [d1,d2,d3,d4]
    Index_max = d.index(max(d))
    if Index_max == 0 or Index_max == 2:
        x2 = intercept[Index_max]
        if x2 > x1:
            return(1.0)
        else:
            return(-1.0) 
    else:
        x2 = conj[Index_max]
        if x2 > x1:
            return(1.0)
        else:
            return(-1.0)           




    


# Function to search for the plume initially if find_data returns false ( all conc are 0) 
# modify it so that it checks which side is longer 
# let it travel that whole distance 
# come back 
# if still unsucessful - step up by 5 units and try again 
def raster(): 
    print("Called raster")
    global check
    rospy.sleep(2)
    avg_wind_dir = sum(past_wind_dir[-9:])/9
    m = math.tan((math.pi/2)+avg_wind_dir) # slope : Perpendicular to wind direction. wind_dir gives +ve values moving from +x to +y
    if( m > 1000): # cause tan(90) is undefined & near values go crazy high
        m = 1000
    if(check == 0):
        rospy.sleep(1) # had to sleep, otherwise x1 and y1 were input as 0
        check = check +1  
    x1 = x
    y1 = y
    xinit = x1
    yinit = y1
    print(x1,y1,wind_dir)
    hit_bound = 0 
    dir_factor = raster_find_longer(x1,y1,m)
    
    while(conc < 0.1):
        x2 = dir_factor*(3/(math.sqrt(m**2 + 1))) + x1 # to get the same step size every time: controlled by the numerator of the first term
        y2 = m*(x2-x1) + y1  # line equation 
        print(x1,y1,x2,y2,m)

        if (((x2 > 20 or y2 > 20) or (x2 <0 or y2<0))and hit_bound < 1) : # Hit the boundary for the first time
            hit_bound = hit_bound+1 # inc this, so I know when it hits again
            do_move(xinit,yinit) # move it back to initial position
            dir_factor = -1.0*dir_factor # Move it in the opp direction from before 
            x1 = x # let it know where it is now 
            y1 = y
            continue

        if (((x2 > 20 or y2 > 20)or (x2 <0 or y2<0)) and hit_bound == 1 ): # If it hits the boundary again, go upwind and try again
            dir_factor2 = raster_find_longer(x1,y1,(-1/m)) # find the longer of the two perpendicular paths 
            xup = dir_factor2*(5/(math.sqrt((-1/m)**2 + 1))) + x1 # move upwind 5 units 
            yup = (-1/m)*(x2-x1) + y1 
            do_move(xup,yup)
            dir_factor = -1.0*dir_factor # so that it moves in the opp direction from before
            x1 = x # let it know where it is now 
            y1 = y  
            continue 

        do_move(x2,y2)
        x1 = x
        y1 = y
    print("Found gas with concentration of:",conc)
    return(True)
        
# Function to identify the direction of increasing concentration 
#def which_way():
#    if conc 0: 


    

#if __name__ == "__main__":
#    read_topics()
#    print("--------------------------------")
#    time.sleep(5)
#else: 
#    print("ERROR ######################")


# Function (1) to subscribe to all required topics wind, PID, current location and speed 

rospy.init_node("grad", anonymous=True) # initialize the node 
rospy.Subscriber("Anemometer/WindSensor_reading", anemometer, wind_callbck) # Subscribe to Anemometer - get wind data
rospy.Subscriber("PID/Sensor_reading", gas_sensor, conc_callbck) # Subscribe to PID data 
rospy.Subscriber("/base_pose_ground_truth", Odometry, loc_callbck) # Current position and speed 

# Function (2) to publish to nodes - cmd_vel
pub_speed = rospy.Publisher("/cmd_vel", Twist, queue_size=10)



if __name__ == "__main__":
    b= 1 # clueless is true
    while(conc<40):
        a,c= look_around(1,b) # step size is 1 and we are clueless when b is 1 ( a is the slope (or) False, c is the direction (up or down slope))
        print(a)
        if a is not False:
            b= 0 # clueless is false 
            while(follow_it(a,c) != False):
                pass
            continue
        else:
            print("calling Raster")
            while(raster() != True):
                pass
            continue
else: 
    
    print("Error")


