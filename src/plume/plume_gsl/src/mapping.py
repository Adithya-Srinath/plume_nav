#! /usr/bin/env python

# Current issues
# 
# The drone does not have a wind history prior to the mission start
# This means that Vx and Vy grow until t_max is reached 
# It should be okay though
#
# t_max depends on average wind speed. Right now t_max is eyeballed
#
# 

import rospy
import numpy as np
from gaden_player.srv import GasPosition
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from gridworld import GridWorld
import math
import tf
from tf.transformations import euler_from_quaternion
from olfaction_msgs.msg import anemometer, gas_sensor
import warnings
from std_msgs.msg import Float32

warnings.filterwarnings("error", "divide", RuntimeWarning)


class Prob_Mapping:

    def __init__(self):
        rospy.init_node("mapping")
        self.x = 0 # x position
        self.y = 0 # y position
        self.listener = tf.TransformListener()
        self.wind_history = [] # An array that stores the history of anemometer readings
        self.t_max = rospy.Duration(15) # Approx the max time it takes for particle to move across the space
        self.K = -2 # (upper) pointer to the wind data for current time
        self.L = None # (lower) pointer to the wind data furthest way from current time but within self.t_max 

        
        # params
        self.fixed_frame  = rospy.get_param("~fixed_frame")
        self.anemo_frame  = rospy.get_param("~anemometer_frame")
        self.sensor_frame = rospy.get_param("~sensor_frame")
        self.verbose      = rospy.get_param("~verbose") 
        self.use_service_for_gas = rospy.get_param("~use_service_for_gas")
        
        self.go() # Start mapping


    def pos_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def normalizing_angle(self, angle):

        while angle <= -math.pi:
            angle += 2*math.pi

        while angle > math.pi:
            angle -= 2*math.pi

        return angle


    def wind_callback(self, msg):

        if self.K > -2:
            
            self.listener.waitForTransform(self.anemo_frame,self.fixed_frame, rospy.Time(), rospy.Duration(5.0))
            
            # Orientation of sensor with respect to world frame
            trans,rot = self.listener.lookupTransform(self.fixed_frame, self.anemo_frame, rospy.Time(0))
            roll,pitch,yaw = euler_from_quaternion(rot)
            wind_dir = yaw + msg.wind_direction - math.pi

            # Normalizing angle
            wind_dir = self.normalizing_angle(wind_dir)

            # Instantaneous wind with current time
            wind_inst = [msg.wind_speed*math.cos(wind_dir),\
                                msg.wind_speed*math.sin(wind_dir),\
                                    rospy.Time.now()]
            
            self.wind_history.append(wind_inst)
            self.K += 1 # Update pointer            

            if self.L is None: # This is when wind starts recording
                self.L = 0

            if self.verbose:
                rospy.loginfo("Wind_x = {}".format(wind_inst[0]))
            # rospy.loginfo("Wind_direction = {}".format(wind_dir))

        
    def sensor_callback(self, msg):
        self.gas_conc = msg.raw


    # def find_index(self, x, y, res, m):
    #     # Currently this works only if the lower bound of grid.xlims and grid.ylims are 0

    #     This function is required only when gamma and beta are treated as matrices

    #     if x/res != int(x/res): # if x is not divisible by res
    #         xbar = int(x/res)
    #         if xbar >= res/2:
    #             x = xbar*res + res
    #         else:
    #             x = xbar*res
            
    #     if y/res != int(y/res):
    #         ybar = int(y/res)
    #         if ybar >= res/2:
    #             y = ybar*res + res
    #         else:
    #             y = ybar*res

    #     a = int(x/res)
    #     b = int(y/res)
    #     ind = a + b*m
    #     return ind       

        
    def go(self):
        
        # Subscribers to know the position
        pos_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.pos_callback)
        anemo_sub = rospy.Subscriber("/Anemometer/WindSensor_reading", anemometer, self.wind_callback)
        rospy.wait_for_message("/Anemometer/WindSensor_reading", anemometer, rospy.Duration(5.0))

        # Publisher
        prob_pub = rospy.Publisher("mapping_viz", OccupancyGrid, queue_size=10)
        conc_pub = rospy.Publisher("concentration", Float32, queue_size=10)
        
        if self.use_service_for_gas:
            rospy.wait_for_service("odor_value")
            odor_req = rospy.ServiceProxy('odor_value', GasPosition)
        else:
            sensor_sub = rospy.Subscriber("/PID/Sensor_reading", gas_sensor, self.sensor_callback)       
        
        grid = GridWorld()

        # Algorithm specific parameters
        # Initializing matrices
        alpha   = (1.0/grid.M) * np.ones(grid.M)
        Sij     = np.zeros(grid.M)
        beta    = np.zeros(grid.M)
        gamma   = (1.0/grid.M) * np.ones(grid.M)

        # OccupancyGrid
        prob = OccupancyGrid()
        prob.info.height = grid.m
        prob.info.width = grid.n
        prob.info.resolution = grid.res
        prob.info.origin.position.x = 0
        prob.info.origin.position.y = 0
        prob.info.origin.position.z = 0
        prob.info.map_load_time = rospy.Time.now()
        prob.header.frame_id = self.fixed_frame
        prob.data = (alpha*100).astype(np.int8).tolist()

        r = rospy.Rate(2) # Might have to be changed later

        self.start_time = rospy.Time.now()
        self.K = -1
        flag = 0
        while not rospy.is_shutdown():
            if self.L is None:
                continue
            
            # Creating local saves at each timestep for continuously changing values
            x_pos,y_pos = self.x, self.y
            K = self.K

            # Find index of points in the prob grid map
            # index = self.find_index(x_pos, y_pos, grid.res, grid.m)

            # Read chemical concentration
            if self.use_service_for_gas:
                try:
                    odor_res = odor_req(x_pos, y_pos, grid.height)
                    gas_conc = odor_res.gas_conc[0]
                except rospy.ServiceException, e:
                    rospy.logerr("[mapping.py] Odor service call failed %s"%e)
            
            else:
                gas_conc = self.gas_conc

            rospy.loginfo("x,y = [%.2f,%.2f], Gas conc: %.2f",x_pos,y_pos,gas_conc)

            # Check if detection occurs     
            detection = gas_conc > 0      

            ####### Adjust lower pointer to not exceed t_max ############
            curr_time = rospy.Time.now()

            wind_data = self.wind_history[self.L:]               

            if (curr_time - wind_data[0][-1]) > self.t_max:
                for windinstance in wind_data:
                    if curr_time - windinstance[-1] > self.t_max:
                        self.L += 1
                    else:
                        break
                    if windinstance == wind_data[-1]:
                        rospy.logerr("Did not break loop")

            ##############################################################
            
            # Wind values from t_L to t_K without accounting for the 'time' columnof wind_history
            wind_data = np.delete(self.wind_history[self.L:K+1],2,1).astype(float)
            Vx,Vy = np.sum(wind_data,0) 

            beta[:] = 0
            gamma[:] = 1
            flag = 0
            for t0 in range(self.L, K):
     
                tl,tk = self.wind_history[t0][2].to_sec(), self.wind_history[K][2].to_sec()
                deviation_x = math.sqrt(tk-tl) * grid.sx
                deviation_y = math.sqrt(tk-tl) * grid.sy

                for i in range(0, grid.M):
                    
                    deltax = x_pos - grid.xcell[i] - Vx
                    deltay = y_pos - grid.ycell[i]- Vy

                    Sij[i] = grid.res**2 * np.exp((-deltax**2)/(2*deviation_x**2)) * \
                                np.exp((-deltay**2)/(2*deviation_y**2)) /\
                                     (2*np.pi*deviation_x*deviation_y)
                    
                    try:
                        Sij /= np.sum(Sij)
                    except RuntimeWarning:
                        rospy.logerr("All values of Sij = 0. sx and/or sy has to be changed")
                    
                    if detection:
                        beta = beta + Sij
                    else:
                        gamma = gamma * (1 - grid.mu*Sij)
                    
                    # Debugger
                    # Values can be changed here
                    # if flag == 0 and i == 2600 and abs(y_pos - 10) < 0.1:
                    #     flag = 1
                    #     print("Vx: ", Vx)
                    #     print("Vy: ", Vy)
                    #     print("deviation_x: ", deviation_x)
                    #     print("deviation_y: ", deviation_y)
                    #     print("tk: ", tk)
                    #     print("tl: ", tl)
                    #     print("deltax: ", deltax)
                    #     print("deltay: ", deltay)
                    #     print("Sij[i]: ", Sij[i])
            
            if self.L != K:
                if detection:
                    beta /= (K-self.L)
                    alpha_k = grid.M * beta * alpha
                else:
                    alpha_k = (grid.M/np.sum(gamma)) * gamma * alpha
                
                alpha_k = alpha_k/np.sum(alpha_k)
                alpha = alpha_k

                prob.data = (alpha*1000).astype(np.int8).tolist()
                prob.data = [100 if x > 100 else x for x in prob.data]

            # rospy.loginfo("Max prob value: %d"%np.max(prob.data))
            # print("[INFO]: Max concentration: {}\n[INFO]: Value at index: {}".format(np.max(alpha),np.where(alpha == np.max(alpha))))

            if self.verbose:
                if self.L != 0:
                    rospy.loginfo("self.L = %d",self.L)
                # rospy.loginfo("time interval %f",(curr_time-wind_data[0][-1]).to_sec())
            
            prob_pub.publish(prob)
            conc_pub.publish(gas_conc)
            
            r.sleep()


if __name__ == "__main__":
    try:               
        Prob_Mapping()
    except rospy.ROSInterruptException:
        rospy.logerr("Unable to start mapping node")