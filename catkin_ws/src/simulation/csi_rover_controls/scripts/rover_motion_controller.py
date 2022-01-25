#!/usr/bin/env python
import sys          # for sys.float_info.min 
import time
import math         # for trig functions
import rospy
import numpy as np  # for dot products and normals
#import collections
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry    # for tests

class RoverMotionController():
    AVOID_0_DIVISION = sys.float_info.min # AVOID_0_DIVISION = 2.2250738585072014e-308
    def __init__(self):
	self.GAZEBO_SPEED_CONSTANT = 5.53184503827	
	self.SPEED_CONSTANT = 5.53184503827 # is ok at pi/3  radius at 6.5 ----> 4.15     #5#.53184503827 for pi/4?    # MAYBE SLOWER FOR PI/6 because now it undershoots it	
	self.MAX_SPEED = 1.5

        self.name_space = rospy.get_param("name_space", "csi_rover")
        self.wheel_separation_length = rospy.get_param("wheel_separation_length", 1.5748)
        self.wheel_separation_width = rospy.get_param("wheel_separation_width", 1.87325)
        self.wheel_radius = 0.275#rospy.get_param("wheel_radius", 0.275)

        self.lf_steering_pub = rospy.Publisher("/csi_rover/left_front_steering_controller/command", Float64,
                                               queue_size=5)
        self.rf_steering_pub = rospy.Publisher("/csi_rover/right_front_steering_controller/command", Float64,
                                               queue_size=5)
        self.lr_steering_pub = rospy.Publisher("/csi_rover/left_rear_steering_controller/command", Float64,
                                               queue_size=5)
        self.rr_steering_pub = rospy.Publisher("/csi_rover/right_rear_steering_controller/command", Float64,
                                               queue_size=5)

        self.lf_axle_pub = rospy.Publisher("/csi_rover/left_front_axle_controller/command", Float64, queue_size=5)
        self.rf_axle_pub = rospy.Publisher("/csi_rover/right_front_axle_controller/command", Float64, queue_size=5)
        self.lr_axle_pub = rospy.Publisher("/csi_rover/left_rear_axle_controller/command", Float64, queue_size=5)
        self.rr_axle_pub = rospy.Publisher("/csi_rover/right_rear_axle_controller/command", Float64, queue_size=5)

	#self.pi_12 = 0.26179938779914974 ######## for reference only                   ?????????????????????????????????????????????????????????????????????????
	#self.pi_24 = 0.13089969389957495 ######## for reference only                   ?????????????????????????????????????????????????????????????????????????

	self.yaw_prev = 0
	self.yaw_start = 0
	self.yaw_stop = 0

	#all vectors as numpy arrays
	self.vector_prev = [0,0]
	self.vector_curr = [-1.123456789 , 1.132456789 ] # some really random specific number

	self.icr_radius = 6.5 # 2*1.7680   # INSTANTANEOUS CENTER_OF_ROTATION_radius
	self.vector_chassis = np.zeros((1,2))
	self.vector_pivot_wheel = np.zeros((1,2))
	self.angle_chassis = 0
	self.angle_pivot_wheel = 0
	self.steer_offset_dict   = { "rf":0, "lf":0, "lr":0, "rr":0 } # need for four_wheel_steering(...)
	self.radii_offset_dict   = { "rf":0, "lf":0, "lr":0, "rr":0 } # need for axle dict, and four_wheel_dif_velo(...)
	self.axles_location_dict = { "rf":(0,0), "lf":(0,0), "lr":(0,0), "rr":(0,0) } # needed if want to organize logic in dif functions ( to avoid large-monster-function i.e. calc_radii_dictionary(...))
	self.axles_offset_dict   = { "rf":0, "lf":0, "lr":0, "rr":0 } # maybe rename to axles_velocity_dict

	# START TIMER TO DELETE AFTER TESTING
	self.start_time = 0
	self.end_time = 0
	self.time_initiated = False
        rospy.Subscriber("/ground_truth",Odometry , callback=self.ground_truth_cb)
        self.x_rover = 0
        self.y_rover = 0
	self.ONE_ROTATION_TIME = 0
	# END   TIMER TO DELETE AFTER TESTING

        self.steering_cmd = 0
        self.linear_vel = 0
        self.linear_x = 0
        self.angular_z = 0
	self.lin_x_is_angular_wheel_vel = self.wheel_radius*2*math.pi # self.wheel_radius*2*math.pi  #== 1.72787595947 
        rospy.Subscriber("/csi_rover/cmd_vel",Twist , callback=self.directional_movement)

        rospy.init_node('rover_motion_controller', anonymous=True)
	self.RATE = 60 # used in chassis_alignment(...)
        rate = rospy.Rate(self.RATE)  # self.RATE hz


        while not rospy.is_shutdown():

           # check to see if there's an explicit yaw command
            if (self.angular_z != 0 and (round(self.angular_z,3) != round(self.yaw_prev,3))) :
		self.yaw_prev = self.angular_z

		"""
		# SKID STEERING
                self.rf_axle_pub.publish((self.linear_x + self.angular_z * self.wheel_width_separation / 2.0) / self.wheel_radius)
                self.rr_axle_pub.publish((self.linear_x + self.angular_z * self.wheel_width_separation / 2.0) / self.wheel_radius)
                self.lf_axle_pub.publish((self.linear_x - self.angular_z * self.wheel_width_separation / 2.0) / self.wheel_radius)
                self.lr_axle_pub.publish((self.linear_x - self.angular_z * self.wheel_width_separation / 2.0) / self.wheel_radius)

                # lock all steering joints to be zero
                self.synchronized_steering(0)


		"""
		#####################################
		#         EXPLICIT STEERING         #
		#####################################
		#####################################
		# STEER ALL WHEELS TO TURN IN PLACE #
		#####################################
		print( "in explicit ")
		rover_center_to_axle = math.hypot( self.wheel_separation_length/2 ,
						   self.wheel_separation_width/2 )
		tangent_angle = self.get_tangent_at( self.wheel_separation_length/2 ,
						     radius = rover_center_to_axle )
		tangent_angle = self.get_theta( np.array([1,0]) , np.array([1,tangent_angle]))
		self.four_wheel_steering( tangent_angle , yaw=True )

		#calculate how many seconds to turn angular_z amount of degrees that many degrees
		CALCULATED_TIME_FOR_ONE_ROTATION = 0
		if( round(self.linear_x,6) != 0 ):
			# time will be infinite if wheel speed is 0
			CALCULATED_TIME_FOR_ONE_ROTATION = 2*7.774999999999864/(self.linear_x+self.AVOID_0_DIVISION) 
		fraction_of_full_rotation = abs(self.angular_z)/(2*math.pi)
		self.yaw_stop = CALCULATED_TIME_FOR_ONE_ROTATION*fraction_of_full_rotation
		# Below is how rotation time should be calculated 
		# but there is something gazebo is doing which is not allowing it to work as such.
		#self.yaw_stop = (math.pi)*(2*(rover_center_to_axle))#/self.lin_x_is_angular_wheel_vel#/((self.lin_x_is_angular_wheel_vel+self.AVOID_0_DIVISION)) )


		#####################################
		# MOVE AXLES FOR "yaw_stop" seconds #
		#####################################
		self.yaw_start = rospy.get_time() 
		while(   (rospy.get_time() - self.yaw_start) < self.yaw_stop ):
			self.four_wheel_dif_velo( self.lin_x_is_angular_wheel_vel*self.linear_x , yaw=True )
		self.four_wheel_dif_velo( 0 , yaw=True )


		self.yaw_start = 0
		self.yaw_stop = 0
	    # reset for a new command with .0101020203210
	    elif( self.angular_z == .0000100010001 ): 
		print( "in reset ")
		self.yaw_prev = self.angular_z
            # else use crab steering
            elif(self.angular_z == 0) :   
		print( "in crab ")
		self.yaw_prev = self.angular_z
		if( not self.chassis_aligned() ): 
			self.four_wheel_dif_velo( self.linear_vel )
                	self.four_wheel_steering(self.steering_cmd)
			#chassis_alignment_change every 30hz /  1/30
			self.chassis_alignment() 
			if( 4 < self.start_time and round(self.x_rover,1)==0 and round(self.x_rover,1)==0):
			    self.ONE_ROTATION_TIME = time.time() - self.start_time
			    print( "self.ONE_ROTATION_TIME   :   !!!!!!!!!!!!!!!!           !!!!!!! ",self.ONE_ROTATION_TIME )
		else:
			#print( " IN SYNC !!!!!!!!!!!!!!!!!!!!!!!!!!")
			self.synchronized_axle( self.linear_vel )
                	self.synchronized_steering(self.steering_cmd)

            rate.sleep()
    def chassis_alignment( self ):
        # should change ONE_ROTATION_IN_SECONDS to a formula if possible 
	# to allow dynamic radii change, for now it works for : 2*1.7680 as the radius
	ONE_ROTATION_IN_SECONDS = 122.9029  
	RADIANS_A_SECONDS = 2*math.pi/ONE_ROTATION_IN_SECONDS  # ANGULAR_VELOCITY
	ROS_RATE = 30 # defined before, should eventually make as variable if people want to change it
	delta_per_loop = RADIANS_A_SECONDS/ROS_RATE
	sign = self.angle_chassis/abs(self.angle_chassis+self.AVOID_0_DIVISION)
	self.angle_chassis -= sign*delta_per_loop
	self.angle_pivot_wheel += sign*delta_per_loop
	self.steering_cmd += sign*delta_per_loop

    # move all of the steering joints to a position.
    # the parameter is an angle value in radians
    def synchronized_steering(self, angle):

        self.lf_steering_pub.publish(angle)
        self.rf_steering_pub.publish(angle)
        self.lr_steering_pub.publish(angle)
        self.rr_steering_pub.publish(angle)

    def synchronized_axle( self, velocity ):
        self.lf_axle_pub.publish( velocity )
        self.lr_axle_pub.publish( velocity )
        self.rf_axle_pub.publish( velocity )
        self.rr_axle_pub.publish( velocity )


    # move all of the steering joints to a position.
    # the parameter is an angle value in radians
    def four_wheel_steering(self, angle , yaw=False):
	#when set to (1,1) takes 55 sec to get to change chassis direction by pi/2 # having not changed vel
        print ( self.steer_offset_dict )
	if( not yaw ):
		self.lf_steering_pub.publish(angle + self.steer_offset_dict[ "lf" ] )
		self.rf_steering_pub.publish(angle + self.steer_offset_dict[ "rf" ] )
		self.lr_steering_pub.publish(angle - self.steer_offset_dict[ "lr" ] )
		self.rr_steering_pub.publish(angle - self.steer_offset_dict[ "rr" ] )
	else:
		# Prepare for a turn in place 
		self.lf_steering_pub.publish( -angle )
		self.rf_steering_pub.publish(angle )
		self.lr_steering_pub.publish(angle )
		self.rr_steering_pub.publish( -angle )

    def four_wheel_dif_velo( self, pivot_vel , yaw=False ):
	if( not yaw ):
		self.lf_axle_pub.publish( self.axles_offset_dict['lf'] )
		self.lr_axle_pub.publish( self.axles_offset_dict['lr'] )
		self.rf_axle_pub.publish( self.axles_offset_dict['rf'] )	
		self.rr_axle_pub.publish( self.axles_offset_dict['rr'] )
	else:
		sign = self.angular_z/abs(self.angular_z+self.AVOID_0_DIVISION)
		self.lf_axle_pub.publish( -pivot_vel*sign )
		self.lr_axle_pub.publish( -pivot_vel*sign )
		self.rf_axle_pub.publish( pivot_vel*sign )	
		self.rr_axle_pub.publish( pivot_vel*sign )	

    # Determine steering angle
    # Set linear_vel as magnitude
    # Range -pi/2 to pi/2
    # else use skid_steering
    def directional_movement(self, data):
	TEST_TIME = 34*9 # 7 ##########################################
	#hyp = lambda x , y : math.sqrt(math.pow(x, 2) + math.pow(y, 2))
        self.angular_z = data.angular.z
        self.linear_x = data.linear.x

        theta = 0
	hypotenuse = math.hypot(data.linear.x,data.linear.y)
        standard_cos = 0
        standard_sin = 0
        if( 0 < hypotenuse ):
            # using standard_sin and standard_cos to standardize the speed vector
            standard_cos = data.linear.x / hypotenuse
            standard_sin = data.linear.y / hypotenuse
	self.vector_curr = [ round(data.linear.x , 1 ) , round( data.linear.y , 1 ) ]
	# assign vehicle velocity
        sign = 1
        if data.linear.x != 0: # avoid division by 0
            sign = data.linear.x/abs(data.linear.x)
        self.linear_vel = self.linear_x*self.lin_x_is_angular_wheel_vel#*math.hypot(standard_cos,standard_sin)# was used before to standardize the speed... keeping in case it is needed in the future
 
	# if not same then user desires new direction
	if( not self.same_vect( self.vector_prev , self.vector_curr ) ):
	    self.vector_prev = self.vector_curr
            if( 0 < hypotenuse ):
		# using hypotenuse to see if vector not ( 0 , 0 ) 
                if( not self.time_initiated ):##############################
                    self.time_initiated = True##############################
                    self.start_time = time.time()###########################
                elif( round(time.time() - self.start_time,0) >= TEST_TIME ):
                    print( "TIME : ",  self.end_time - self.start_time)#####
                    self.linear_vel = 0#####################################
                    self.steering_cmd = 0###################################
                    exit()##################################################

                # adding .0000001 to avoid division by 0   
                theta = math.atan(standard_sin / (standard_cos + self.AVOID_0_DIVISION))
                self.steering_cmd = theta    
		#self.angle_chassis = -theta/2    # will eventually want to have them meet at 0
		#self.angle_pivot_wheel = theta/2 # will eventually want to have them meet at 0
		#self.angle_chassis -= theta# will eventually want to have them meet at 0
		self.angle_chassis = -theta# will eventually want to have them meet at 0
		self.angle_pivot_wheel = theta # will eventually want to have them meet at 0 ( used in angle_A_rf_rr = self.angle_pivot_wheel wish to use steering_cmd but not sure if steering_cmd needed )
            else:
                # (x,y) = (0,0)
                self.linear_vel = 0
                self.steering_cmd = 0
                if( self.time_initiated ):#############################
                    self.end_time = time.time()########################
                    print( "TIME : ",  self.end_time - self.start_time)
                    self.start_time = 0################################
                    self.time_initiated = False########################
                    exit()#############################################
	# if not the same then check chassis-to-wheel angle
	elif( self.same_vect( self.vector_prev , self.vector_curr ) ):
	    if( self.chassis_aligned() ):
		self.steer_offset_dict['rf'] = 0
		self.steer_offset_dict['lf'] = 0
		self.steer_offset_dict['lr'] = 0
		self.steer_offset_dict['rr'] = 0
	    else:
		self.steer_offset_dict['rf'] = 0
		self.steer_offset_dict['lf'] = 0
		self.calc_radii_dictionary()   # for use in four_wheel_dif_velo(...)
		self.calc_axles_dictionary()   # velocity -------------- might be able to consolidate calc_radii_dict() --> into calc_axles_dictionary() and remove class variable radii_offset_dict
		

	#print( "VELOCITY : ",self.linear_vel, " STEER THETA : ", self.steering_cmd , "TIME : ", time.time() - self.start_time)



    def same_vect(self,prev_vect,curr_vect):
        if( round(prev_vect[0],1) == round(curr_vect[0],1)
        and round(prev_vect[1],1) == round(curr_vect[1],1) ):
                return True
        return False

    def calc_axles_dictionary( self ):
	pass

    def calc_radii_dictionary( self ):

	THETA_SIGN = self.angle_pivot_wheel/abs(self.angle_pivot_wheel)      
	RADIUS_PIVOT = THETA_SIGN*2*1.7680  #6.5
	RADIUS_POINT = np.array([ 0 , RADIUS_PIVOT ])
	WHEEL_SEP_LENGTH = 1.5724    # will be hypotenuse for : rf <--> rr OR lf <--> lr 
	WHEEL_SEP_WIDTH  = 1.7680    # will be hypotenuse for : rf <--> lf
	WHEEL_SEP_CROSS  = math.hypot( WHEEL_SEP_LENGTH , WHEEL_SEP_WIDTH ) # lf <--> rr


	################################################
	# PRELIMINARY STEPS TO GET DIFFERENT AXLE SPEEDS
	################################################
        # Get line which runs through a 6.5 radius circle center 
	# and the "normalized point" that runs through each wheel

	################################################
	# CAN PROBABLY CONSOLIDATE THE FOLLOWING IF-ELSE
	# IN A SINGLE FUNCTION BUT IS HERE NOW FOR CLARITY
	# OF DIFFERENT VARIABLES
	################################################
        rise_run = lambda left_point , right_point : ( right_point[1] - left_point[1] )/( right_point[0] - left_point[0] + THETA_SIGN*(self.AVOID_0_DIVISION) ) 
	if( self.angle_chassis < 0 ):	
	    #pivot wheel is right front 
	    angle_A_rf_lf = 90 - self.angle_pivot_wheel
	    angle_A_rf_lr = self.angle_pivot_wheel - THETA_SIGN*self.get_theta( np.array( [WHEEL_SEP_LENGTH , 0 ]), np.array( [WHEEL_SEP_LENGTH , WHEEL_SEP_WIDTH ])) 
	    angle_A_rf_rr = self.angle_pivot_wheel
	    self.axles_location_dict['rf'] = [0,0] # is pivot
	    self.axles_location_dict['lf'] = self.sine_law( angle_A_rf_lf , WHEEL_SEP_WIDTH  ,sign = THETA_SIGN)
	    self.axles_location_dict['lr'] = self.sine_law( angle_A_rf_lr , WHEEL_SEP_CROSS  ,sign = THETA_SIGN)
            self.axles_location_dict['rr'] = self.sine_law( angle_A_rf_rr , WHEEL_SEP_LENGTH ,sign = THETA_SIGN)

	    # use axle_locations to to build an equation to get the slope (rise over run ) which 
	    # it intersects the centerof-6.5-radius and each wheel
	    slope_lf = rise_run( RADIUS_POINT , self.axles_location_dict['lf'] )
	    slope_lr = rise_run( self.axles_location_dict['lr'] , RADIUS_POINT )
	    slope_rr = rise_run( self.axles_location_dict['rr'] , RADIUS_POINT )
	    
	    self.update_steer_offset( slope_lf , RADIUS_POINT[1] , 'lf')
	    self.update_steer_offset( slope_lr , RADIUS_POINT[1] , 'lr')
	    self.update_steer_offset( slope_rr , RADIUS_POINT[1] , 'rr')

	else:
	    #pivot wheel is left front 
	    angle_A_lf_rf = 90 - THETA_SIGN*self.angle_pivot_wheel
	    angle_A_lf_lr = THETA_SIGN*self.angle_pivot_wheel  
	    angle_A_lf_rr = THETA_SIGN*self.angle_pivot_wheel - self.get_theta( np.array( [WHEEL_SEP_LENGTH , 0 ]), np.array( [WHEEL_SEP_LENGTH , WHEEL_SEP_WIDTH ]))
	    self.axles_location_dict['rf'] = self.sine_law( angle_A_lf_rf , WHEEL_SEP_WIDTH ,sign = THETA_SIGN )
	    self.axles_location_dict['lf'] = [0,0] # is pivot
	    self.axles_location_dict['lr'] = self.sine_law( angle_A_lf_lr , WHEEL_SEP_LENGTH  ,sign = THETA_SIGN)
            self.axles_location_dict['rr'] = self.sine_law( angle_A_lf_rr , WHEEL_SEP_CROSS ,sign = THETA_SIGN)

	    # use axle_locations to to build an equation to get the slope (rise over run ) which 
	    # it intersects the centerof-6.5-radius and each wheel
	    slope_rf = rise_run( RADIUS_POINT ,self.axles_location_dict['rf'] )
	    slope_lr = rise_run( self.axles_location_dict['lr'] ,RADIUS_POINT  )
	    slope_rr = rise_run( self.axles_location_dict['rr'],RADIUS_POINT  )
	    
	    self.update_steer_offset( slope_rf , RADIUS_POINT[1] , 'rf')
	    self.update_steer_offset( slope_lr , RADIUS_POINT[1] , 'lr')
	    self.update_steer_offset( slope_rr , RADIUS_POINT[1] , 'rr')


	#################################
	# BEGIN GET DIFFERENT AXLE SPEEDS
	#################################

        # Get radius for each circle that the axles are on
	distance_between_pts = lambda pt1 , pt2 : np.linalg.norm( [ pt1[0] - pt2[0] , pt1[1] - pt2[1] ] )  
        self.radii_offset_dict['rf'] = distance_between_pts( [ 0, RADIUS_PIVOT] , self.axles_location_dict['rf'] )
	self.radii_offset_dict['lf'] = distance_between_pts( [ 0, RADIUS_PIVOT] , self.axles_location_dict['lf'] )
	self.radii_offset_dict['lr'] = distance_between_pts( [ 0, RADIUS_PIVOT] , self.axles_location_dict['lr'] )
        self.radii_offset_dict['rr'] = distance_between_pts( [ 0, RADIUS_PIVOT] , self.axles_location_dict['rr'] )

	# With different radii, all velocities will be in relation to the pivot wheel's velocity
	#                                        current_radius / RADIUS_PIVOT
        self.axles_offset_dict['rf'] = self.linear_vel*(self.radii_offset_dict['rf']/ abs(RADIUS_PIVOT))
	self.axles_offset_dict['lf'] = self.linear_vel*(self.radii_offset_dict['lf']/ abs(RADIUS_PIVOT))
	self.axles_offset_dict['lr'] = self.linear_vel*(self.radii_offset_dict['lr']/ abs(RADIUS_PIVOT))
        self.axles_offset_dict['rr'] = self.linear_vel*(self.radii_offset_dict['rr']/ abs(RADIUS_PIVOT))



    # returns two missing sides
    def sine_law( self , angle_A , side_c , angle_C = math.pi/2 ,sign = 1):
        # want side a and side b
        # assuming side_c is the hypotenuse to a right triangle
        # returns [ side_a , side_b ]
        side_a = (side_c/angle_C)*math.sin( sign*angle_A )
        side_b = (side_c/angle_C)*math.sin( math.pi-sign*angle_A-sign*angle_C )
        return [ side_a , side_b ]

    def update_steer_offset( self, slope , radius , wheel_name ):
	sign = radius/abs(radius)
	# find the two points which intersect the circle by adjusting slope to be at center of circle
	# and run through each wheel and the radius to get its slope
	center_to_pt_to_circle_edge_intersection = self.xy_intersection( slope , radius )

	# get the tangent at those points on the circle
        tangent_parrallel_to_pt_tan = self.get_tangent_at( center_to_pt_to_circle_edge_intersection[0] , radius )

	# make vector for wheel according to that tangent_slope
	tan_vector_from_origin = np.array([ 1 , tangent_parrallel_to_pt_tan ])
	x_vector = np.array([ 1 , 0 ])
        
        # update the angle between <1,0> and the vector_tangent for the other three wheels
        # to know the offset needed for each wheel in four_wheel_steering 

        self.steer_offset_dict[ wheel_name ] =  sign*self.get_theta( tan_vector_from_origin , x_vector )
	

    # get tangent slope at x from pivot wheel circle equation
    def get_tangent_at( self, x , radius ):
	tangent_slope = -1*( x )/(math.sqrt(radius**2-(x)**2))
	return tangent_slope


    # gets intersection of a : line y = mx+b , and circle (y - b)**2 + (x)**2 = b**2 
    def xy_intersection(self, m , b ):
	x_intersection = b/(math.sqrt( m**2 + 1 ))
	y_intersection = m*x_intersection + b
	return [ x_intersection , y_intersection ]

    # check if the chassis is aligned with the pivot_wheel
    def chassis_aligned(self):
	return (round(self.angle_chassis,1) == 0 and round( self.angle_pivot_wheel , 1 ) == 0 ) 


    def get_theta(self , a , b ):
	return math.acos( (a).dot(b)/(np.linalg.norm(a)*np.linalg.norm(b)) )




    #--------------------------------for testing ##############################################################################################################
    def ground_truth_cb( self , data ):
	self.x_rover = data.pose.pose.position.x
        self.y_rover = data.pose.pose.position.y



if __name__ == '__main__':
    try:
        RoverMotionController()
    except rospy.ROSInterruptException:
        pass
