#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import math
import imu_o as im
#import camera

#maximum velocity used for the robot
max_vel = 0.09
max_ang = math.pi*0.3

max_vel_r = 0.075
max_ang_r = math.pi*0.276

"""
Adds the current imu data to the samples list. Camera data is currently not added.
@param samples: the samples object in the movement function, that collects the recorded data
"""
def get_vals(samples):
    imu_vals = im.get_vals()
    for imu_val in imu_vals:
        samples['accel'].append(imu_val['accel'])
        samples['gyro'].append(imu_val['gyro'])
        samples['compass'].append(imu_val['compass'])
        #TODO make image smaller?
        #samples['image'].append(camera.get_val())

"""
This class provides methods for recording specific types of samples. All of the method return the following:
-samples: The samples dict. It has three entries for the three sensors: "accel", "gyro" and "compass". Each entry is a list of 3-axis vectors of the sensor data, with index 0 being the start of the recording.
-velocity/angle/radius: Values provided by the user or defined by the type of sample. These values further describe the sample, and can later be used for filtering samples.
"""
class MovementSet:
    """
    The constructor. It uses the publisher pub to send control data to the robot at the rate hertz.
    @param pub: publisher to use for controlling
    @param hertz: rate at which to send control data
    """
    def __init__(self, pub, hertz):
        self.pub = pub
        self.hertz = hertz
        self.rate = rospy.Rate(hertz)
	

    """
    Recording functions for all curve samples. They call the standard curve function with their respective parameters.
    """
    def mv_curve_large(self, left=True):
        return self.mv_curve(velocity=max_vel, radius=0.91, angle=max_ang*0.2, left=left, short=True)
    def mv_curve_middle(self, left=True):
        return self.mv_curve(velocity=max_vel, radius=0.4, angle=max_ang*0.4, left=left)
    def mv_curve_small(self, left=True):
        return self.mv_curve(velocity=max_vel, radius=0.25, angle=max_ang*0.7, left=left)
    def mv_curve_ml(self, left=True):
        return self.mv_curve(velocity=max_vel, radius=(0.4+0.91)*0.5, angle=max_ang*0.3, left=left, short=True)
    def mv_curve_sm(self, left=True):
        return self.mv_curve(velocity=max_vel, radius=(0.4+0.25)*0.5, angle=max_ang*0.55, left=left)

    """
    Recording function for a curve. If angle is not provided, it is calculated using radius and velocity. The robot will drive in total to a 90 degree turn, with one second straight before and after the curve.
    Angle, velocity and radius have same/similar unit (ex. rad/sec, meter/sec, meter).
    @param velocity: velocity for forward movement
    @param radius: the radius of the curve (should be provided for more accurate sample description)
    @param angle: angular velocity for the curve, if None, velocity/radius is used
    @param left: if the curve goes left or right
    @param short: the curve is a little short and doesn't do a whole 90 degree turn (needed for large curves, as the sample data is too big otherwise)
    """
    def mv_curve(self, velocity, radius, angle, left=True, short=False):
        if angle == None:
            angle = velocity/radius
        #seconds for the curve (calculated for a 90 degree turn)
        secs = math.pi/2/angle
        if short:
            secs = secs/2
        #seconds to go straight
        secs_straight = 2

        if not left:
            angle = -angle

        #movement data
        vec_zero = Vector3(0,0,0)
        vec_fwd = Vector3(velocity,0,0)
        vec_curve = Vector3(0,0,angle)
        move_fwd = Twist(vec_fwd, vec_zero)
        move_curve = Twist(vec_fwd, vec_curve)

        #movement: first half straight, curve, second half straight
        total = int(self.hertz*secs_straight)
        first_half = int(total/2)
        sec_half = total - first_half
        steps = int(self.hertz*secs)

        samples = {'accel' : [], 'gyro' : [], 'compass' : [], 'image' : []}
        recording = False
        #one second straight
        for i in range(first_half):
            if recording:
                get_vals(samples)
            elif i>=first_half*0.5:
                recording = True
                im.get_vals() #empty_queue
            self.pub.publish(move_fwd)
            self.rate.sleep()

        #curve
        for i in range( steps ):
            get_vals(samples)
            self.pub.publish(move_curve)
            self.rate.sleep()

        #one second straight
        for i in range(sec_half):
            if i<=sec_half*0.6:
                get_vals(samples)
            self.pub.publish(move_fwd)
            self.rate.sleep()
        return samples,velocity,angle,radius

    """
    Moves in a straight line for secs seconds. After that, the robot moves back to the starting point by going backwards (not recorded).
    This function is used for straight samples and object samples.
    @param secs: how long to move forward in seconds
    """
    def mv_straight(self, secs=4):
        vel = max_vel

        vec_zero = Vector3(0,0,0)
        vec_fwd = Vector3(vel,0,0)
        vec_bwd = Vector3(-vel,0,0)
        move_fwd = Twist(vec_fwd, vec_zero)
        move_bwd = Twist(vec_bwd, vec_zero)

        steps = int(self.hertz*secs)
        
        samples = {'accel' : [], 'gyro' : [], 'compass' : [], 'image' : []}
        recording = False
        for i in range( steps ):
            if recording:
                get_vals(samples)
            elif i>=steps*0.2:
                im.get_vals() #empty queue
                recording = True
            self.pub.publish(move_fwd)
            self.rate.sleep()

        for i in range( steps ):
            self.pub.publish(move_bwd)
            self.rate.sleep()

        return samples,vel,0,0

    """
    Essentially like mv_straight, expect it only moves back about 70% of the way forward. A wall needs to be set up in front of the robot.
    @param secs: how long to move forward
    """
    def mv_wall(self, secs=3):
        vel = max_vel

        vec_zero = Vector3(0,0,0)
        vec_fwd = Vector3(vel,0,0)
        vec_bwd = Vector3(-vel,0,0)
        move_fwd = Twist(vec_fwd, vec_zero)
        move_bwd = Twist(vec_bwd, vec_zero)

        steps = int(self.hertz*secs)
        steps2 = int(self.hertz*secs*0.7)

        samples = {'accel' : [], 'gyro' : [], 'compass' : [], 'image' : []}
        recording = False
        for i in range( steps ):
            if recording:
                get_vals(samples)
            elif i>=steps*0.5:
                im.get_vals() #empty queue
                recording = True
            self.pub.publish(move_fwd)
            self.rate.sleep()
        
        #*0.5 since it shouldn't go back too far
        for i in range( steps2 ):
            self.pub.publish(move_bwd)
            self.rate.sleep()

        return samples,vel,0,0

    """
    The movement function for angles. The robot moves according to the given angle in degrees,and then tries to move back to the starting point (doesn't exactly work, attention during recording is required).
    Only 25 timesteps before and after the moment the robot acutally drives the angle are saved (so in total 50 steps).
    @param degree: the angle to move
    @param left: if left or right
    """
    def mv_angle(self, degree, left=True):
        #half the window length -> window size is steps_to_save*2
        steps_to_save = 25
    
        vel = max_vel
        secs = 2
        
        angle = math.pi * degree/180
        
        velY = math.sin(angle)*vel
        velX = math.cos(angle)*vel

        #make sure there is enough time to get back (worst case 180 degree -> 2*secs)
        #if secs_back is chosen too low, max speed might get exceeded
        #use law of cosines to calculate best value
        #ex. for angle=180 -> secs_back = 2*secs
        #ex. for angle=90 -> secs_back = sqrt(2)*secs
        #note: angle needs to be converted (by "mirroring"), since we need the angle inside of the triangle (currently angle = <outside angle>-180)
        secs_back = math.sqrt(secs*secs*2*(1-math.cos(math.pi - angle)))
        velX_back = (-vel-velX)*secs/secs_back
        velY_back = (-velX)*secs/secs_back
        #if degree == 0:
        #    secs_back = 2*secs

        if not left:
            velY = -velY

        vec_zero = Vector3(0,0,0)
        vec_fwd = Vector3(vel,0,0)
        vec_side = Vector3(velX,velY,0)
        vec_back = Vector3(velX_back,velY_back,0)
        move_fwd = Twist(vec_fwd, vec_zero)
        move_side = Twist(vec_side, vec_zero)
        move_back = Twist(vec_back, vec_zero)
        
        steps = int(self.hertz*secs)
        steps_back = int(self.hertz*secs_back+0.5)
        
        samples = {'accel' : [], 'gyro' : [], 'compass' : [], 'image' : []}
        recording = False
        for i in range( steps ):
            if recording:
                get_vals(samples)
            elif i>=steps*0.2:
                im.get_vals() #empty queue
                recording = True
            self.pub.publish(move_fwd)
            self.rate.sleep()
        
        #middle of the sample
        pivot = len(samples['accel'])+10

        for i in range( steps ):
            if i<=steps*0.8:
                get_vals(samples)
            self.pub.publish(move_side)
            self.rate.sleep()
        
        #go back to start
        for i in range( steps_back ):
            self.pub.publish(move_back)
            self.rate.sleep()
            
        #cut sample to fixed window size
        keys = ['accel','gyro','compass'] #(keys except image)
        for key in keys:
            data = samples[key]
            samples[key] = data[(pivot-steps_to_save):(pivot+steps_to_save)]
            assert len(samples[key]) == steps_to_save*2 #in case steps_to_save is too big
        
        #return the degree for the radius, not really needed
        return samples,vel,angle,degree
		
				





    


