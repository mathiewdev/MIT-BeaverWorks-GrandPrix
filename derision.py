#!/usr/bin/env python

#the line above makes it a python script, the lines below make it a ros script

import rospy

from ackermann_msgs.msg import AckermannDriveStamped

from sensor_msgs.msg import LaserScan

import cv2

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import numpy as np

##################################
#MODULARIZED CODE - FINAL PROGRAM#
#CONTRIBUTORS - Sam Carpenter,   #
#Gabi Tessier, Ben Ellis, Anthony#
#Grandel, Matt Daniels-Deihl,    #
#Tyler Harwood, and Liam Brister.#
#Special thanks to Gabe Madonna, #
#Albert Go, Matt Tung, and Harry.#
################################## 

'''ASSIGNMENTS - 
Sam and Gabi - PD Controller, code layout and general logic.
Ben and Liam - Follow the yellow brick road, help Matt/Anthony.
Matt and Anthony - Red/green light.
READ THE CODE TO FIND THE SPECS FOR YOUR FUNCTION.'''


class DownTheHall(): #the class contains everything

        def __init__(self): #this function just runs the callback function

            	rospy.Subscriber('/scan', LaserScan, self.safe)

        def safe(self, data):

                msg = AckermannDriveStamped() #the message is created - msg is the drive commando

                center = len(data.ranges) // 2 #finds the center

                #the following block creates the minimum distances within a certain range on either side

                rightmin = center / 4

                rightmax  = rightmin*3

                leftmin = len(data.ranges) - rightmin

                leftmax = len(data.ranges) - rightmax

                left = min(data.ranges[leftmax:leftmin])

                right = min(data.ranges[rightmin:rightmax])

                maxo = 1.5

                if left > maxo:
			
			left = maxo

                if right > maxo:

                        right = maxo

                setp = (left + right) / 2 #setp is the distance it wants to be from both walls

                dist = min(right, left) #dist is the closest wall

                error = setp - dist #this is how close it is to the setpoint - the ERROR!
                
                if right > left: #makes it so that it can turn both ways

                        error = 0 - error

                msg.drive.steering_angle = error * 1.25 #calculates steering angle - the higher the gain, the higher the steering angle.
                
                try: #this is in a try except loop for the first iteration, so that there's no NameError because last is undefined.

                        derivative = last - error #this finds the derivative, and subtracts a smaller version of it from the angle. the gain can be anywhere
                        msg.drive.steering_angle -= derivative

                except(NameError):

                        pass
	
		front = data.ranges[center] #lidar value in front
               
                rightC = min(data.ranges[rightmin:center])    #left
                
                leftC = min(data.ranges[center:leftmin])
                counter = 0

                if front < 1.5:    #checking to see if there is a car in front 
                        if leftC > 1:    #checking to see if left is clear, if yes than turn
                                for item in range(center, leftmin):
                                    if  data.ranges[item] > 1:
                                        counter += 1
                                        
                                    else:
                                        counter = 0
                                    
                                    if counter >= 40:
                                    
                                        msg.drive.steering_angle -= (center - item) * 0.1


                        elif rightC > 1:        #checking to see if right is clear, if yes than turn 
                                for item in range(rightmin, center):
                                    if  data.ranges[item] > 1:
                                        counter += 1
                                        
                                    else:
                                        counter = 0
                                    
                                    if counter >= 40:
                                    
                                        msg.drive.steering_angle -= (center - item) * 0.1

                if front < 1.3:

                        msg.drive.speed *= front - 0.3

		                #aaaaand puuubbblish! /vesc/ackermann_cmd_mux/input/navigation is the topic for gazebo, /ackermann_cmd_mux/input/teleop is for the car itself

                pub_command = rospy.Publisher('/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size = 10)

                pub_command.publish(msg)

                last = error #sets last

if __name__ == '__main__': #creates da node and runs da code
        
        rospy.init_node('derision')

        node = DownTheHall()

        rospy.spin()
