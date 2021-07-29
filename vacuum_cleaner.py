#!/usr/bin/env python
# license removed for brevity

import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class VacuumCleaner:
    
# This method is to initialize the environment
    def __init__(self,maxX_p,maxY_p,side_p,radius_p,cube_centre_p,cylinder_centre_p,positive_y_p):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.init_node('path_instructor', anonymous=True)
        self.rot=Twist()
        self.maxX=maxX_p
        self.maxY=maxY_p
        self.side=side_p
        a=self.side/2
        area_square=0
        radius=radius_p
        area_cyl= 0 
        self.cube_centre=cube_centre_p
        self.cylinder_centre=cylinder_centre_p
        self.current_vertical=1
        self.current_horizontal=1
        self.back_cnt=0
        self.positive_y=positive_y_p
        self.moves=[]
        self.moves_complete=[]
        self.grids_covered= []
        self.yaw=0.0
        self.rate = rospy.Rate(10)
        self.forbidden=[[-1]*(self.maxX+2) for _ in range((self.maxY+2))]
        self.facing_right=0
        self.facing_left=0
        self.current_x=0.45
        for i in range(0,(int(self.maxX)+2)):
            self.forbidden[0].pop(0)
            self.forbidden[0].append(int(i))
            self.forbidden[int(self.maxY)+1].pop(0)
            self.forbidden[int(self.maxY)+1].append(int(i))
        for i in range(0,self.maxY+2):
            self.forbidden[i].pop(0)
            self.forbidden[i].append(0)
            self.forbidden[i].pop(0)
            self.forbidden[i].append((self.maxY)+1)
        self.forbidden[0].remove((self.maxY)+1)
        self.forbidden[0].append(1)
        self.forbidden[(self.maxY)+1].remove((self.maxY)+1)   
        self.forbidden[(self.maxY)+1].append(1) 
        for l in self.cube_centre:
            for m in range(int(math.ceil(l[1]-a)),int(math.floor(l[1]+(a-1)+1))):
                for k in range(int(math.ceil(l[0]-a)),int(math.floor(l[0]+(a-1)+1))):
                    if k not in self.forbidden[m]:
                        area_square+=1
                        self.forbidden[m].pop(0)
                        self.forbidden[m].append(k)
        for l in self.cylinder_centre:
            for m in range(int(math.ceil(l[1]-radius)),int(math.floor(l[1]+(radius-1)+1))):
                for k in range(int(math.ceil(l[0]-radius)),int(math.floor(l[0]+(radius-1)+1))):
                    if (m>=l[1]) and (k>=l[0]):
                        if((pow((l[0]-k),2)+pow((l[1]-m),2))<pow(radius,2)):
                            if k not in self.forbidden[m]:
                                area_cyl+=1
                                self.forbidden[m].pop(0)
                                self.forbidden[m].append(k)  
                    elif (m>=l[1]) and (k<=l[0]):
                        if((pow((l[0]-k-1),2)+pow((l[1]-m),2))<pow(radius,2)):
                            if k not in self.forbidden[m]:
                                area_cyl+=1
                                self.forbidden[m].pop(0)
                                self.forbidden[m].append(k)
                    elif (m<=l[1]) and (k<=l[0]):
                        if((pow((l[0]-k-1),2)+pow((l[1]-m-1),2))<pow(radius,2)):
                            if k not in self.forbidden[m]:
                                area_cyl+=1
                                self.forbidden[m].pop(0)
                                self.forbidden[m].append(k)
                    else :
                        if((pow((l[0]-k),2)+pow((l[1]-m-1),2))<pow(radius,2)):
                            if k not in self.forbidden[m]:
                                area_cyl+=1
                                self.forbidden[m].pop(0)
                                self.forbidden[m].append(k)
        self.grids_to_cover= ((self.maxX*self.maxY)-area_square-area_cyl)  
        self.first_step()
        self.tune()

    def first_step(self):
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
        target_dist = self.current_x+0.3
        while self.current_x < (target_dist-0.01) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.1*(target_dist-self.current_x)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
        self.tune()
        self.rest()

    def tune(self):
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        if self.yaw < 0.0 :
            while self.yaw < ( - 0.02) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=0.5*(0.0-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot)
                self.sub.unregister()
                self.rate.sleep()  
        elif self.yaw > 0.0 :
            while self.yaw > (0.02) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=0.5*(0.0-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot)
                self.sub.unregister()
                self.rate.sleep()  
        self.rest()           

# This function checks if a robot can make a left turn or not            
    def left_is_free(self):
        if (self.current_horizontal-1) in self.forbidden[self.current_vertical]:
            return False
        else:
            return True

# This function checks if a robot can make a right turn or not
    def right_is_free(self):
        if (self.current_horizontal+1) in self.forbidden[self.current_vertical]:
            return False
        else:
            return True

# This function checks if a robot can go up or not
    def up_is_free(self):
        if (self.current_horizontal) in self.forbidden[self.current_vertical+1]:
            return False
        else:
            return True

# This function checks if a robot can go down or not
    def down_is_free(self):
        if (self.current_horizontal) in self.forbidden[self.current_vertical-1]:
            return False
        else:
            return True                      
                
# This method is for moving the robot around
    def move(self,direction):
        self.forbidden[self.current_vertical].pop(0)    
        self.forbidden[self.current_vertical].append(self.current_horizontal) 
        self.moves.append([self.current_horizontal,self.current_vertical])
        self.moves_complete.append([self.current_horizontal,self.current_vertical])
        for i in range(self.back_cnt):
            self.moves[-1-i].pop()
            self.moves[-1-i].pop(0)
        self.back_cnt=0
        if(direction=='U'):
            self.current_vertical+=1 
            self.go_up()
        elif(direction=='D'):
            self.current_vertical-=1 
            self.go_down()
        elif(direction=='L'):
            self.current_horizontal-=1
            self.go_left()
        elif(direction=='R'):
            self.current_horizontal+=1
            self.go_right() 
        
#This method is to plan the next moves
    def plan_move(self):
        if self.positive_y:
            if self.left_is_free():
                self.move('L')                                                                                  
            elif self.up_is_free():
                self.move('U')
            elif self.down_is_free():
                self.move('D')
                self.positive_y = not self.positive_y    
            elif self.right_is_free():
                self.move('R')
                self.positive_y = not self.positive_y
            else:
                self.back_track()
        else:
            if self.left_is_free():
                self.move('L')
            elif self.down_is_free():
                self.move('D')
            elif self.up_is_free():
                self.move('U')
                self.positive_y = not self.positive_y
            elif self.right_is_free():
                self.move('R')
                self.positive_y = not self.positive_y
            else:
                self.back_track()
        
            
# This function helps the robot to trace back it's steps in-case it gets stuck somewhere
    def back_track(self):
        if self.back_cnt==0:
            self.forbidden[self.current_vertical].pop(0)   
            self.forbidden[self.current_vertical].append(self.current_horizontal)
        horizontal_back_track= self.moves[-1-self.back_cnt][0]- self.current_horizontal
        vertical_back_track= self.moves[-1-self.back_cnt][1]-self.current_vertical
        self.moves_complete.append([self.current_horizontal,self.current_vertical])
        if horizontal_back_track:
            if horizontal_back_track==1:
                self.current_horizontal+=1
                self.go_right()
            else:
                self.current_horizontal-=1
                self.go_left()
        else:
            if vertical_back_track==1:
                self.current_vertical+=1 
                self.go_up()  
            else:
                self.current_vertical-=1 
                self.go_down()
        self.back_cnt+=1      
    
#This method makes the bot turn left in gazebo
    def go_left(self):
        if self.facing_right==1:
            self.facing_right=-1
            self.go_right()
        if self.facing_left == 0:
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            target_rad = 90 * math.pi/180
            while self.yaw < (target_rad-0.05) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=2.0*(target_rad-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot) 
                self.sub.unregister()
                self.rate.sleep() 
            self.rest()
            self.rot.linear.x=0.05
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot)
            time.sleep(4.59*3/2)
            self.rest()
            self.facing_left=1
            
        elif self.facing_left==1 :
            self.rot.linear.x=0.05
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot)
            time.sleep(4.59*3/2)
            self.rest()

        elif self.facing_left == -1:
            target_rad = 0 * math.pi/180
            while self.yaw > (target_rad+0.05) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=0.2*(target_rad-self.yaw)
                rospy.loginfo(self.rot)
                self.pub.publish(self.rot)
                print (self.yaw)
                self.sub.unregister()
                self.rate.sleep()  
            self.rest()
            self.facing_left=0
            

#This method makes the bot turn right in gazebo
    def go_right(self):
        if self.facing_left==1:
            self.facing_left=-1
            self.go_left()
        if self.facing_right == 0:
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            target_rad = -90 * math.pi/180
            while self.yaw > (target_rad+0.05) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=2.0*(target_rad-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot)
                self.sub.unregister()
                self.rate.sleep()  
            self.rest()
            self.rot.linear.x=0.05
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot)
            time.sleep(4.59*3/2)
            self.rest()
            self.facing_right=1
            
        
        elif self.facing_right==1 :
            self.rot.linear.x=0.05
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot)
            time.sleep(4.59*3/2)
            self.rest()
        
        elif self.facing_right == -1:
            target_rad = 0 * math.pi/180
            while self.yaw < (target_rad-0.05) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=2.0*(target_rad-self.yaw)
                rospy.loginfo(self.rot)
                self.pub.publish(self.rot)
                print (self.yaw)
                self.rate.sleep()  
                self.sub.unregister()
            self.rest() 
            self.facing_right=0

#This method makes the bot move up in gazebo
    def go_up(self):
        if self.facing_left==1:
            self.facing_left=-1
            self.go_left()
        elif self.facing_right==1:
            self.facing_right=-1
            self.go_right()    
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
        target_dist = self.current_x+0.3
        while self.current_x < (target_dist-0.01) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.1*(target_dist-self.current_x)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
            self.rate.sleep() 
        self.tune()
        self.rest()
  
#This method makes the bot move down in gazebo
    def go_down(self):
        if self.facing_left==1:
            self.facing_left=-1
            self.go_left()
        elif self.facing_right==1:
            self.facing_right=-1
            self.go_right()    
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
        target_dist = self.current_x-0.3
        while self.current_x > (target_dist+0.01) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.1*(target_dist-self.current_x)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
            self.rate.sleep() 
        self.tune()
        self.rest()

#Resting time for the bot
    def rest(self):
        self.rot.linear.x=0.0
        self.rot.angular.z=0.0
        rospy.loginfo(self.rot)
        self.pub.publish(self.rot)
        time.sleep(0.8)

    def get_rotation (self,msg):
        self.current_x=msg.pose.pose.position.x
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        



def main():                                                           # Change the first 6 variables of this method according to your map
    maxX_p =15                                                     # Maximum X-Dimension of map
    maxY_p = 20                                                     # Maximum Y-Dimension of map        
    side_p = 4                                                        # length of side of cube 
    radius_p = 2                                                      # radius of cylinder 
    cube_centre_p =  [[5,5],[12,5],[9,16]]                            # List of co-ordinates of cubes 
    cylinder_centre_p = [[4,11],[10,11],[14,19]]                       # List of co-ordinates of cylinder    
    positive_y_p = True 
    obj=VacuumCleaner(maxX_p,maxY_p,side_p,radius_p,cube_centre_p,cylinder_centre_p,positive_y_p)
    while len(obj.grids_covered) < obj.grids_to_cover-1:
        if [obj.current_horizontal,obj.current_vertical] not in obj.grids_covered:
            obj.grids_covered.append([obj.current_horizontal,obj.current_vertical])
        obj.plan_move()
         
    obj.moves.append([obj.current_horizontal,obj.current_vertical])
    obj.moves_complete.append([obj.current_horizontal,obj.current_vertical])
    print(obj.moves_complete)
main()    
