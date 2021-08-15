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
        self.rate = rospy.Rate(20)
        self.forbidden=[[-1]*(self.maxX+2) for _ in range((self.maxY+2))]
        self.facing_right=0
        self.facing_left=0
        self.current_x=0.45
        self.current_y=-0.45
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
        target_dist = self.current_x+0.300000
        self.prevent_slip(1,'U')
        while self.current_x < (target_dist-0.020000) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.3*(target_dist-self.current_x)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
            self.rate.sleep()
        self.tune()
        self.rest()

    def tune(self):
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        if self.yaw < 0.0 :
            while self.yaw < ( - 0.0190000) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=5*(0.0-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot)
                self.sub.unregister()
                self.rate.sleep()  
        elif self.yaw > 0.0 :
            while self.yaw > (0.019000) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=5*(0.0-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot)
                self.sub.unregister()
                self.rate.sleep()  
        self.rest()           


    def tune_sideways(self,facing):
        if facing=='L':
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            target_rad = 90 * math.pi/180
            if self.yaw < target_rad :
                while self.yaw < ( target_rad- 0.0190000) :
                    self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                    self.rot.linear.x= 0.0
                    self.rot.angular.z=5*(target_rad-self.yaw)
                    rospy.loginfo(self.rot)
                    print (self.yaw)
                    self.pub.publish(self.rot)
                    self.sub.unregister()
                    self.rate.sleep()  
            elif self.yaw > target_rad:
                while self.yaw > (target_rad+0.019000) :
                    self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                    self.rot.linear.x= 0.0
                    self.rot.angular.z=5*(target_rad-self.yaw)
                    rospy.loginfo(self.rot)
                    print (self.yaw)
                    self.pub.publish(self.rot)
                    self.sub.unregister()
                    self.rate.sleep()  
        elif facing=='R':
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            target_rad = -90 * math.pi/180
            if self.yaw < target_rad :
                while self.yaw < ( target_rad- 0.0190000) :
                    self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                    self.rot.linear.x= 0.0
                    self.rot.angular.z=5*(target_rad-self.yaw)
                    rospy.loginfo(self.rot)
                    print (self.yaw)
                    self.pub.publish(self.rot)
                    self.sub.unregister()
                    self.rate.sleep()  
            elif self.yaw > target_rad :
                while self.yaw > (target_rad+0.019000) :
                    self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                    self.rot.linear.x= 0.0
                    self.rot.angular.z=5*(target_rad-self.yaw)
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
                self.rot.angular.z=0.5*(target_rad-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot) 
                self.sub.unregister()
                self.rate.sleep() 
            self.rest()
            self.go_straight_left()
            
            self.facing_left=1
            
        elif self.facing_left==1 :
            self.go_straight_left()
            

        elif self.facing_left == -1:
            target_rad = 0 * math.pi/180
            while self.yaw > (target_rad+0.05) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=0.5*(target_rad-self.yaw)
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
                self.rot.angular.z=0.5*(target_rad-self.yaw)
                rospy.loginfo(self.rot)
                print (self.yaw)
                self.pub.publish(self.rot)
                self.sub.unregister()
                self.rate.sleep()  
            self.rest()
            self.go_straight_right()
            
            self.facing_right=1
            
        
        elif self.facing_right==1 :
            self.go_straight_right()
            
        
        elif self.facing_right == -1:
            target_rad = 0 * math.pi/180
            while self.yaw < (target_rad-0.05) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.0
                self.rot.angular.z=0.5*(target_rad-self.yaw)
                rospy.loginfo(self.rot)
                self.pub.publish(self.rot)
                print (self.yaw)
                self.rate.sleep()  
                self.sub.unregister()
            self.rest() 
            self.facing_right=0

#This method makes the bot move forward in left direction in gazebo
    def go_straight_left(self):
        
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
        target_dist = self.current_y+0.30000000000
        self.prevent_slip(1,'L')  
        while self.current_y < (target_dist-0.020000) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.3*(target_dist-self.current_y)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
            self.rate.sleep() 
        self.rest()
        self.tune_sideways('L')
        self.rest()


#This method makes the bot move forward in right direction in gazebo
    def go_straight_right(self):
        
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
        target_dist = self.current_y-0.30000000000
        self.prevent_slip(-1,'R')  
        while self.current_y > (target_dist+0.020000) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.3*(self.current_y -target_dist)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
            self.rate.sleep() 
        self.rest()
        self.tune_sideways('R')
        self.rest()

#This method makes the bot move up in gazebo
    def go_up(self):
        if self.facing_left==1:
            self.facing_left=-1
            self.go_left()
        elif self.facing_right==1:
            self.facing_right=-1
            self.go_right()    
           
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
        target_dist = self.current_x+0.30000000000
        self.prevent_slip(1,'U')  
        while self.current_x < (target_dist-0.018000) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.3*(target_dist-self.current_x)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
            self.rate.sleep() 
        self.rest()
        self.tune()
        self.rest()
  
Skip to content
Pull requests
Issues
Marketplace
Explore
@Utkarsh-S-1110
Utkarsh-S-1110 /
VacuumCleaner

1
0

    1

Code
Issues
Pull requests
Actions
Projects
Wiki
Security
Insights

    Settings

VacuumCleaner/
in
master

1

#!/usr/bin/env python

2

# license removed for brevity

3

​

4

import rospy

5

import math

6

import time

7

from geometry_msgs.msg import Twist

8

from nav_msgs.msg import Odometry

9

from tf.transformations import euler_from_quaternion

10

​

11

class VacuumCleaner:

12

    

13

# This method is to initialize the environment

14

    def __init__(self,maxX_p,maxY_p,side_p,radius_p,cube_centre_p,cylinder_centre_p,positive_y_p):

15

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

16

        rospy.init_node('path_instructor', anonymous=True)

17

        self.rot=Twist()

18

        self.maxX=maxX_p

19

        self.maxY=maxY_p

20

        self.side=side_p

21

        a=self.side/2

22

        area_square=0

23

        radius=radius_p

24

        area_cyl= 0 

25

        self.cube_centre=cube_centre_p

26

        self.cylinder_centre=cylinder_centre_p

27

        self.current_vertical=1

28

        self.current_horizontal=1

29

        self.back_cnt=0

30

        self.positive_y=positive_y_p

31

        self.moves=[]

32

        self.moves_complete=[]

33

        self.grids_covered= []

34

        self.yaw=0.0

35

        self.rate = rospy.Rate(10)

36

        self.forbidden=[[-1]*(self.maxX+2) for _ in range((self.maxY+2))]

37

        self.facing_right=0

38

        self.facing_left=0

39

        self.current_x=0.45

40

        for i in range(0,(int(self.maxX)+2)):

41

            self.forbidden[0].pop(0)

42

            self.forbidden[0].append(int(i))

43

            self.forbidden[int(self.maxY)+1].pop(0)

44

            self.forbidden[int(self.maxY)+1].append(int(i))

45

        for i in range(0,self.maxY+2):

46

            self.forbidden[i].pop(0)

47

            self.forbidden[i].append(0)

48

            self.forbidden[i].pop(0)

49

            self.forbidden[i].append((self.maxY)+1)

50

        self.forbidden[0].remove((self.maxY)+1)

51

        self.forbidden[0].append(1)

52

        self.forbidden[(self.maxY)+1].remove((self.maxY)+1)   

53

        self.forbidden[(self.maxY)+1].append(1) 

54

        for l in self.cube_centre:

55

            for m in range(int(math.ceil(l[1]-a)),int(math.floor(l[1]+(a-1)+1))):

56

                for k in range(int(math.ceil(l[0]-a)),int(math.floor(l[0]+(a-1)+1))):

57

                    if k not in self.forbidden[m]:

58

                        area_square+=1

59

                        self.forbidden[m].pop(0)

60

                        self.forbidden[m].append(k)

61

        for l in self.cylinder_centre:

62

            for m in range(int(math.ceil(l[1]-radius)),int(math.floor(l[1]+(radius-1)+1))):

63

                for k in range(int(math.ceil(l[0]-radius)),int(math.floor(l[0]+(radius-1)+1))):

64

                    if (m>=l[1]) and (k>=l[0]):

65

                        if((pow((l[0]-k),2)+pow((l[1]-m),2))<pow(radius,2)):

66

                            if k not in self.forbidden[m]:

67

                                area_cyl+=1

68

                                self.forbidden[m].pop(0)

69

                                self.forbidden[m].append(k)  

70

                    elif (m>=l[1]) and (k<=l[0]):

71

                        if((pow((l[0]-k-1),2)+pow((l[1]-m),2))<pow(radius,2)):

72

                            if k not in self.forbidden[m]:

73

                                area_cyl+=1

74

                                self.forbidden[m].pop(0)

75

                                self.forbidden[m].append(k)

76

                    elif (m<=l[1]) and (k<=l[0]):

77

                        if((pow((l[0]-k-1),2)+pow((l[1]-m-1),2))<pow(radius,2)):

78

                            if k not in self.forbidden[m]:

79

                                area_cyl+=1

80

                                self.forbidden[m].pop(0)

81

                                self.forbidden[m].append(k)

82

                    else :

83

                        if((pow((l[0]-k),2)+pow((l[1]-m-1),2))<pow(radius,2)):

84

                            if k not in self.forbidden[m]:

85

                                area_cyl+=1

86

                                self.forbidden[m].pop(0)

87

                                self.forbidden[m].append(k)

88

        self.grids_to_cover= ((self.maxX*self.maxY)-area_square-area_cyl)  

89

        self.first_step()

90

        self.tune()

91

​

92

    def first_step(self):

93

        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 

94

        target_dist = self.current_x+0.3

95

        while self.current_x < (target_dist-0.01) :

96

            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

97

            self.rot.linear.x= 0.1*(target_dist-self.current_x)

98

            self.rot.angular.z=0.0

99

            rospy.loginfo(self.rot)

100

            self.pub.publish(self.rot) 

101

            self.sub.unregister()

102

        self.tune()

103

        self.rest()

104

​

105

    def tune(self):

106

        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

107

        if self.yaw < 0.0 :

108

            while self.yaw < ( - 0.02) :

109

                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

110

                self.rot.linear.x= 0.0

111

                self.rot.angular.z=0.5*(0.0-self.yaw)

112

                rospy.loginfo(self.rot)

113

                print (self.yaw)

114

                self.pub.publish(self.rot)

115

                self.sub.unregister()

116

                self.rate.sleep()  

117

        elif self.yaw > 0.0 :

118

            while self.yaw > (0.02) :

119

                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

120

                self.rot.linear.x= 0.0

121

                self.rot.angular.z=0.5*(0.0-self.yaw)

122

                rospy.loginfo(self.rot)

123

                print (self.yaw)

124

                self.pub.publish(self.rot)

125

                self.sub.unregister()

126

                self.rate.sleep()  

127

        self.rest()           

128

​

129

# This function checks if a robot can make a left turn or not            

130

    def left_is_free(self):

131

        if (self.current_horizontal-1) in self.forbidden[self.current_vertical]:

132

            return False

133

        else:

134

            return True

135

​

136

# This function checks if a robot can make a right turn or not

137

    def right_is_free(self):

138

        if (self.current_horizontal+1) in self.forbidden[self.current_vertical]:

139

            return False

140

        else:

141

            return True

142

​

143

# This function checks if a robot can go up or not

144

    def up_is_free(self):

145

        if (self.current_horizontal) in self.forbidden[self.current_vertical+1]:

146

            return False

147

        else:

148

            return True

149

​

150

# This function checks if a robot can go down or not

151

    def down_is_free(self):

152

        if (self.current_horizontal) in self.forbidden[self.current_vertical-1]:

153

            return False

154

        else:

155

            return True                      

156

                

157

# This method is for moving the robot around

158

    def move(self,direction):

159

        self.forbidden[self.current_vertical].pop(0)    

160

        self.forbidden[self.current_vertical].append(self.current_horizontal) 

161

        self.moves.append([self.current_horizontal,self.current_vertical])

162

        self.moves_complete.append([self.current_horizontal,self.current_vertical])

163

        for i in range(self.back_cnt):

164

            self.moves[-1-i].pop()

165

            self.moves[-1-i].pop(0)

166

        self.back_cnt=0

167

        if(direction=='U'):

168

            self.current_vertical+=1 

169

            self.go_up()

170

        elif(direction=='D'):

171

            self.current_vertical-=1 

172

            self.go_down()

173

        elif(direction=='L'):

174

            self.current_horizontal-=1

175

            self.go_left()

176

        elif(direction=='R'):

177

            self.current_horizontal+=1

178

            self.go_right() 

179

        

180

#This method is to plan the next moves

181

    def plan_move(self):

182

        if self.positive_y:

183

            if self.left_is_free():

184

                self.move('L')                                                                                  

185

            elif self.up_is_free():

186

                self.move('U')

187

            elif self.down_is_free():

188

                self.move('D')

189

                self.positive_y = not self.positive_y    

190

            elif self.right_is_free():

191

                self.move('R')

192

                self.positive_y = not self.positive_y

193

            else:

194

                self.back_track()

195

        else:

196

            if self.left_is_free():

197

                self.move('L')

198

            elif self.down_is_free():

199

                self.move('D')

200

            elif self.up_is_free():

201

                self.move('U')

202

                self.positive_y = not self.positive_y

203

            elif self.right_is_free():

204

                self.move('R')

205

                self.positive_y = not self.positive_y

206

            else:

207

                self.back_track()

208

        

209

            

210

# This function helps the robot to trace back it's steps in-case it gets stuck somewhere

211

    def back_track(self):

212

        if self.back_cnt==0:

213

            self.forbidden[self.current_vertical].pop(0)   

214

            self.forbidden[self.current_vertical].append(self.current_horizontal)

215

        horizontal_back_track= self.moves[-1-self.back_cnt][0]- self.current_horizontal

216

        vertical_back_track= self.moves[-1-self.back_cnt][1]-self.current_vertical

217

        self.moves_complete.append([self.current_horizontal,self.current_vertical])

218

        if horizontal_back_track:

219

            if horizontal_back_track==1:

220

                self.current_horizontal+=1

221

                self.go_right()

222

            else:

223

                self.current_horizontal-=1

224

                self.go_left()

225

        else:

226

            if vertical_back_track==1:

227

                self.current_vertical+=1 

228

                self.go_up()  

229

            else:

230

                self.current_vertical-=1 

231

                self.go_down()

232

        self.back_cnt+=1      

233

    

234

#This method makes the bot turn left in gazebo

235

    def go_left(self):

236

        if self.facing_right==1:

237

            self.facing_right=-1

238

            self.go_right()

239

        if self.facing_left == 0:

240

            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

241

            target_rad = 90 * math.pi/180

242

            while self.yaw < (target_rad-0.05) :

243

                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

244

                self.rot.linear.x= 0.0

245

                self.rot.angular.z=2.0*(target_rad-self.yaw)

246

                rospy.loginfo(self.rot)

247

                print (self.yaw)

248

                self.pub.publish(self.rot) 

249

                self.sub.unregister()

250

                self.rate.sleep() 

251

            self.rest()

252

            self.rot.linear.x=0.05

253

            self.rot.angular.z=0.0

254

            rospy.loginfo(self.rot)

255

            self.pub.publish(self.rot)

256

            time.sleep(4.58*3/2)

257

            self.rest()

258

            self.facing_left=1

259

            

260

        elif self.facing_left==1 :

261

            self.rot.linear.x=0.05

262

            self.rot.angular.z=0.0

263

            rospy.loginfo(self.rot)

264

            self.pub.publish(self.rot)

265

            time.sleep(4.58*3/2)

266

            self.rest()

267

​

268

        elif self.facing_left == -1:

269

            target_rad = 0 * math.pi/180

270

            while self.yaw > (target_rad+0.05) :

271

                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

272

                self.rot.linear.x= 0.0

273

                self.rot.angular.z=0.2*(target_rad-self.yaw)

274

                rospy.loginfo(self.rot)

275

                self.pub.publish(self.rot)

276

                print (self.yaw)

277

                self.sub.unregister()

278

                self.rate.sleep()  

279

            self.rest()

280

            self.facing_left=0

281

            

282

​

283

#This method makes the bot turn right in gazebo

284

    def go_right(self):

285

        if self.facing_left==1:

286

            self.facing_left=-1

287

            self.go_left()

288

        if self.facing_right == 0:

289

            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

290

            target_rad = -90 * math.pi/180

291

            while self.yaw > (target_rad+0.05) :

292

                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

293

                self.rot.linear.x= 0.0

294

                self.rot.angular.z=2.0*(target_rad-self.yaw)

295

                rospy.loginfo(self.rot)

296

                print (self.yaw)

297

                self.pub.publish(self.rot)

298

                self.sub.unregister()

299

                self.rate.sleep()  

300

            self.rest()

301

            self.rot.linear.x=0.05

302

            self.rot.angular.z=0.0

303

            rospy.loginfo(self.rot)

304

            self.pub.publish(self.rot)

305

            time.sleep(4.58*3/2)

306

            self.rest()

307

            self.facing_right=1

308

            

309

        

310

        elif self.facing_right==1 :

311

            self.rot.linear.x=0.05

312

            self.rot.angular.z=0.0

313

            rospy.loginfo(self.rot)

314

            self.pub.publish(self.rot)

315

            time.sleep(4.58*3/2)

316

            self.rest()

317

        

318

        elif self.facing_right == -1:

319

            target_rad = 0 * math.pi/180

320

            while self.yaw < (target_rad-0.05) :

321

                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

322

                self.rot.linear.x= 0.0

323

                self.rot.angular.z=2.0*(target_rad-self.yaw)

324

                rospy.loginfo(self.rot)

325

                self.pub.publish(self.rot)

326

                print (self.yaw)

327

                self.rate.sleep()  

328

                self.sub.unregister()

329

            self.rest() 

330

            self.facing_right=0

331

​

332

#This method makes the bot move up in gazebo

333

    def go_up(self):

334

        if self.facing_left==1:

335

            self.facing_left=-1

336

            self.go_left()

337

        elif self.facing_right==1:

338

            self.facing_right=-1

339

            self.go_right()    

340

        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 

341

        target_dist = self.current_x+0.3

342

        while self.current_x < (target_dist-0.01) :

343

            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

344

            self.rot.linear.x= 0.1*(target_dist-self.current_x)

345

            self.rot.angular.z=0.0

346

            rospy.loginfo(self.rot)

347

            self.pub.publish(self.rot) 

348

            self.sub.unregister()

349

            self.rate.sleep() 

350

        self.tune()

351

        self.rest()

352

  

353

#This method makes the bot move down in gazebo

354

    def go_down(self):

355

        if self.facing_left==1:

356

            self.facing_left=-1

357

            self.go_left()

358

        elif self.facing_right==1:

359

            self.facing_right=-1

360

            self.go_right()    

361

        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 

362

        target_dist = self.current_x-0.3

363

        while self.current_x > (target_dist+0.01) :

364

            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

365

            self.rot.linear.x= 0.1*(target_dist-self.current_x)

366

            self.rot.angular.z=0.0

367

            rospy.loginfo(self.rot)

368

            self.pub.publish(self.rot) 

369

            self.sub.unregister()

370

            self.rate.sleep() 

371

        self.tune()

372

        self.rest()

373

​

374

#Resting time for the bot

375

    def rest(self):

376

        self.rot.linear.x=0.0

377

        self.rot.angular.z=0.0

378

        rospy.loginfo(self.rot)

379

        self.pub.publish(self.rot)

380

        time.sleep(0.8)

381

​

382

    def get_rotation (self,msg):

383

        self.current_x=msg.pose.pose.position.x

384

        orientation_q = msg.pose.pose.orientation

385

        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

386

        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)

387

        

388

​

389

​

390

​

391

def main():                                                           # Change the first 6 variables of this method according to your map

392

    maxX_p =15                                                     # Maximum X-Dimension of map

393

    maxY_p = 20                                                     # Maximum Y-Dimension of map        

394

    side_p = 4                                                        # length of side of cube 

395

    radius_p = 2                                                      # radius of cylinder 

396

    cube_centre_p =  [[5,5],[12,5],[9,16]]                            # List of co-ordinates of cubes 

397

    cylinder_centre_p = [[4,11],[10,11],[14,19]]                       # List of co-ordinates of cylinder    

398

    positive_y_p = True 

399

    obj=VacuumCleaner(maxX_p,maxY_p,side_p,radius_p,cube_centre_p,cylinder_centre_p,positive_y_p)

400

    while len(obj.grids_covered) < obj.grids_to_cover-1:

401

        if [obj.current_horizontal,obj.current_vertical] not in obj.grids_covered:

402

            obj.grids_covered.append([obj.current_horizontal,obj.current_vertical])

403

        obj.plan_move()

404

         

405

    obj.moves.append([obj.current_horizontal,obj.current_vertical])

406

    obj.moves_complete.append([obj.current_horizontal,obj.current_vertical])

407

    print(obj.moves_complete)

408

main()    

409

​

@Utkarsh-S-1110
Commit changes
Commit summary
Optional extended description
Commit directly to the master branch.
Create a new branch for this commit and start a pull request. Learn more about pull requests.

    © 2021 GitHub, Inc.
    Terms
    Privacy
    Security
    Status
    Docs

    Contact GitHub
    Pricing
    API
    Training
    Blog
    About


#This method makes the bot move down in gazebo
    def go_down(self):
        if self.facing_left==1:
            self.facing_left=-1
            self.go_left()
        elif self.facing_right==1:
            self.facing_right=-1
            self.go_right()    
          
        self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
        target_dist = self.current_x-0.300000000
        self.prevent_slip(-1,'D')  
        while self.current_x > (target_dist+0.00500000) :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
            self.rot.linear.x= 0.3*(target_dist-self.current_x)
            self.rot.angular.z=0.0
            rospy.loginfo(self.rot)
            self.pub.publish(self.rot) 
            self.sub.unregister()
            self.rate.sleep() 
        self.rest()
        self.tune()
        self.rest()
        

#Resting time for the bot
    def rest(self):
        self.rot.linear.x=0.0
        self.rot.angular.z=0.0
        rospy.loginfo(self.rot)
        self.pub.publish(self.rot)
        time.sleep(1)
        

    def prevent_slip(self,sign,facing) :
        if(facing=='U') :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
            start=self.current_x-(0.02*sign)
            target_dist = self.current_x+(0.150000000000*sign)
            
            while self.current_x < (target_dist-(0.00700000*sign)) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.3*(self.current_x-start)
                self.rot.angular.z=0.0
                rospy.loginfo(self.rot)
                self.pub.publish(self.rot) 
                self.sub.unregister()
                self.rate.sleep()
        elif(facing=='D') :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
            start=self.current_x-(0.02*sign)
            target_dist = self.current_x+(0.150000000000*sign)
             
            while self.current_x > (target_dist-(0.00700000*sign)) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.3*(self.current_x-start)
                self.rot.angular.z=0.0
                rospy.loginfo(self.rot)
                self.pub.publish(self.rot) 
                self.sub.unregister()
                self.rate.sleep()        
        elif (facing=='L'):
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
            start=self.current_y-(0.02*sign)
            target_dist = self.current_y+(0.16000000000*sign)
            
            while self.current_y < (target_dist-(0.00700000*sign)) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.3*(self.current_y-start)
                self.rot.angular.z=0.0
                rospy.loginfo(self.rot)
                self.pub.publish(self.rot) 
                self.sub.unregister()
                self.rate.sleep()

        elif (facing=='R') :
            self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation) 
            start=self.current_y-(0.02*sign)
            target_dist = self.current_y+(0.155000000000*sign)
            
            while self.current_y >(target_dist-(0.00700000*sign)) :
                self.sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
                self.rot.linear.x= 0.3*(start -self.current_y)
                self.rot.angular.z=0.0
                rospy.loginfo(self.rot)
                self.pub.publish(self.rot) 
                self.sub.unregister()
                self.rate.sleep()        
                

    def get_rotation (self,msg):
        self.current_x=msg.pose.pose.position.x
        self.current_y=msg.pose.pose.position.y
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
