import math
class VacuumCleaner:
    
# This method is to initialize the environment
    def __init__(self,maxX_p,maxY_p,side_p,radius_p,cube_centre_p,cylinder_centre_p,positive_y_p):
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
        self.forbidden=[[-1]*(self.maxX+2) for _ in range((self.maxY+2))]
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
        elif(direction=='D'):
            self.current_vertical-=1
        elif(direction=='L'):
            self.current_horizontal-=1
        elif(direction=='R'):
            self.current_horizontal+=1
        
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
            else:
                self.current_horizontal-=1
        else:
            if vertical_back_track==1:
                self.current_vertical+=1 
                self.positive_y= not self.positive_y
            else:
                self.current_vertical-=1 
                self.positive_y= not self.positive_y
        self.back_cnt+=1      
        

def main():                                                           # Change the first 6 variables of this method according to your map
    maxX_p = 15                                                       # Maximum X-Dimension of map
    maxY_p = 20                                                       # Maximum Y-Dimension of map        
    side_p = 4                                                        # length of side of cube 
    radius_p = 2                                                      # radius of cylinder 
    cube_centre_p =  [[5,5],[12,5],[9,16]]                            # List of co-ordinates of cubes 
    cylinder_centre_p = [[4,11],[10,11],[14,19]]                      # List of co-ordinates of cylinder    
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