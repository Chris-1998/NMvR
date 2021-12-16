import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
import numpy as np
from random import randint
import matplotlib.pyplot as plt
from tkinter import *
import pygame
from pygame.locals import *
from math import cos, radians, sin, sqrt, pow, atan2, degrees, ceil

#import d_star


class DStar():
    
    def __init__(self, s_start, s_goal, map):
        self.s_start = s_start
        self.s_goal = s_goal
        self.map = map
        
        self.k_m = 0
        
        self.g = {}
        self.rhs = {}
        self.U = {}
        
        for idx in range(0, self.map.shape[0]):
            for jdx in range(0, self.map.shape[1]):
                self.g[(idx, jdx)] = float('inf')
                self.rhs[(idx, jdx)] = float('inf')
                
        self.rhs[self.s_goal] = 0.0
        self.U[self.s_goal] = self. calcKey(self.s_goal)
        self.path = []
    
    
    def go(self):
        self.calcShortestPath()
        print(self.createPath())
    
    #* pocitanie heuristiky
    def getH(self, s_start, s):
        print("----POCITAM H ALE NEJDEM TI TO----")
        h = abs(s_start[0] - s[0]) + abs(s_start[1] - s[1])
        
        return h
    
    #* cost function
    def getC(self, s_start, s_goal):
        cost = abs(s_start[0] - s_goal[0]) + abs(s_start[1] - s_goal[1])
        
        return cost
    
    
    def getStart(self, current_pos):
        my_list = []
        
        for ndx in self.getNeighbours(current_pos):
            if not self.collide(current_pos, ndx):
                my_list[ndx] = self.g[ndx]
        start = min(my_list, key=my_list.get)

        return start
    
    #* vytrvorenie zoznamu pozicii vypocitanej cesty
    def createPath(self):
        path = [self.s_start]
        s = self.s_start
        
        while True:
            my_list = {}
            
            for pdx in self.getNeighbours(s):
                if not self.collide(s, pdx):
                    my_list[pdx] = self.g[pdx]
                else:
                    pass
                    print("---- Wall occured!----")
            s = min(my_list, key=my_list.get)
            path.append(s)
            
            if s == self.s_goal:
                break 

        self.path = path
        
        return path
    
    #vysetrovanie susedov
    def getNeighbours(self, s):
        neighbours = set()
        possible_motion = [(0,1), (1,0), (1,1), (0,-1), (-1,0), (-1,-1), (1,-1), (-1,1)]
        
        for kdx in possible_motion:
            s_new = tuple([s[ldx] + kdx[ldx] for ldx in range(2)])
            
            if  (s_new[0] >= 0 and s_new[0] < self.map.shape[0] and \
                 s_new[1] >= 0 and s_new[1] < self.map.shape[1]) and \
                 self.map[s_new[0]][s_new[1]] != 1: 
                
                neighbours.add(s_new)
            
        return neighbours

    
    def calcKey(self, s):
        key = [ min(self.g[s],
                self.rhs[s]) + self.k_m + self.getH(self.s_start, s),
                min(self.g[s], self.rhs[s])]
        
        return key
    
    
    def updateVerteces(self, s):
        
        if s != self.s_goal:
            self.rhs[s] = float('inf')
            
            for mdx in self.getNeighbours(s):
                self.rhs[s] = min(self.rhs[s], self.g[mdx] + self.getC(s, mdx))
        
        if s in self.U:
            self.U.pop(s)
        
        if self.g[s] != self.rhs[s]:
            self.U[s] = self. calcKey(s)
    
    
    def obstacle(self, old, act, x, y):
        
        self.k_m += self.getH(old, act)
        
        if self.map[(x, y)] == 0:
            self.map[(x, y)] = 1
            self.rhs[(x, y)] = float('inf')
            self.g[(x, y)] = float('inf')
        else:
            self.map[(x, y)] = 0
            self.updateVerteces((x, y))
        for ydx in self.getNeighbours((x, y)):
            self.updateVerteces(ydx)
    
    
    def collide(self, s_start, s_goal):
        
        if self.map[s_start[0]][s_start[1]] == 1:
            return True
        
        if self.map[s_goal[0]][s_start[1]] == 1:
            return True
        
        if s_start[0] != s_goal[0] and s_start[1] != s_goal[1]:
            if s_goal[0] - s_start[0] == s_start[1] - s_goal[1]:
                s1 = (min(s_start[0], s_goal[0]), min(s_start[1], s_goal[1]))
                s2 = (max(s_start[0], s_goal[0]), max(s_start[1], s_goal[1]))
            else:
                s1 = (min(s_start[0], s_goal[0]), max(s_start[1], s_goal[1]))
                s2 = (max(s_start[0], s_goal[0]), min(s_start[1], s_goal[1]))

            if self.map[s1[0]][s1[1]] == 1 or self.map[s2[0]][s2[1]] == 1:
                return True
  
        return False
    

    def topKey(self):
        s = min(self.U, key=self.U.get)
        return s, self.U[s]


    def calcShortestPath(self):   
        while True:
            s, s_value = self.topKey()

            if s_value > self.calcKey(self.s_start) and self.rhs[self.s_start] == self.g[self.s_start]:
                break
            else:
                pass

            self.U.pop(s)

            if  s_value < self.calcKey(s):
                self.U[s] = self.calcKey(s)
            elif self.g[s] > self.rhs[s]:
                self.g[s] = self.rhs[s]
                for zdx in self.getNeighbours(s):
                    self.updateVerteces( zdx)
            else:
                self.g[s] = float("inf")
                self.updateVerteces( s)
                for wdx in self.getNeighbours(s):
                    self.updateVerteces( wdx)

class Publisher(Node): 

    def __init__(self):
        super().__init__('publisher') 
        self.publisher = self.create_publisher(String, 'vel', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angular = 0
        self.linear = 0

    def timer_callback(self):
        print('in pub ', self.angular) 
        msg = String()
        msg.data = str(self.linear) + "," + str(self.angular)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(String, 'vel', self.listener_callback, 10)
        self.x = 1
        self.y = 1
        self.theta = 0
        self.angular = 0
        self.linear = 0
        self.l = 1

    def listener_callback(self, msg):
        self.linear = float(msg.data.split(",")[0])
        self.angular = float(msg.data.split(",")[1])
        self.theta = self.theta + self.angular
        if self.theta > 6.28: 
            self.theta -= 6.28
        dcenter = self.linear
        self.x += round(dcenter * cos(self.theta), 1)
        self.y += round(dcenter * sin(self.theta), 1)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher = Publisher()
        self.subscriber = Subscriber()

        self.subscription1 = self.create_subscription(String, 'map', self.listener_callback1, 10) 
        self.subscription2 = self.create_subscription(Pose, 'goal', self.listener_callback2, 10) 
        self.subscription3 = self.create_subscription(String, 'stena', self.listener_callback3, 10)
        self.subscriptions
        self.robot = 2
        self.x = 1
        self.y = 1
        self.oldX = 1
        self.oldY = 1
        self.theta = 0
        self.map = []
        self.goal = Pose()
        self.goal.x = 1.0
        self.goal.y = 1.0
        self.distanceTolerance = 0.5
        self.direction = ''
        self.start = (self.x, self.y)
        self.my_dstar = None
        self.new = None
        self.togo = (10, 10)
        self.calculated_path = []
        self.depricated_path = []
        
        self.screen = pygame.display.set_mode((1000, 1000))
        pygame.display.set_caption('Mapa')

        self.wall = pygame.image.load('/home/nmvr/dev_ws/src/zadanieNMvR/zadanieNMvR/wall.png')
        self.void = pygame.image.load('/home/nmvr/dev_ws/src/zadanieNMvR/zadanieNMvR/void.png')
        self.robotIMG = pygame.image.load('/home/nmvr/dev_ws/src/zadanieNMvR/zadanieNMvR/robot.png')


    def euclidean_distance(self, goal): 
        rclpy.spin_once(self.subscriber)
        self.x = self.subscriber.x
        self.y = self.subscriber.y
        self.theta = self.subscriber.theta
        print('x ', self.x, 'y ', self.y, 'theta ', self.theta)
        return sqrt(pow((goal.x - self.x), 2) +
                    pow((goal.y - self.y), 2))

    def linear(self):
        return 1

    def steering_angle(self, goal): 
        steering_angle = atan2(goal.y - self.y, goal.x - self.x)
        if steering_angle < 0:
            steering_angle += 6.28
        print('steering angle ', steering_angle)
        return steering_angle

    def angular(self, goal, constant=2): 
        print('angular', constant * (self.steering_angle(goal.x, goal.y) - self.theta))
        return constant * (self.steering_angle(goal.x, goal.y) - self.theta)

    def listener_callback1(self, msg):          

        self.map = np.array(eval(msg.data)).astype(float)

        if len(self.direction) > 2:
            print('working')
            
            #*sem dame prepocet mapy po zmene prekazky

            self.my_dstar.s_start = self.start
            print("----DOSIEL SOM KU PREKAZKE A SOM V PICI Z TOHO---")
            self.my_dstar.obstacle(self.start,self.new,self.x,self.y)
            print("----DOSIEL SOM PO POCITANIE NOVEJ TRASY A DALEJ JEBEM NA TO---")
            self.my_dstar.calcShortestPath()
            self.my_path = list(self.my_dstar.createPath())

            x = int(self.direction.split()[0])
            y = int(self.direction.split()[1])
            if self.map[x][y] != self.robot:
                if self.map[y][x] == 0:
                    self.map[y][x] = 1
                else:
                    self.map[y][x] = 0
                self.map[self.y][self.x] = 0
                np.savetxt('/home/nmvr/dev_ws/src/zadanieNMvR/zadanieNMvR/map.csv', self.map, delimiter=',', fmt='%d')
            self.direction = ''

        y = 0
        if (float(self.y) % 1) >= 0.5:
            y = ceil(self.y)
        else:
            y = round(self.y)

        x = 0
        if (float(self.x) % 1) >= 0.5:
            x = ceil(self.x)
        else:
            x = round(self.x)
        

        self.map[int(y)][int(x)] = self.robot

        tile_list = []
        row_count = 0
        for row in self.map:
            col_count = 0
            for tile in row:
                if tile == 0:
                    img = pygame.transform.scale(self.void, (10, 10))
                    img_rect = img.get_rect()
                    img_rect.x = col_count * 10
                    img_rect.y = row_count * 10
                    tile = (img, img_rect)
                    tile_list.append(tile)
                if tile == 1:
                    img = pygame.transform.scale(self.wall, (10, 10))
                    img_rect = img.get_rect()
                    img_rect.x = col_count * 10
                    img_rect.y = row_count * 10
                    tile = (img, img_rect)
                    tile_list.append(tile)
                if tile == 2:
                    img = pygame.transform.scale(self.robotIMG, (10, 10))
                    img_rect = img.get_rect()
                    img_rect.x = col_count * 10
                    img_rect.y = row_count * 10
                    tile = (img, img_rect)
                    tile_list.append(tile)
                col_count += 1
            row_count += 1

        for tilee in tile_list:
            self.screen.blit(tilee[0], tilee[1])

        for line in range(100):
            pygame.draw.line(self.screen, (255, 255, 255), (0, line * 10), (1000, line * 10))
            pygame.draw.line(self.screen, (255, 255, 255), (line * 10, 0), (line * 10, 1000))

        pygame.display.update()

    def listener_callback2(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        self.goal = msg

        #* DSTAR ZACIATOK - VYTVORENIE OBJEKTU A HLADANIE CESTY
        self.my_dstar = DStar(self.start, self.togo, self.map)
        self.my_dstar.calcShortestPath()
        #print(self.my_dstar.getStart(self.s_start))
        self.depricated_path = self.calculated_path.copy()
        self.calculated_path = list(self.my_dstar.createPath())
        print("---- VYPOCITAL SOM CESTU BRATU---")
        print(self.calculated_path)
        
        self.publisher.angular = 0
        self.publisher.linear = 0
        rclpy.spin_once(self.publisher)

        i = 0
        qdx = 0
        while self.euclidean_distance(self.goal) > self.distanceTolerance:
            y = 0
            if (float(self.y) % 1) >= 0.5:
                y = ceil(self.y)
            else:
                y = round(self.y)

            x = 0
            if (float(self.x) % 1) >= 0.5:
                x = ceil(self.x)
            else:
                x = round(self.x)

            #* posunutie pozicie robota    
            y = self.calculated_path[qdx][1]
            x = self.calculated_path[qdx][0]

            self.map[int(self.oldY)][int(self.oldX)] = 0
            self.map[int(y)][int(x)] = self.robot
            

            degree = [0, 45, 90, 135, 180, 225, 270, 315, 360]
            theta = min(degree, key=lambda x: abs(x - degrees(abs(self.theta - 6.28))))
            
            if i >= 1:
                print("----DOSIEL SOM PO RIADOK 411---")
                '''
                if (theta == 0 or theta == 360) and self.map[int(y)][int(x + 1)] == 1:
                    self.goal.x = float(x)
                    self.goal.y = float(y)
                    break
                elif theta == 45 and self.map[int(y - 1)][int(x + 1)]:
                    self.goal.x = float(x)
                    self.goal.y = float(y)
                    break
                elif theta == 90 and self.map[int(y - 1)][int(x)]:
                    self.goal.x = float(x)
                    self.goal.y = float(y)
                    break
                elif theta == 135 and self.map[int(y - 1)][int(x - 1)]:
                    self.goal.x = float(x)
                    self.goal.y = float(y)
                    break
                elif theta == 180 and self.map[int(y)][int(x - 1)]:
                    self.goal.x = float(x)
                    self.goal.y = float(y)
                    break
                elif theta == 225 and self.map[int(y + 1)][int(x - 1)]:
                    self.goal.x= float(x)
                    self.goal.y = float(y)
                    break
                elif theta == 270 and self.map[int(y + 1)][int(x)]:
                    self.goal.x = float(x)
                    self.goal.y = float(y)
                    break
                elif theta == 315 and self.map[int(y + 1)][int(x + 1)]:
                    self.goal.x = float(x)
                    self.goal.y = float(y)
                    break
                '''

            tile_list = []
            row_count = 0
            for row in self.map:
                col_count = 0
                for tile in row:
                    if tile == 0:
                        img = pygame.transform.scale(self.void, (10, 10))
                        img_rect = img.get_rect()
                        img_rect.x = col_count * 10
                        img_rect.y = row_count * 10
                        tile = (img, img_rect)
                        tile_list.append(tile)
                    if tile == 1:
                        img = pygame.transform.scale(self.wall, (10, 10))
                        img_rect = img.get_rect()
                        img_rect.x = col_count * 10
                        img_rect.y = row_count * 10
                        tile = (img, img_rect)
                        tile_list.append(tile)
                    if tile == 2:
                        img = pygame.transform.scale(self.robotIMG, (10, 10))
                        img_rect = img.get_rect()
                        img_rect.x = col_count * 10
                        img_rect.y = row_count * 10
                        tile = (img, img_rect)
                        tile_list.append(tile)
                    col_count += 1
                row_count += 1

            for tilee in tile_list:
                self.screen.blit(tilee[0], tilee[1])

            for line in range(100):
                pygame.draw.line(self.screen, (255, 255, 255), (0, line * 10), (1000, line * 10))
                pygame.draw.line(self.screen, (255, 255, 255), (line * 10, 0), (line * 10, 1000))

            pygame.display.update()


            #* zmena pozcie robota podla vypocitanej trasy
            self.oldX = self.calculated_path[qdx][0]
            self.oldY = self.calculated_path[qdx][1]
            qdx +=1
            i += 1

            difference = abs(self.theta - self.steering_angle(self.goal)) 
            if difference > 0.01:
                self.publisher.angular = self.steering_angle(self.goal) - self.theta 
            else:
                self.publisher.angular = 0 

            self.publisher.angular = int(degrees(self.publisher.angular)) 

            if self.publisher.angular == 45 or self.publisher.angular == 135 or self.publisher.angular == 225 or self.publisher.angular == 315:
                self.publisher.linear = sqrt(2) 
            else:
                self.publisher.linear = self.linear()

            self.publisher.angular = radians(self.publisher.angular)
            rclpy.spin_once(self.publisher)

    def listener_callback3(self, msg11):
        self.get_logger().info('I heard: "%s"' % msg11.data)
        self.direction = msg11.data

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
