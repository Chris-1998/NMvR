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


class Publisher(Node):

    def __init__(self):
        super().__init__('publisher') 
        self.publisher = self.create_publisher(String, 'vel', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angular = 0
        self.linear = 0

    def timer_callback(self):
        print('in pub ', self.angular) # angularna rychlost a linearna rychlost
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
        if self.theta > 6.28: # > 360 stupnov
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
        print('angularvel', constant * (self.steering_angle(goal.x, goal.y) - self.theta))
        return constant * (self.steering_angle(goal.x, goal.y) - self.theta)

    def listener_callback1(self, msg):

        self.map = np.array(eval(msg.data)).astype(float)

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
        self.get_logger().info('I heard: "%s"' % msg)
        self.goal = msg

        self.publisher.angular = 0
        self.publisher.linear = 0
        rclpy.spin_once(self.publisher)

        i = 0
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

            self.map[int(self.oldY)][int(self.oldX)] = 0
            self.map[int(y)][int(x)] = self.robot
            #vymazem zo starej pozicie a dam na novu

            degree = [0, 45, 90, 135, 180, 225, 270, 315, 360]
            theta = min(degree, key=lambda x: abs(x - degrees(abs(self.theta - 6.28))))

            if i >= 1:
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

            self.oldX = x
            self.oldY = y
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


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
