#!/usr/bin/env python
import roslib
import rospy
import time
from fw_wrapper.srv import *
from map import *
from collections import deque, defaultdict
import math

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def setMotorWheelSpeedSync(motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync', motor_ids[0], target_vals[0], 2, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class Robot:
    def __init__(self):
        self.ID = {
            "right wheel": 5,
            "left wheel": 6,
            "motor sensor": 8}

        self.port = 3
        self.sensor_scale = float(15.0/2350)
        self.all_sides = ["left", "front", "right"]

        # for clarification purposes, not used
        self.directions = {0: "South", 1: "West", 2: "North", 3: "East"}
        self.position = [0, 0, 0]
        self.width = 11.2 # cm
        self.map = EECSMap()
        self.grid_time = 3 # at speed = 900, takes 3 sec to travel 30 cm

        self.maze = EECSMap()
        self.maze.clearObstacleMap()
        self.maze_list = defaultdict(list)
        self.convert_direction = {
            0: 3,
            1: 4,
            2: 1,
            3: 2
        }

        self.maze_visited = {(0,0)}





    def default_config(self):
        setMotorMode(self.ID["right wheel"], 1)
        setMotorMode(self.ID["left wheel"], 1)
        setMotorTargetSpeed(self.ID["motor sensor"], 300)
        self.move_to(512)

    def stop_wheels(self):
        ids = [self.ID["right wheel"], self.ID["left wheel"]]
        vals = [0, 0]
        setMotorWheelSpeedSync(ids, vals)

    def move_backward(self, speed):
        ids = [self.ID["right wheel"], self.ID["left wheel"]]
        vals = [speed, 1023 + speed + 30]

        setMotorWheelSpeedSync(ids, vals)

    def move_forward(self, speed):
        ids = [self.ID["right wheel"], self.ID["left wheel"]]
        vals = [1023 + speed + 25, speed]

        setMotorWheelSpeedSync(ids, vals)

    
    def move_forward_by(self, speed, grid):
        self.move_forward(900)
        time.sleep(grid * 2.1)
        self.stop_wheels()        


    

    def move_to_grid(self, start, end): 
        # given a start and end grid, move there
        start_x, end_x = start[0], end[0]
        start_y, end_y = start[1], end[1]

        dx, dy = (end_x - start_x), (end_y - start_y)

        if (dx == -1):
            # move in x direction by 1  
            if (self.position[2] == 2):
                self.left_turn(90, 900)
            elif (self.position[2] == 0):
                self.right_turn(90, 900)

            self.move_forward_by(900, 1)
        
        elif (dx == 1):
            if (self.position[2] == 0):
                self.left_turn(90, 900)
            elif (self.position[2] == 2):
                self.right_turn(90, 900)

            self.move_forward_by(900, 1)

        elif (dy == -1):
            if (self.position[2] == 3):
                self.left_turn(90, 900)
            elif (self.position[2] == 1):
                self.right_turn(90, 900)
            self.move_forward_by(900, 1)
        
        elif (dy == 1):
            if (self.position[2] == 1):
                self.left_turn(90, 900)
            elif (self.position[2] == 3):
                self.right_turn(90, 900)
            self.move_forward_by(900, 1)




    def change_heading(self, direction):
        # change heading for 90 deg turns only
        if (direction == "left"):
            # if facing south, decrement by 1 (wrap around) to face east
            if (self.position[2] == 0):
                self.position[2] = 3
            else:
                self.position[2] -= 1

        if (direction == "right"):
            # if facing east, increment by 1 (wrap around) to face south
            if (self.position[2] == 3):
                self.position[2] = 0
            else:
                self.position[2] += 1            


    
    def right_turn(self, turn_amount, speed):
        self.stop_wheels()
        turn_amount = turn_amount * 3.1415926535897932384626433832795/180
        speed_scale = float(10)/float(900) # convert to cm/s
        r = self.width/2
        error_scale = float(90)/float(152)
        t = turn_amount * r / (float(speed) * speed_scale) * error_scale
        print(t)
        ids = [self.ID["right wheel"], self.ID["left wheel"]]
        vals = [speed, speed]
        setMotorWheelSpeedSync(ids, vals)
        time.sleep(t)
        self.stop_wheels()

        # change postion vector
        self.change_heading("right")

    def left_turn(self, turn_amount, speed):
        self.stop_wheels()
        turn_amount = turn_amount * 3.1415926535897932384626433832795/180
        speed_scale = float(10)/float(900) # convert to cm/s
        r = self.width/2
        error_scale = float(90)/float(152)
        t = turn_amount * r / (float(speed) * speed_scale) * error_scale
        print(t)
        ids = [self.ID["left wheel"], self.ID["right wheel"]]
        vals = [1023 + speed, 1023 + speed]
        setMotorWheelSpeedSync(ids, vals)
        time.sleep(t)
        self.stop_wheels()

        # change postion vector
        self.change_heading("left")

    def move_to(self, position): 
        motor_ID = self.ID["motor sensor"]
        setMotorTargetPositionCommand(motor_ID, position)
    

    def create_adjacency_list(self):
        horizontal = self.map.horizontalWalls
        vertical = self.map.verticalWalls

        def is_valid(i, j):
            return 0 <= i < 8 and 0 <= j < 8

        adjacency_list = defaultdict(list)

        for i in range(8):
            for j in range(8):
                pos = (i, j)
                if (is_valid(i, j+1)):
                    if (vertical[i][j+1] == 0):
                        adjacency_list[pos].append((i, j+1))
                        adjacency_list[(i, j+1)].append(pos)

        for i in range(8):
            for j in range(8):
                pos = (i, j)
                if (is_valid(i+1, j)):
                    if (horizontal[i+1][j] == 0):
                        adjacency_list[pos].append((i+1, j))
                        adjacency_list[(i+1, j)].append(pos)
        
        self.adjacency_list = adjacency_list

    def get_reading(self):
        self.move_to(820)
        time.sleep(1)
        left = 0
        for i in range(10):
            left += getSensorValue(self.port)
        self.move_to(512)
        time.sleep(1)
        front = 0
        for i in range(10):
            front += getSensorValue(self.port)
        self.move_to(200)
        time.sleep(1)
        right = 0
        for i in range(10):
            right += getSensorValue(self.port)
        self.move_to(512)
        return (left//10, front//10, right//10)



    def wander(self):

        next_pos = None

        THRESHOLD = 5
        
        curr_x, curr_y, heading = self.position[0], self.position[1], self.position[2]

        readings = self.get_reading()
        dir_left = heading - 1
        dir_front = heading
        dir_right = heading + 1

        # left, front, right
        dirs = [dir_left, dir_front, dir_right]
        grids = None

        if heading == 0:
            grids = [(curr_x + 1, curr_y), (curr_x, curr_y + 1), (curr_x - 1, curr_y)]
        elif heading == 1:
            grids = [(curr_x, curr_y - 1), (curr_x + 1, curr_y), (curr_x, curr_y + 1)]
        elif heading == 2:
            grids = [(curr_x - 1, curr_y ), (curr_x, curr_y - 1), (curr_x + 1, curr_y)]
        elif heading == 3:
            grids = [(curr_x, curr_y + 1), (curr_x - 1, curr_y), (curr_x, curr_y - 1)]


        def is_valid(i, j):
            return 0 <= i < 8 and 0 <= j < 8


        for i in range(len(dirs)):
            if (dirs[i] < 0):
                dirs[i] = 3
            elif (dirs[i] > 3):
                dirs[i] = 0
        
        for i in range(len(dirs)):
            dirs[i] = self.convert_direction[dirs[i]]

        
        for i in range(3):
            print(readings[i])
            print(readings[i] * 15 / 2350)
            reading, direction, grid = readings[i], dirs[i], grids[i]
            if (reading * 15 / 2350 < THRESHOLD):
                if (is_valid(grid[0], grid[1])):
                    # we will always pick the first available position to go to as the next position
                    if not (next_pos):
                        if (next_pos not in self.maze_visited):
                            next_pos = grid
            else:
                print("set grid to blocked")
                print("{}, {}".format(grid[0], grid[1]))
                if (is_valid(grid[0], grid[1])):
                    self.maze.setObstacle(grid[1], grid[0] + 1, True, direction)

        # need to account for cases where we can't move to an adjacent cell
        if not next_pos:
            print("could not find next position")
            pass
        
        return next_pos
    
        









    # def nearest_wall(self):
    #     shortest = 12232
    #     for i in self.all_sides:
    #         reading = self.get_reading(i)
    #         if 


        
    def wall(self):
        THRESHOLD = 3 # cm

        def check_error():

            # read sensor
            get_reading(self, side)

            # # reactive control

            # if ((sensor_reading * 15.0/145 - THRESHOLD) > 1):
            #     other_wall_sensor_reading = getSensorValue(other_wall_sensor)
            #     # all sides blocked
            #     if ((other_wall_sensor_reading * other_scale - THRESHOLD) > 1 and (error > 1)):
            #         self.left_turn_180()
            #         print("All sides blocked, turning 180!")
            #     # wall_sensor blocked
            #     elif (error > 1):
            #         print("2 sides blocked, turning 90!")
            #         if (wall_side == "right"):
            #             self.left_turn_90()
            #         else:
            #             self.right_turn_90()
            #     # other sensor blocked
            #     elif ((other_wall_sensor_reading * other_scale - THRESHOLD) > 1):
            #         print("2 sides blocked, turning 90!")
            #         if (wall_side == "left"):
            #             self.left_turn_90()
            #         else:
            #             self.right_turn_90()

        




    def bfs(self, start, end):
        # right now only going to worry about (x, y) and not heading
        # start and end will be triplets but only using x, y for now
        start_node, end_node = (start[0], start[1]), (end[0], end[1])
        print(start_node)
        print(end_node)
        seen = {start_node}

        queue = deque([(start_node, [])])
        while queue:
            node, curr_path = queue.popleft()

            if (node == end_node):
                return curr_path + [node]

            for neighbor in self.adjacency_list[node]:
                if (neighbor not in seen):
                    seen.add(neighbor)
                    queue.append((neighbor, curr_path + [node]))
        
        return None

    def test1(self):
        self.move_forward_by(900, 1)
        self.move_forward_by(900, 1)
        self.move_forward_by(900, 1)

    def test2(self):
        self.left_turn(90, 900)
        self.move_forward_by(900, 1)
        self.move_forward_by(900, 1)
        self.move_forward_by(900, 1)

    def test3(self):
        self.move_forward_by(900, 1)
        self.move_forward_by(900, 1)
        self.left_turn(90, 900)
        self.move_forward_by(900, 1)
        self.move_forward_by(900, 1)

    

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")
    robot = Robot()
    robot.default_config()
    #robot.map.printObstacleMap()
    # robot.map.printCostMap()
    robot.create_adjacency_list()
    # print(robot.bfs([0,0,0], [7,5,0]))
    #print(robot.bfs([0,0,0], [4,7,0]))

    #robot.move_forward(900)
    #robot.move_backward(400)
    #time.sleep(10)
    #robot.test1()
    #robot.stop_wheels()
    #robot.get_reading("front")
    #robot.get_reading("left")
    # robot.get_reading("front")
    # time.sleep(2)
    # robot.get_reading("left")
    #robot.test1()
# code
    # start = list(input("Enter start (no space comma separated:)"))
    # print(start)
    # start[0], start[1] = start[1], start[0]
    # robot.position = start

    # end = list(input("Enter end (no space comma separated:)"))
    # end[0], end[1] = end[1], end[0]

    # path = robot.bfs(start, end)
    # for i in range(len(path)):
    #     path[i] = list(reversed(path[i]))
    # print(path)
#code
    # for i in range(len(path) - 1):
    #     start_coord, end_coord = path[i], path[i+1]
        
    #     if (start_coord[0][])
    #     # horizontal compress
    #     for j in range(i + 1, len(path))
    #         if (path[j][0] != start_coord[0]):
    #             break
    #         else:
    #             end_coord = path[j]
    #     start_coord, end_coord = path[i], path[i+1]

    #     print("moving from: " + str(start_coord) + " to " + str(end_coord))
    #     robot.move_to_grid(start_coord, end_coord)
    #     robot.stop_wheels()

#code
    # for i in range(len(path) - 1):
    #     start_coord, end_coord = path[i], path[i+1]

    #     print("moving from: " + str(start_coord) + " to " + str(end_coord))
    #     print(robot.position[2])
    #     robot.move_to_grid(start_coord, end_coord)
    #     robot.stop_wheels()
    #     print("finished moving to: " + str(end_coord))

    #print(robot.get_reading())
    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    curr_pos = (0,0)
    while not rospy.is_shutdown():
        # call function to get sensor value

        next_pos = robot.wander()
        print("going to move to")
        print(next_pos)
        # next_pos = next_pos[::-1]
        robot.maze_visited.add(next_pos)
        robot.move_to_grid(curr_pos, next_pos)
        curr_pos = next_pos
        robot.position[0] = curr_pos[0]
        robot.position[1] = curr_pos[1]
        robot.maze.printObstacleMap()
        # sleep to enforce loop rate
        r.sleep()
