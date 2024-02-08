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

        self.port = 4
        self.sensor_scale = float(15.0/2350)

        self.position = [0, 0, 0]
        self.width = 11.2 # cm
        self.map = EECSMap()
        self.grid_time = 3 # at speed = 900, takes 3 sec to travel 30 cm

        self.sensor_side = "right"



    def default_config(self):
        setMotorMode(self.ID["right wheel"], 1)
        setMotorMode(self.ID["left wheel"], 1)
        setMotorTargetSpeed(self.ID["motor sensor"], 300)

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

    def move_to(self, i, j, k): 
        pass

    
    def right_turn(self, turn_amount, speed):
        self.stop_wheels()
        turn_amount = turn_amount * 3.1415926535897932384626433832795/180
        speed_scale = float(10)/float(900) # convert to cm/s
        r = self.width/2
        error_scale = float(90)/float(148)
        t = turn_amount * r / (float(speed) * speed_scale) * error_scale
        print(t)
        ids = [self.ID["right wheel"], self.ID["left wheel"]]
        vals = [speed, speed]
        setMotorWheelSpeedSync(ids, vals)
        time.sleep(t)
        self.stop_wheels()

    def left_turn(self, turn_amount, speed):
        self.stop_wheels()
        turn_amount = turn_amount * 3.1415926535897932384626433832795/180
        speed_scale = float(10)/float(900) # convert to cm/s
        r = self.width/2
        error_scale = float(90)/float(148)
        t = turn_amount * r / (float(speed) * speed_scale) * error_scale
        print(t)
        ids = [self.ID["left wheel"], self.ID["right wheel"]]
        vals = [1023 + speed, 1023 + speed]
        setMotorWheelSpeedSync(ids, vals)
        time.sleep(t)
        self.stop_wheels()

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

    def get_reading(self, side):
        self.sensor_side = side
        if (side == "front"):
            self.move_to(0)
        elif (side == "right"):
            self.move_to(520)
        else:
            self.move_to(1023)

        
    # def wall(self):
    #     THRESHOLD = 3 # cm

    #     def check_error():

    #         # read sensor
    #         # sensor_reading = getSensorValue(self.port)
    #         # print(wall_sensor_reading)
    #         # error = wall_sensor_reading * scale - THRESHOLD # cm
    #         # print(error)

    #         # # reactive control

    #         # if ((sensor_reading * 15.0/145 - THRESHOLD) > 1):
    #         #     other_wall_sensor_reading = getSensorValue(other_wall_sensor)
    #         #     # all sides blocked
    #         #     if ((other_wall_sensor_reading * other_scale - THRESHOLD) > 1 and (error > 1)):
    #         #         self.left_turn_180()
    #         #         print("All sides blocked, turning 180!")
    #         #     # wall_sensor blocked
    #         #     elif (error > 1):
    #         #         print("2 sides blocked, turning 90!")
    #         #         if (wall_side == "right"):
    #         #             self.left_turn_90()
    #         #         else:
    #         #             self.right_turn_90()
    #         #     # other sensor blocked
    #         #     elif ((other_wall_sensor_reading * other_scale - THRESHOLD) > 1):
    #         #         print("2 sides blocked, turning 90!")
    #         #         if (wall_side == "left"):
    #         #             self.left_turn_90()
    #         #         else:
    #         #             self.right_turn_90()

        




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
        self.move_forward(900)
        time.sleep(9)
        self.stop_wheels()




    

# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")
    robot = Robot()
    robot.default_config()
    # robot.map.printObstacleMap()
    # robot.map.printCostMap()
    robot.create_adjacency_list()
    # print(robot.bfs([0,0,0], [7,5,0]))
    print(robot.bfs([0,0,0], [4,7,0]))

    #robot.move_forward(900)
    #robot.move_backward(400)
    #time.sleep(10)
    #robot.test1()
    #robot.stop_wheels()
    #robot.get_reading("front")
    #robot.get_reading("left")
    robot.get_reading("front")
    time.sleep(2)
    robot.get_reading("left")

    # control loop running at 10hz
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # call function to get sensor value
        port = 5
        sensor_reading = getSensorValue(port)
        rospy.loginfo("Sensor value at port %d: %f", 5, sensor_reading)


        # sleep to enforce loop rate
        r.sleep()
