#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import json
import math
from DroneBlocksTelloSimulator import SimulatedDrone

class PID:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
    
    def update(self, error):
        self.error = error
        self.integral = self.integral + error
        self.derivative = error - self.prev_error
        self.prev_error = error
        return self.P * self.error + self.I * self.integral + self.D * self.derivative
   
class TurtleController:
	def __init__(self):
		self.pub_cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.sub_pose = rospy.Subscriber('/turtle1/pose', Pose, self.onPose)
		self.subTasks = rospy.Subscriber('/turtle_command', String, self.droneTasks)
		self.pose = Pose()
		
		self.anglePID = PID(1, 0, 0)
		self.distancePID = PID(1, 0, 0)

	def onPose(self, data):
		self.pose = data
		if self.pose.theta < 0:
			self.pose.theta += 2*math.pi

	def droneTasks(self, msg):
		if (len(msg.data) == 0):
			return
		tasks = self.parseJson(msg.data)
		print(tasks)
		drone = SimulatedDrone('e9437905-893a-4d91-8bf1-0f449fb36b1a')
		for task in tasks:
			if hasattr(drone, task['action']) and callable(getattr(drone, task['action'])):
				method_to_call = getattr(drone, task['action'])
				method_to_call(*task['params'])
	
	def onTasks(self, msg):
		if (len(msg.data) == 0):
			return
		tasks = self.parseJson(msg.data)
		print(tasks)
		for task in tasks:
			if hasattr(self, task['action']) and callable(getattr(self, task['action'])):
				method_to_call = getattr(self, task['action'])
				method_to_call(task['params'])

	def parseJson(self, inp_string):
		inp_string = inp_string.replace('\'', '"')
		inp_json = json.loads(inp_string)
		return inp_json

	def move(self, args):
		rospy.loginfo(f"Turtle Moving at speed of {args['speed']} in the {args['direction']} direction for {args['distance']} units")
		vel = Twist()
		t0 = rospy.Time.now().to_sec()
		dist_traveled = 0
		while (dist_traveled < float(args['distance'])):
			vel.linear.x = float(args['speed']) * pow(-1, args['direction'] == 'backward')
			self.pub_cmd_vel.publish(vel)
			t1 = rospy.Time.now().to_sec()
			dist_traveled = (t1 - t0) * float(args['speed'])
		vel.linear.x = 0
		self.pub_cmd_vel.publish(vel)

	def rotate(self, args):
		rospy.loginfo(f"Turtle Rotating at angular speed of {args['angular_speed']} in the {args['direction']} direction for {args['angle']} radians")
		vel = Twist()
		t0 = rospy.Time.now().to_sec()
		angleTraveled = 0
		vel.angular.z = float(args['angular_speed']) * pow(-1, args['direction'] == 'clockwise')
		while (angleTraveled < float(args['angle'])):
			self.pub_cmd_vel.publish(vel)
			t1 = rospy.Time.now().to_sec()
			angleTraveled = (t1 - t0) * float(args['angular_speed'])
		vel.angular.z = 0
		self.pub_cmd_vel.publish(vel)

	def rotateByTheta(self, args):
		if 'direction' not in args :
			args['direction'] = 'anticlockwise'
		rospy.loginfo(f"Turtle Rotating by theta {args['theta']} {args['direction']}")
		goal_theta = self.pose.theta + float(args['theta'])*math.pow(-1, args['direction']=='cloclwise')
		vel = Twist()
		angle_distance = goal_theta - self.pose.theta
		while abs(angle_distance) >= 0.05:
			angle_distance = goal_theta - self.pose.theta
			vel.angular.z = self.anglePID.update(angle_distance)
			self.pub_cmd_vel.publish(vel)
		vel.angular.z = 0
		self.pub_cmd_vel.publish(vel)
		
	def moveByDistance(self, args):
		rospy.loginfo(f"Turtle Moving by {args['distance']}")
		goal_x = self.pose.x + float(args['distance']) * math.cos(self.pose.theta)
		goal_y = self.pose.y + float(args['distance']) * math.sin(self.pose.theta)
		vel = Twist()
		distance = math.sqrt(math.pow(goal_x - self.pose.x, 2) + math.pow(goal_y - self.pose.y, 2))
		while abs(distance) >= 0.05:
			distance = math.sqrt(math.pow(goal_x - self.pose.x, 2) + math.pow(goal_y - self.pose.y, 2))
			vel.linear.x = self.distancePID.update(distance)
			self.pub_cmd_vel.publish(vel)
		vel.linear.x = 0
		self.pub_cmd_vel.publish(vel)

	def goToGoal(self, args):
		rospy.loginfo(f"Turtle Moving to x:{args['goal_x']} y:{args['goal_y']} theta:{args['goal_theta']}")
		angle = math.atan2(args['goal_y'] - self.pose.y, args['goal_x'] - self.pose.x) - self.pose.theta
		self.rotateByTheta({ 'theta': angle, 'direction': 'clockwise' })
		distance = math.sqrt(math.pow(args['goal_x'] - self.pose.x, 2) + math.pow(args['goal_y'] - self.pose.y, 2))
		self.moveByDistance({ 'distance': distance })
		angle = args['goal_theta'] - self.pose.theta
		self.rotateByTheta({ 'theta': angle, 'direction': 'clockwise'})
		
		
def main():
    TurtleController()
    rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim', anonymous=True)
		main()
	except rospy.ROSInterruptException:
		pass