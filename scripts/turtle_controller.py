#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class TurtleController:
	def __init__(self):
		self.pub_cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.subTasks = rospy.Subscriber('/turtle_command', String, self.onTasks)

	def onTasks(self, msg):
		if (len(msg.data) == 0):
			return
		tasks = self.parseJson(msg.data)
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


def main():
    TurtleController()
    rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('turtlesim', anonymous=True)
		main()
	except rospy.ROSInterruptException:
		pass