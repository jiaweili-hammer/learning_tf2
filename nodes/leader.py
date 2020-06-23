#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions
from math import atan2, radians, pow, sqrt, pi
import tf2_ros
import geometry_msgs.msg
from turtlesim.msg import Pose



class TurtleBot:
	def __init__(self):
		rospy.init_node('sucker_broadcaster', anonymous = True)
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size = 10)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.handle_pose)
		self.pose = Pose()
		self.rate = rospy.Rate(10)
		self.br = tf2_ros.TransformBroadcaster()
	
	def handle_pose(self,data):
		
		self.pose = data
		self.pose.x = round(self.pose.x,4)
		self.pose.y = round(self.pose.y,4)
		t = geometry_msgs.msg.TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "world"
		t.child_frame_id = "turtle1"
		t.transform.translation.x = self.pose.x
		t.transform.translation.y = self.pose.y
		t.transform.translation.z = 0.0
		q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.pose.theta)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
		self.br.sendTransform(t)
	
		
	
	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.x-self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))
	
	def linear_vel(self, goal_pose, constant = 1.5):
		return constant * self.euclidean_distance(goal_pose)
	
	def steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
	
	def angular_vel(self, goal_pose, constant = 4):
		return constant * (self.steering_angle(goal_pose) - self.pose.theta)
	
	def move(self,speed,distance,isForward):
		vel_msg = geometry_msgs.msg.Twist()
		if isForward:
			vel_msg.linear.x = abs(speed)
		else:
			vel_msg.linear.x = -abs(speed)
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while current_distance < distance:
			self.velocity_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_distance = speed*(t1-t0)
			self.rate.sleep()
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)
		#rospy.spin()
	
	def rotate(self, angular_speed_degree, desired_angle_degree, clockwise):
		vel_msg = geometry_msgs.msg.Twist()
		angular_speed = angular_speed_degree * 2 * pi / 360
		desired_angle = desired_angle_degree * 2 * pi / 360
		vel_msg.linear.x = 0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		if clockwise:
			vel_msg.angular.z = -abs(angular_speed)
		else:
			vel_msg.angular.z = abs(angular_speed)
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
		while current_angle < desired_angle:
			self.velocity_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1-t0)
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
		#rospy.spin()
	
	def set_abs_rotation(self, desired_angle):
		relative_angle = radians(desired_angle) - self.pose.theta
		if relative_angle < 0:
			isClockwise = 0
		else:
			isClockwise = 1;
		relative_angle = relative_angle /2/pi*360
		self.rotate(abs(relative_angle)/4, abs(relative_angle), isClockwise)
	
	def move2goal(self,goal_pose,tolerance):
		#goal_pose = Pose()
		vel_msg = geometry_msgs.msg.Twist()
		while self.euclidean_distance(goal_pose) >= tolerance:
			vel_msg.linear.x = self.linear_vel(goal_pose)
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel(goal_pose)
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)
	
	def grid_clean(self):
		
		goal_pose = Pose(1,1,0,0,0)
		self.move2goal(goal_pose,0.01)
		
		self.set_abs_rotation(0)
		
		self.move(2,9,1)
		self.rotate(10,90,0)
		
		i = 0
		while i < 5:
			i = i+1
			self.move(3,9,1)
			self.rotate(30,90,0)
			self.move(3,1,1)
			self.rotate(30,90,0)
			self.move(3,9,1)
			self.rotate(30,90,1)
			self.move(3,1,1)
			self.rotate(30,90,1)
			self.move(3,9,1)

	def spiral_clean(self):
		vel_msg = geometry_msgs.msg.Twist()
		const_speed = 2
		spiral = 0.5
		while self.pose.x and self.pose.y < 10.5:
			spiral = spiral + 0.05
			vel_msg.linear.x = spiral
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = const_speed
			self.velocity_publisher.publish(vel_msg)
			self.rate.sleep()
		vel_msg.linear.x = 0
		self.velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
	try:
		x = TurtleBot()
		menu = input("1 for grid clean\n2 for spiral clean\nenter:")
		if menu == 1:
			x.grid_clean()
		else:
			x.spiral_clean()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()
