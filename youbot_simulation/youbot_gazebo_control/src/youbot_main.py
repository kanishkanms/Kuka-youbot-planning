#!/usr/bin/env python

import rospy, math
from nav_msgs.msg import Odometry
from obstacle import obstacle, wall, obstacle_dist, wall_dist
from geometry_msgs.msg import Twist

class youbot_class:
	
	def __init__(self):
		self.seq = 0
		self.x_pos = 0.0
		self.y_pos = 0.0
		self.z_pos = 0.0
		self.w = 0.0
		self.x_vel = 0.0
		#self.y_vel = 0.0
		self.z_ang = 0.0

		rospy.init_node('youbot_node')
		pub = rospy.Publisher('cmd_vel',Twist,queue_size=1000)
		sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
		
		self.x_goal = 0.0; self.y_goal = 0.0
		
		# Switching range for attractive potential
		self.d_star = 1.0

		# Distance region of interest for repulsive potential
		self.num_obs = 4 # number of obstacles
		self.num_wall = 4 # number of walls

		self.Q_star = [0 for i in range(self.num_obs+self.num_wall)]
		rep_obs = 0.8
		rep_wall = 0.8
		self.Q_star[0] = rep_obs # obstacles
		self.Q_star[1] = rep_obs
		self.Q_star[2] = rep_obs
		self.Q_star[3] = rep_obs
		self.Q_star[4] = rep_wall # walls
		self.Q_star[5] = rep_wall
		self.Q_star[6] = rep_wall
		self.Q_star[7] = rep_wall

		# width of youbot
		self.w_you = 0.1
		# Attractive gradient gain
		self.zeta = -1.0
		# Repuslive gradient gain
		self.eta = -0.5

		# Initialize obstacles
		self.obs_list = []
		self.obs_list.append(obstacle(2.5,2.5,1.0))
		self.obs_list.append(obstacle(5.0,1.0,0.75))
		self.obs_list.append(obstacle(3.0,5.0,0.5))
		self.obs_list.append(obstacle(0.0,3.0,0.25))

		# Initialize walls
		self.wall_list = []
		self.wall_list.append(wall([-1.0,6.0],[6.0,6.0]))
		self.wall_list.append(wall([6.0,6.0],[6.0,-1.0]))
		self.wall_list.append(wall([6.0,-1.0],[-1.0,-1.0]))
		self.wall_list.append(wall([-1.0,-1.0],[-1.0,6.0]))
		
		loop_rate = rospy.Rate(5)

		
		self.x_goal = float(input('Enter input goal x coordinate: '))
		self.y_goal = float(input('Enter input goal y coordinate: '))
		
		while not rospy.is_shutdown():
			# User input goal
			#if (self.x_pos - self.x_goal) < 0.01 and (self.y_pos - self.y_goal) < 0.01:
				#self.x_goal = float(input('Enter input goal x coordinate: '))
				#self.y_goal = float(input('Enter input goal y coordinate: '))

			# Compute potential field gradient of obstacles
			gx_rep,gy_rep = self.repulsive_gradient()

			# Compute potential field gradient of goal
			gx_att,gy_att = self.attractive_gradient()
			
			dx = gx_rep + gx_att
			dy = gy_rep + gy_att
			# Move youbot based on gradient
			vel=Twist()
			vel.linear.x = dx
			vel.linear.y = dy
			print("X-velocity:"+str(dx),"X-velocity:"+str(dy))
			pub.publish(vel)

			loop_rate.sleep()

			#print 'current position:', self.x_pos, self.y_pos
			#print 'goal position:', self.x_goal, self.y_goal
			#print 'repulsive gradient:', gx_rep, gy_rep


	# Callback function stores subscribed variables into class
	def odomCallback(self,msg):
		self.seq = msg.header.seq
		self.x_pos = msg.pose.pose.position.x
		self.y_pos = msg.pose.pose.position.y
		self.z_pos = msg.pose.pose.position.z
		self.w = msg.pose.pose.orientation.w
		self.x_vel = msg.twist.twist.linear.x
		self.y_vel = msg.twist.twist.linear.y
		self.z_ang = msg.twist.twist.angular.z

	# Gradient of the potiential of obstacles
	def repulsive_gradient(self):
		youbot = obstacle(self.x_pos,self.y_pos,self.w_you)

		dist = [0 for i in range(self.num_obs+self.num_wall)]
		x_grad_dist = [0 for i in range(self.num_obs+self.num_wall)]
		y_grad_dist = [0 for i in range(self.num_obs+self.num_wall)]

		x_obs = [0 for i in range(self.num_obs)]
		y_obs = [0 for i in range(self.num_obs)]

		x_wall = [0 for i in range(self.num_wall)]
		y_wall = [0 for i in range(self.num_wall)]

		x_youbot = [0 for i in range(self.num_obs+self.num_wall)]
		y_youbot = [0 for i in range(self.num_obs+self.num_wall)]

		# determine distance between nearest point on youbot and obstacle
		for i in range(0,self.num_obs):
			dist[i],x_youbot[i],y_youbot[i],x_obs[i],y_obs[i] = obstacle_dist(youbot,self.obs_list[i])
			x_grad_dist[i] = (x_youbot[i]-x_obs[i])/dist[i]
			y_grad_dist[i] = (y_youbot[i]-y_obs[i])/dist[i]

		# determine distance between nearest point on youbot and wall
		for i in range(self.num_obs,self.num_obs+self.num_wall):
			dist[i],x_youbot[i],y_youbot[i],x_wall[i-self.num_obs],y_wall[i-self.num_obs] = wall_dist(youbot,self.wall_list[i-self.num_obs])
			x_grad_dist[i] = (x_youbot[i]-x_wall[i-self.num_obs])/dist[i]
			y_grad_dist[i] = (y_youbot[i]-y_wall[i-self.num_obs])/dist[i]

		
		# initialize gradient
		dqx = 0.0; dqy = 0.0
		# generate sum of potential gradients
		for i in range(0,self.num_obs+self.num_wall):
			if dist[i] <= self.Q_star[i]:
				#dqx += self.eta*(1.0/self.Q_star[i]-1.0/dist[i])*(1/(dist[i]**2))*(x_youbot[i]-x_obs[i])
				#dqy += self.eta*(1.0/self.Q_star[i]-1.0/dist[i])*(1/(dist[i]**2))*(y_youbot[i]-y_obs[i])
				dqx += self.eta*(1.0/self.Q_star[i]-1.0/dist[i])*(1.0/(dist[i]**2))*x_grad_dist[i]
				dqy += self.eta*(1.0/self.Q_star[i]-1.0/dist[i])*(1.0/(dist[i]**2))*y_grad_dist[i]
		return dqx,dqy
		
	

	# Gradient of the potiential of goal
	def attractive_gradient(self):
		qx = self.x_pos-self.x_goal
		qy = self.y_pos-self.y_goal
		dist = math.sqrt((self.x_pos-self.x_goal)**2 + (self.y_pos-self.y_goal)**2)
		if dist <= self.d_star:
			# Quadratic gradient
			return self.zeta*qx,self.zeta*qy
		elif dist > self.d_star:
			# Conical gradient
			return self.d_star*self.zeta*qx/dist, self.d_star*self.zeta*qy/dist

if __name__ == '__main__':
	try:
		temp = youbot_class()
	except rospy.ROSInterruptException: 
		pass






