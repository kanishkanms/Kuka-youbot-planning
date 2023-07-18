import rospy, math
# Class for a square 2D obstacle
# Make 4 corners and 4 midpoints (called control points (ctrpnt)
class obstacle:
	def __init__(self, c_x, c_y, w):
		self.c_x = c_x # center x
		self.c_y = c_y # center y
		self.w = w # width
		self.ctrpnt = [ [ 0 for i in range(2) ] for j in range(8) ]
		
		self.ctrpnt[0][0] = c_x + w/2.0
		self.ctrpnt[0][1] = c_y + w/2.0

		self.ctrpnt[1][0] = c_x + w/2.0
		self.ctrpnt[1][1] = c_y

		self.ctrpnt[2][0] = c_x + w/2.0
		self.ctrpnt[2][1]= c_y - w/2.0

		self.ctrpnt[3][0] = c_x
		self.ctrpnt[3][1] = c_y - w/2.0

		self.ctrpnt[4][0] = c_x - w/2.0
		self.ctrpnt[4][1] = c_y - w/2.0

		self.ctrpnt[5][0] = c_x - w/2.0
		self.ctrpnt[5][1] = c_y

		self.ctrpnt[6][0] = c_x - w/2.0
		self.ctrpnt[6][1] = c_y + w/2.0

		self.ctrpnt[7][0] = c_x
		self.ctrpnt[7][1] = c_y + w/2.0


# Input corners to make wall
class wall:
	def __init__(self, cn1, cn2): # corners are each (x,y)
		self.cn1 = [0 for i in range(2)]
		self.cn2 = [0 for i in range(2)]
		self.cn1[0] = cn1[0]
		self.cn1[1] = cn1[1]
		self.cn2[0] = cn2[0]
		self.cn2[1] = cn2[1]

# Function for checking collision between Youbot and obstacle
# Return distance between two closest control points
def obstacle_dist(youbot,obs):
	idx_obs = 0 # set index for chosen control point
	idx_youbot = 0 # set index for chosen control point
	dist = math.sqrt((youbot.ctrpnt[0][0] - obs.ctrpnt[0][0])**2+(youbot.ctrpnt[0][1] - obs.ctrpnt[0][1])**2)
	for i in range(8): # loop through obstacle control points
		for j in range(8): # loop through youbot control points
			temp_dist = math.sqrt((youbot.ctrpnt[j][0] - obs.ctrpnt[i][0])**2+(youbot.ctrpnt[j][1] - obs.ctrpnt[i][1])**2)
			if temp_dist < dist:
				dist = temp_dist
				idx_obs = i
				idx_youbot = j

	# return closest distance and x, y coordinates of obstacle point
	return dist, youbot.ctrpnt[idx_youbot][0], youbot.ctrpnt[idx_youbot][1], obs.ctrpnt[idx_obs][0], obs.ctrpnt[idx_obs][1]


def wall_dist(youbot,wall):
	if wall.cn1[0]-wall.cn2[0] == 0:
		# vertical wall, distance is difference in x coordinates
		idx = 0 # set index for chosen control point
		dist = abs(youbot.ctrpnt[0][0] - wall.cn1[0])
		for j in range(8): # loop through youbot control points
			temp_dist = abs(youbot.ctrpnt[j][0] - wall.cn1[0])
			if temp_dist < dist:
				dist = temp_dist
				idx = j
		
		# return the closest control points of youbot, and the point on the wall closest to it
		return dist, youbot.ctrpnt[idx][0], youbot.ctrpnt[idx][1], wall.cn1[0], youbot.ctrpnt[idx][1]
		

	if wall.cn1[1]-wall.cn2[1] == 0:
		# horizontal wall, distance is difference in x coordinates
		idx = 0 # set index for chosen control point
		dist = abs(youbot.ctrpnt[0][1] - wall.cn1[1])
		for j in range(8): # loop through youbot control points
			temp_dist = abs(youbot.ctrpnt[j][1] - wall.cn1[1])
			if temp_dist < dist:
				dist = temp_dist
				idx = j
		
		# return the closest control points of youbot, and the point on the wall closest to it
		return dist, youbot.ctrpnt[idx][0], youbot.ctrpnt[idx][1], youbot.ctrpnt[idx][0], wall.cn1[1]



