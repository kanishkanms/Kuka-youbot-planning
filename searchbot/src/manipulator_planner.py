
#!/usr/bin/env python3
"""
@author: NIKHIL S
"""
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import random
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse



start_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
pick_state = np.array([-0.1, -0.42, -2.8, 0.4, -0.1])
goal_state_1 = np.array([-3.1, 0.1, -2.8, +0.1, 0.0])
goal_state_3 = np.array([2.9, 0.05, -2.8, +0.1, 0.0])
goal_state_2 = np.array([3, -0.02, -2.5, 0.1, 0.0])
inter_state_2 = np.array([3,0,0,0,0])
inter_state_1 = np.array([-3.1, 0.0, 0, +0.0, 0.0])
inter_state_3  =np.array([2.9, 0.00, 0, +0.0, 0.0])

states = {1:[inter_state_1,goal_state_1],2:[inter_state_2,goal_state_2],3:[inter_state_3,goal_state_3]}


joint_limits = [(np.radians(-180.0), np.radians(180.0)),
                             (np.radians(-65.0), np.radians(90.0)),
                             (np.radians(-170.0), np.radians(151)),
                             (np.radians(-102.5), np.radians(102.5)),
                             (np.radians(-167.5), np.radians(167.5))]

class RRTNode:
    def __init__(self,config):
        self.config = config
        self.parent = None
        self.cost = 0

iter_array = []


def attach(box):
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "youbot"
    req.link_name_1 = "gripper_finger_link_l"
    req.model_name_2 = "box"+str(box)
    req.link_name_2 = "link"
    attach_srv.call(req)

def detach(box):
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Attaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "youbot"
    req.link_name_1 = "gripper_finger_link_l"
    req.model_name_2 = "box"+str(box)
    req.link_name_2 = "link"
    attach_srv.call(req)

def random_state(bias_goal,goal_state,goal_radius):
    if random.random()<bias_goal:
        return np.random.normal(goal_state,goal_radius)
    else:
        return np.random.uniform(low=np.array(joint_limits)[:,0], high=np.array(joint_limits)[:,1])

def add_node(config,parent,cost,node_tree):
    node = RRTNode(config)
    node.parent = parent
    node.cost = cost
    node_tree.append(node)
    #return node

def nearest_config(config,config_tree):
    distances = np.linalg.norm(np.array(config_tree) - config, axis=1)
    q_near = config_tree[np.argmin(distances)]
    return q_near

def new_config(config,nearest,step_size):
    distance = np.linalg.norm(config - nearest)
    if distance < step_size:
        new_config = config
    else:
        direction = (config - nearest) / distance
        new_config = nearest + step_size * direction
    return new_config

def publish_path(path):
    joint_traj = JointTrajectory()
    joint_traj.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
    for i in range(len(path)):
        #print('a')
        point = JointTrajectoryPoint()
        point.positions = path[i]
        point.time_from_start = rospy.Duration(0.1*i)
        joint_traj.points.append(point)
    global traj_pub
    traj_pub.publish(joint_traj)

def reset(goal):
    joint_traj = JointTrajectory()
    joint_traj.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        #print('a')
    point = JointTrajectoryPoint()
    point.positions = goal
    point.time_from_start = rospy.Duration(2)
    joint_traj.points.append(point)
    global traj_pub
    traj_pub.publish(joint_traj)

def is_valid(q):
        # Check if the joint angles are within the limits
        if np.any(q < np.array(joint_limits)[:,0]) or np.any(q > np.array(joint_limits)[:,1]):
            return False

        # TODO: Add collision checking here

        return True
def find_node(q,node_tree):
    for node in node_tree:
        if (node.config==q).all():
            return node
def path_to_root(node):
    path = [node.config]
    while node.parent!=None:
        path.append(node.parent.config)
        node = node.parent
    path.append(node.config)
    return path

def pick_planner(t,box):

    #rospy.init_node('rrt_planner')
    global traj_pub
    traj_pub = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(2)
    
    start_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    pick_state = np.array([-0.1, -0.46, -2.8, 0.4, -0.1])
    goal_state_1 = np.array([-3.1, 0.3, -2.8, +0.1, 0.0])
    goal_state_3 = np.array([2.9, 0.05, -2.8, +0.1, 0.0])
    goal_state_2 = np.array([3, -0.02, -2.5, 0.1, 0.0])
    inter_state_2 = np.array([3,0,0,0,0])
    inter_state_1 = np.array([-3.1, 0.0, 0, +0.0, 0.0])
    inter_state_3  =np.array([2.9, 0.00, 0, +0.0, 0.0])
    
    global states
    reset([0,0,0,0,0])
    rospy.sleep(2)
    gripper_open()
    rospy.sleep(2)
    gripper_open()
    rospy.sleep(2)
    planner  = RRT_starPlanner(start_state,pick_state,0.5,500)
    planner.run_rrt()
    rospy.sleep(2)
    gripper_close()
    rospy.sleep(2)
    gripper_close()
    rospy.sleep(2)
    attach(box)
    planner  = RRT_starPlanner(pick_state,states[t][0],0.5,500)
    planner.run_rrt()
    rospy.sleep(2)
    # reset(states[t][1])
    # rospy.sleep(2)
    planner  = RRT_starPlanner(states[t][0],states[t][1],0.5,500)
    planner.run_rrt()
    rospy.sleep(2)
    gripper_open()
    rospy.sleep(2)
    detach(box)
    reset(states[t][0])
    rospy.sleep(2)
    reset([0,0,0,0,0])


def drop_planner(t,box):

    #rospy.init_node('rrt_planner')
    global traj_pub
    traj_pub = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(2)
    
    start_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    pick_state = np.array([-0.1, -0.46, -2.8, 0.4, -0.1])
    goal_state_1 = np.array([-3.1, 0.1, -2.8, +0.1, 0.0])
    goal_state_3 = np.array([2.9, 0.05, -2.8, +0.1, 0.0])
    goal_state_2 = np.array([3, -0.02, -2.5, 0.1, 0.0])
    inter_state_2 = np.array([3,0,0,0,0])
    inter_state_1 = np.array([-3.1, 0.0, 0, +0.0, 0.0])
    inter_state_3  =np.array([2.9, 0.00, 0, +0.0, 0.0])
    global states

    for i in range(t):
        reset([0,0,0,0,0])
        rospy.sleep(2)
        gripper_open()
        rospy.sleep(2)
        gripper_open()
        rospy.sleep(2)
        planner  = RRT_starPlanner(start_state,states[i+1][0],0.5,500)
        planner.run_rrt()
        rospy.sleep(2)
        planner  = RRT_starPlanner(states[i+1][0],states[i+1][1],0.5,500)
        planner.run_rrt()
        rospy.sleep(2)
        gripper_close()
        rospy.sleep(2)
        #attach(box)
        # reset(states[i+1][0])
        # rospy.sleep(2)
        planner  = RRT_starPlanner(states[i+1][0],pick_state,0.5,500)
        planner.run_rrt()
        rospy.sleep(2)
        gripper_open()
        rospy.sleep(2)
        #detach(box)
        reset([0,0,0,0,0])





class RRTPlanner:
    def __init__(self,start_config,goal_config,step_size,max_iterations):
        # Define the joint limits for the YouBot arm
        #self.joint_limits = np.array([[0.0100692, 5.84014], [-2.61799, 2.61799], [-2.70526, 0.707216], [-3.4292, 3.4292],[0.00872665, 5.02455]])
        self.start_node = RRTNode(start_config)
        self.goal_node  = RRTNode(goal_config)
        self.step_size = step_size
        self.max_iter = max_iterations
        self.goal_radius = 0.1
        self.tree = [self.start_node]
        #self.run_rrt()
        

    def run_rrt(self):
        # Initialize the tree with the start position
        config_tree = [self.start_node.config]
        # Define the number of iterations and the step size
        
        bias_goal = 0.3
        done = 0
        t = 0
        # Perform the RRT algorithm
        for i in range(self.max_iter):
            # Sample a random point in the joint space
            q_rand = random_state(bias_goal,self.goal_node.config,self.goal_radius)
            # Find the closest node in the tree
            q_near = nearest_config(q_rand,config_tree)
            # Generate a new point in the direction of q_rand
            q_new = new_config(q_rand,q_near,self.step_size)
            
            if is_valid(q_new):
                # Add the new point to the tree
                config_tree.append(q_new)
                #print(self.tree,q_near)
                add_node(q_new,find_node(q_near,self.tree),0,self.tree)
                # Check if the goal has been reached
                if np.linalg.norm(q_new - self.goal_node.config) < 0.1:
                    done= 1
                    t=i
                    print('Goal reached!')
                    break
        
        if done ==0:
            print("not found")
            return 0
        # Generate the path
        path = []
        end_node = self.tree[-1]
        path = path_to_root(end_node)
        path = path[::-1]
        path.append(self.goal_node.config)
        print(path[-2])
        # Print the path
        #print('Path: ')
        #print(path)

        # Publish the path as a trajectory
        publish_path(path)
        return [t,len(path)]

    


class Bider_RRTPlanner:
    def __init__(self,start_config,goal_config,step_size,max_iterations):
        # Define the joint limits for the YouBot arm
        #self.joint_limits = np.array([[0.0100692, 5.84014], [-2.61799, 2.61799], [-2.70526, 0.707216], [-3.4292, 3.4292],[0.00872665, 5.02455]]
        self.start_node = RRTNode(start_config)
        self.goal_node  = RRTNode(goal_config)
        self.step_size = step_size
        self.max_iter = max_iterations
        self.goal_radius = 0.1
        self.tree = []
        self.start_tree = [self.start_node] 
        self.goal_tree = [self.goal_node]
        self.run_rrt()

    def run_rrt(self):
        
        # Define the number of iterations and the step size
        
        bias_goal = 0.3
        done = 0
        t = 0
        # Perform the RRT algorithm
        for i in range(self.max_iter):
            if i%2 == 0:

                # Sample a random point in the joint space
                q_rand = random_state(bias_goal,self.goal_node.config,self.goal_radius)
                # Find the closest node in the tree
                q_near = nearest_config(q_rand,np.array([node.config for node in self.start_tree]))
                # Generate a new point in the direction of q_rand
                q_new = new_config(q_rand,q_near,self.step_size)
                if is_valid(q_new):
                    add_node(q_new,find_node(q_near,self.start_tree),0,self.start_tree)
                    q_near_tree_2 = nearest_config(q_new,np.array([node.config for node in self.goal_tree]))
                    if np.linalg.norm(q_new -q_near_tree_2)<0.3:
                        done =1
                        print("goal_reached1")
                        t=i
                        break
            else :
                # Sample a random point in the joint space
                q_rand = random_state(bias_goal,self.start_node.config,self.goal_radius)
                # Find the closest node in the tree
                q_near = nearest_config(q_rand,np.array([node.config for node in self.goal_tree]))
                # Generate a new point in the direction of q_rand
                q_new = new_config(q_rand,q_near,self.step_size)
            
                if is_valid(q_new):
                    add_node(q_new,find_node(q_near,self.start_tree),0,self.goal_tree)
                    q_near_tree_1 = nearest_config(q_new,np.array([node.config for node in self.start_tree]))
                    if np.linalg.norm(q_new -q_near_tree_1)<0.3:
                        done =2
                        print("goal_reached_2")
                        t=i
                        break
        
        if done ==0:
            print("not found")
            return 0
        # Generate the path
        path_from_start = []
        path_to_goal = []
        if done ==1:
            end_node_start = self.start_tree[-1]
            path_from_start = path_to_root(end_node_start)
            path_from_start = path_from_start[::-1]
            end_node_goal = find_node(q_near_tree_2,self.goal_tree)
            path_to_goal = path_to_root(end_node_goal)
            
        if done == 2:
            end_node_goal = self.goal_tree[-1]
            path_to_goal = path_to_root(end_node_goal)
            end_node_start = find_node(q_near_tree_1,self.start_tree)
            path_from_start = path_to_root(end_node_start)
            path_from_start = path_from_start[::-1]

        #print(end_node_start.config)
        #print(path_from_start[-1])
        print(path_from_start+path_to_goal)
        print(t)

        publish_path(path_from_start+path_to_goal)
        return 1


class RRT_starPlanner:
    def __init__(self,start_config,goal_config,step_size,max_iterations):
        # Define the joint limits for the YouBot arm
        #self.joint_limits = np.array([[0.0100692, 5.84014], [-2.61799, 2.61799], [-2.70526, 0.707216], [-3.4292, 3.4292],[0.00872665, 5.02455]])
        self.start_node = RRTNode(start_config)
        self.goal_config  = goal_config
        self.step_size = step_size
        self.max_iter = max_iterations
        self.goal_radius = 0.1
        self.tree = []

    def run_rrt(self):
        # Initialize the tree with the start position
        self.tree = [self.start_node]
        # Define the number of iterations and the step size
        
        bias_goal = 0.3
        done = 0
        t = 0
        # Perform the RRT algorithm
        for i in range(self.max_iter):
            # Sample a random point in the joint space
            q_rand = random_state(bias_goal,self.goal_config,self.goal_radius)
            # Find the closest node in the tree
            q_near = nearest_config(q_rand,np.array([node.config for node in self.tree]))
            # Generate a new point in the direction of q_rand
            q_new = new_config(q_rand,q_near,self.step_size)
           
            if is_valid(q_new):
                near_nodes = self.find_near_nodes(q_new)
                cost_array = np.array([np.linalg.norm(q_new-node.config)+node.cost for node in near_nodes])
                add_node(q_new,near_nodes[np.argmin(cost_array)],cost_array[np.argmin(cost_array)],self.tree)
                self.rewire(find_node(q_new,self.tree),near_nodes)
                if np.linalg.norm(q_new-self.goal_config)<self.step_size:
                    done = 1 
                    print("goal_reached")
                    t=i
                    break
        if done ==0:
            print("not found")
            return 0
        # Generate the path
        path = []
        end_node = self.tree[-1]
        path = path_to_root(end_node)
        path = path[::-1]
        path.append(self.goal_config)

        # Print the path
        #print('Path: ')
        print(path[-2])

        # Publish the path as a trajectory
        publish_path(path)
        return [t,len(path)]
    
    def find_near_nodes(self,q_new):
        distances = [np.linalg.norm(q_new - n.config) for n in self.tree]
        radius = 10.0 * self.step_size * np.sqrt(np.log(len(self.tree)) / len(self.tree))
        if radius ==0:
            radius =10*self.step_size
        return [self.tree[i] for i in range(len(self.tree)) if distances[i] < radius]
    
    def rewire(self, q_new, near_nodes):
        for q_near in near_nodes:
            q_near_cost = q_new.cost + np.linalg.norm(q_new.config - q_near.config)
            if q_near_cost < q_near.cost:
                q_near.parent = q_new
                q_near.cost = q_near_cost
                self.propagate_cost_to_leaves(q_near)

    def propagate_cost_to_leaves(self, q):
        for n in self.tree:
            if n.parent == q:
                n.cost = q.cost + np.linalg.norm(n.config - q.config)
                self.propagate_cost_to_leaves(n)


def gripper_open():
    gripper_positions = [0.02,0.02]
    
    pub = rospy.Publisher('/arm_1/gripper_controller/command', JointTrajectory, queue_size=10)

    # Create a JointTrajectory message
    traj = JointTrajectory()
    traj.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']

    # Create a JointTrajectoryPoint message for the gripper positions
    point = JointTrajectoryPoint()
    point.positions = [0.015,0.015]
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)
    #print(traj)
    # Publish the gripper joint trajectory
    pub.publish(traj)
    #rospy.sleep(3)

def gripper_close():
    
    pub = rospy.Publisher('/arm_1/gripper_controller/command', JointTrajectory, queue_size=10)

    # Create a JointTrajectory message
    traj = JointTrajectory()
    traj.joint_names = ['gripper_finger_joint_l', 'gripper_finger_joint_r']

    # Create a JointTrajectoryPoint message for the gripper positions
    point = JointTrajectoryPoint()
    point.positions = [0.007,0.007]
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)
    #print(traj)

    # Publish the gripper joint trajectory
    pub.publish(traj)
    #rospy.sleep()
    

if __name__ == '__main__':
    
    rospy.init_node('rrt_planner')
    global traj_pub
    traj_pub = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(2)
    
    start_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    pick_state = np.array([-0.1, -0.46, -2.8, 0.4, -0.1])
    goal_state_1 = np.array([-3.1, 0.1, -2.8, +0.1, 0.0])
    goal_state_3 = np.array([2.9, 0.05, -2.8, +0.1, 0.0])
    goal_state_2 = np.array([3, -0.02, -2.5, 0.1, 0.0])
    inter_state_2 = np.array([3,0,0,0,0])
    inter_state_1 = np.array([-3.1, 0.0, 0, +0.0, 0.0])
    inter_state_3  =np.array([2.9, 0.00, 0, +0.0, 0.0])
 
    
    pick_planner(2)





    





    
