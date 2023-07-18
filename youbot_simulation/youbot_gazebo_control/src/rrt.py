#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import random
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RRTPlanner:
    def __init__(self):
        # Define the joint limits for the YouBot arm
        #self.joint_limits = np.array([[0.0100692, 5.84014], [-2.61799, 2.61799], [-2.70526, 0.707216], [-3.4292, 3.4292],[0.00872665, 5.02455]])
        self.joint_limits = [(np.radians(-169.0), np.radians(169.0)),
                             (np.radians(-65.0), np.radians(90.0)),
                             (np.radians(-151.0), np.radians(146)),
                             (np.radians(-102.5), np.radians(102.5)),
                             (np.radians(-165.0), np.radians(165.0))]
        
        # Define the start and goal joint angles
        self.start_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.goal_angles = np.array([0, 0.0, -2.63, 0.6, 0.0])

        # Initialize the node
        rospy.init_node('rrt_planner')

    
        # Set up a subscriber for the current joint angles
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        # Set up a publisher for the trajectory
        self.traj_pub = rospy.Publisher('/arm_1/arm_controller/command', JointTrajectory, queue_size=10)

        # Wait for the subscribers to connect
        rospy.sleep(1.0)

        # Run the RRT algorithm
        reached = self.run_rrt()
        print(reached)


    def random_state(self,bias_goal,goal_state,goal_radius):
        if random.random()<bias_goal:
            return np.random.normal(goal_state,goal_radius)
        else:
            return np.random.uniform(low=np.array(self.joint_limits)[:,0], high=np.array(self.joint_limits)[:,1])

    def run_rrt(self):
        # Initialize the tree with the start position
        tree = [self.start_angles]

        # Define the number of iterations and the step size
        num_iter = 2000
        step_size = 0.1
        goal_radius = 0.1
        bias_goal = 0.2
        done = 0
        t = 0
        # Perform the RRT algorithm
        for i in range(num_iter):
            # Sample a random point in the joint space
            q_rand = self.random_state(bias_goal,self.goal_angles,goal_radius)
            # print(q_rand)
            # print(np.array(tree))

            # Find the closest node in the tree
            distances = np.linalg.norm(np.array(tree) - q_rand, axis=1)
            q_near = tree[np.argmin(distances)]
            #print(q_near)

            # Generate a new point in the direction of q_rand
            q_new = q_near + step_size * (q_rand - q_near) / np.linalg.norm(q_rand - q_near)
            #print(q_new)
            #print("min",np.array(self.joint_limits)[:,0])
            #print("max",np.array(self.joint_limits)[:,1])
            # Check if the new point is valid
            t+=1
            if self.is_valid(q_new):
                # Add the new point to the tree
                #print('a')
                tree.append(q_new)
                #print(np.linalg.norm(q_new - self.goal_angles))
                # Check if the goal has been reached
                if np.linalg.norm(q_new - self.goal_angles) < 0.1:
                    done= 1
                    print('Goal reached!')
                    break
        
        if done ==0:
            print("not found")
            return 0
        # Generate the path
        print(t)
        path = []
        q_end = tree[-1]
        print(q_end)
        path.append(q_end)
        while not np.array_equal(q_end, self.start_angles):
            distances = np.linalg.norm(np.array(tree) - q_end, axis=1)
            q_near = tree[np.argmin(distances)]
            tree.pop(np.argmin(distances))
            path.append(q_near)
            q_end = q_near
        path = path[::-1]

        # Print the path
        print('Path: ')
        print(path)

        # Publish the path as a trajectory
        joint_traj = JointTrajectory()
        joint_traj.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
        for i in range(len(path)):
            #print('a')
            point = JointTrajectoryPoint()
            point.positions = path[i]
            point.time_from_start = rospy.Duration(0.1*i)
            joint_traj.points.append(point)

        self.traj_pub.publish(joint_traj)
        return 1

    def is_valid(self, q):
        # Check if the joint angles are within the limits
        if np.any(q < np.array(self.joint_limits)[:,0]) or np.any(q > np.array(self.joint_limits)[:,1]):
            #print("min_test",np.any(q < np.array(self.joint_limits)[:,0]))
            #print("max_test",np.any(q < np.array(self.joint_limits)[:,1]))
            return False

        # TODO: Add collision checking here

        return True

    def joint_callback(self, msg):
        # Update the current joint angles
        self.current_angles = np.array(msg.position[:5])

if __name__ == '__main__':
    planner = RRTPlanner()


    
