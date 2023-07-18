import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class RRTPlanner:
    def __init__(self):
        # Set up ROS node and publishers/subscribers
        rospy.init_node('rrt_planner')
        self.traj_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.joint1_pub = rospy.Publisher('/arm_joint_1_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/arm_joint_2_controller/command', Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher('/arm_joint_3_controller/command', Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher('/arm_joint_4_controller/command', Float64, queue_size=10)
        self.joint5_pub = rospy.Publisher('/arm_joint_5_controller/command', Float64, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.current_angles = np.zeros(5)

        # Set up RRT parameters
        self.start_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.goal_angles = np.array([1.57, 0.0, 0.0, 0.0, 0.0])
        self.joint_limits = [(np.radians(-169.0), np.radians(169.0)),
                             (np.radians(-65.0), np.radians(90.0)),
                             (np.radians(-151.0), np.radians(-34.0)),
                             (np.radians(-102.5), np.radians(102.5)),
                             (np.radians(-165.0), np.radians(165.0))]
        self.step_size = 0.1
        self.goal_tolerance = 0.1

    def run_rrt(self):
        # Initialize the tree with the start node
        tree = [self.start_angles]

        # Grow the tree until the goal is reached
        while not np.allclose(tree[-1], self.goal_angles, atol=self.goal_tolerance):
            # Sample a random configuration
            q_rand = np.array([np.random.uniform(lim[0], lim[1]) for lim in self.joint_limits])

            # Find the nearest node in the tree to the random configuration
            distances = np.linalg.norm(np.array(tree) - q_rand, axis=1)
            q_near = tree[np.argmin(distances)]

            # Move from the nearest node towards the random configuration
            q_new = q_near + self.step_size * (q_rand - q_near) / np.linalg.norm(q_rand - q_near)

            # Check if the new node is valid (i.e. within joint limits and collision-free)
            if not self.is_valid(q_new):
                continue

            # Add the new node to the tree
            tree.append(q_new)

        # Generate the path
        path = []
        q_end = tree
# ... Code for RRTPlanner class up until generating the path ...

    def generate_path(self, goal_node):
        # Backtrack from goal node to find path
        path = []
        current_node = goal_node
        while current_node is not None:
            path.append(current_node)
            current_node = current_node.parent
        path = path[::-1]

        # Convert path to joint trajectory
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        for node in path:
            point = JointTrajectoryPoint()
            point.positions = node.state
            point.time_from_start = rospy.Duration(0.5)  # Constant time between points for simplicity
            traj.points.append(point)

        return traj

    def execute_trajectory(self, traj):
        # Publish trajectory and wait for execution
        self.traj_pub.publish(traj)
        rospy.sleep(traj.points[-1].time_from_start)
        rospy.loginfo("Trajectory executed")

if __name__ == '__main__':
    planner = RRTPlanner()
    start = [0, 0, 0]
    goal = [1.5, 0.5, 0.5]
    goal_tol = 0.1
    max_iter = 5000

    # Run RRT to find path from start to goal
    goal_node = planner.rrt(start, goal, goal_tol, max_iter)
    if goal_node is not None:
        rospy.loginfo("Path found!")
        traj = planner.generate_path(goal_node)
        planner.execute_trajectory(traj)
    else:
        rospy.loginfo("Could not find path")
