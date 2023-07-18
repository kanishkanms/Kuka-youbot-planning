#!/usr/bin/env python

"""
@author: KANISHKAN M S
"""
import rospy, math
import multiprocessing
from nav_msgs.msg import Odometry
#from obstacle import obstacle, wall, obstacle_dist, wall_dist
from geometry_msgs.msg import Twist
#from rrt import run_forward_arm_planning,run_backward_arm_planning,reset
from manipulator_planner import pick_planner,drop_planner
from mobile_planner import run_mobile_planning


#path=[[3,3],[3,3.5],[3,2],[3,1],[3,0],[2,0],[2,-1],[1,-1],[2,-2],[1,-3],[1,-4]]
path=run_mobile_planning()
#path=[[3.5, 4.5], [3.0, 4.5], [2.5, 4.5], [2.0, 4.5], [1.5, 4.5], [1.0, 4.5], [0.5, 4.5], [0.0, 4.5], [-0.5, 4.5], [-1.0, 4.5], [-1.5, 4.0], [-2.0, 4.0], [-2.5, 4.0], [-3.0, 4.0], 'Pickup', [-2.5, 3.5], [-2.5, 3.0], [-2.5, 2.5], [-2.5, 2.0], [-2.5, 1.5], [-2.5, 1.0], [-2.0, 0.5], [-1.5, 0.0], [-1.0, 0.0], [-0.5, 0.0], [0.0, 0.0], 'Pickup', [0.5, -0.5], [1.0, -1.0], [1.5, -1.0], [2.0, -1.0], [2.5, -1.0], [3.0, -1.0], [3.5, -1.0], [4.0, -1.0], 'Pickup', [3.5, -1.0], [3.0, -1.5], [2.5, -2.0], [2.5, -2.5], [2.0, -3.0], [1.5, -3.5], [1.0, -4.0], 'Drop', [0.5, -3.5], [0.0, -3.5], [-0.5, -3.0], 'Pickup', [0.0, -3.5], [0.5, -3.5], [1.0, -4.0], 'Drop']
seq = 0
x_pos = 0
y_pos = 0
z_pos = 0.0
w = 0.0
x_vel = 0.0
#self.y_vel = 0.0
z_ang = 0.0

x_goal=3
y_goal=3



def odomCallback(msg):

    global x_pos
    global y_pos
    seq = msg.header.seq
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    z_pos = msg.pose.pose.position.z
    w = msg.pose.pose.orientation.w
    x_vel = msg.twist.twist.linear.x
    y_vel = msg.twist.twist.linear.y
    z_ang = msg.twist.twist.angular.z
    #print(x_pos,y_pos)


rospy.init_node('youbot_node')
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1000)
sub = rospy.Subscriber('/odom', Odometry, odomCallback)
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1000)

loop_rate = rospy.Rate(20)

def set_sign(x):
    if x > 0:
        return 0.9
    elif x < 0:
        return -0.9
    else:
        return 0

def stay_still(vel):
    counter=0
    while(counter<100):
        vel.linear.x = 0
        vel.linear.y = 0
        loop_rate.sleep()
        counter+=1


count=1
box=1
#while not rospy.is_shutdown():
vel=Twist()
init_state=[4,4]
drop=0
for i in range(len(path)):

    if(path[i]=="Pickup"):
        if(drop==1):
            box=4
        #planner=run_forward_arm_planning(count)
        t1 = multiprocessing.Process(target=pick_planner(count,box))
        t2 = multiprocessing.Process(target=stay_still(vel))

        # start both threads
        t1.start()
        t2.start()

        # join both threads to wait for them to finish
        t1.join()
        t2.join()

        x_pos_calc=(path[i-1][0]-init_state[0])
        y_pos_calc=path[i-1][1]-init_state[1]
        print(x_pos_calc,y_pos_calc)
        print(x_pos,y_pos)

        
        #reset()
        box+=1
        count+=1
    elif(path[i]=="Drop"):
        count=3
        box-=1
        if(drop==1):
            box=4
        #planner=run_forward_arm_planning(count)
        t1 = multiprocessing.Process(target=drop_planner(count,box))
        t2 = multiprocessing.Process(target=stay_still(vel))

        # start both threads
        t1.start()
        t2.start()

        # join both threads to wait for them to finish
        t1.join()
        t2.join()

        count=1

        drop=1
        
        #reset()

        
    elif(path[i]!="Pickup" and path[i]!="Drop"):
        loop_rate.sleep()
        x_pos_calc=(path[i][0]-init_state[0])
        y_pos_calc=path[i][1]-init_state[1]
        action_x=x_pos_calc
        action_y=y_pos_calc
        if(i>=1):
            if(path[i-1]=="Pickup" or path[i-1]=="Drop"):
                action_x=path[i][0]-path[i-2][0]
                action_y=path[i][1]-path[i-2][1]
            else:
                action_x=path[i][0]-path[i-1][0]
                action_y=path[i][1]-path[i-1][1]
        print(x_pos_calc,y_pos_calc)

        action_x=set_sign(action_x)
        action_y=set_sign(action_y)
        #while(x_pos>x_pos_calc and y_pos>y_pos_calc):
        while(math.sqrt(pow((x_pos-x_pos_calc),2)+pow((y_pos-y_pos_calc),2))>0.2):
            #print("Error:",error)
            print(x_pos,x_pos_calc,y_pos,y_pos_calc)
            vel.linear.x = action_x
            vel.linear.y = action_y
            print("Actions:",action_x,action_y)
            pub.publish(vel)
            loop_rate.sleep()
        else:
            vel.linear.x=0
            vel.linear.y=0
            pub.publish(vel)
            loop_rate.sleep()
# print("ENDDDDDD")
# count=0


#planner=run_arm_planning()
        
    #vel=Twist()
print(x_pos,y_pos)
    # if(x_pos>-0.85 and y_pos>-0.85):
    #     vel.linear.x = -1
    #     vel.linear.y = -1
    # else:
    #     vel.linear.x = 0
    #     vel.linear.y = 0
    #     reset_odom(Odometry())

    
    # pub.publish(vel)
    # loop_rate.sleep()


