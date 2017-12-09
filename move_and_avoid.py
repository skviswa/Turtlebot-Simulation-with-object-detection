#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
#from go_to_specific_point_on_map import GoToPose
import time
from std_msgs.msg import Int8
from std_msgs.msg import String

global yolo_state, cancel_flag 
global x, y
x = 0
y = 0
yolo_state = False
cancel_flag = False

dest = {

    'position': {
            'x': 0,
            'y': 0
            },
    'quaternion': {
            'r1': 0,
            'r2': 0,
            'r3': -0.628,
            'r4': 0.778
            }
}

def init_goal(g, pos, quat):
    print('Goal reinitialized')
    g.target_pose.header.frame_id = 'map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
    return g


class GoToPos():
    def __init__(self):

        self.goal_sent = False

    	# What to do if shut down (e.g. Ctrl-C or failure)
    	rospy.on_shutdown(self.shutdown)
    
    	# Tell the action client that we want to spin a thread by default
    	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    	rospy.loginfo("Wait for the action server to come up")

    	# Allow up to 5 seconds for the action server to come up
    	self.move_base.wait_for_server(rospy.Duration(5))
	
    
    def goto_pos(self, pos, quat):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal = init_goal(goal, pos, quat)

        # Start moving
        self.move_base.send_goal(goal)
        #t0 = datetime.utcnow().timestamp()
        cancel_flag = False
        state = self.move_base.get_state()
        while(state != GoalStatus.SUCCEEDED):
 	    state = self.move_base.get_state()
            yolo_state = check_yolo()
	    print(yolo_state)
	   # print("Cancel Flag" + str(cancel_flag))
            if yolo_state is True:
		#self.move_base.send_goal()
                self.move_base.cancel_goal()
                cancel_flag = True
            else:#if cancel_flag is True and yolo_state is False:
		#goal1 = MoveBaseGoal()
		#goal1 = init_goal(goal1, pos, quat)
                self.move_base.send_goal(goal)
                #cancel_flag = False
            time.sleep(3)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


def check_yolo():
    global yolo_state
    msg = rospy.wait_for_message('/found_object', Int8)
    #print(type(msg))
    #print(msg)
    msg = str(msg)
    #print(type(msg1))
    #print(msg1)
    if msg == "data: 1":
        yolo_state = True
	print("Object Detected")
    else:
        yolo_state = False
    return yolo_state

def update_x_y():
    global x
    global y
    rospy.init_node('loc_mon', anonymous = True)
    msg = rospy.wait_for_message("/odom", Odometry)
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo('x: {}, y: {}'.format(x,y))

def main():
    print('Started Main')
    update_x_y()

    nav = GoToPos()
    name = "Checkpoint 1"

    rospy.loginfo("Go to %s pose", name[:-4])
    success = nav.goto_pos(dest['position'], dest['quaternion'])
    if not success:
            rospy.loginfo("Failed to reach %s pose", name[:-4])
    rospy.loginfo("Reached %s pose", name[:-4])
    update_x_y()
    
if __name__ == '__main__':
    main()
