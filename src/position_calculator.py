#!/usr/bin/env python  
#----------------------------------
# runs on the hub gateway
# receives all of the position reports for all of the robots
# calculates the relative position to each robot of all of the 
# nearby robots
#----------------------------------
import rospy
import geometry_msgs.msg as gm
from math import sqrt

NUM_AGENTS = 2
RADIUS = 10 #m

class Agent:
    def __init__(self, name, xi, yi):
        self.name = name
        self.xi = xi#the initial coordinates of the robot in the global frame
        self.yi = yi
        self.pose = gm.Pose()
        incoming_topic_name = '/' + name + '/adjusted_pose'
        self.sub = rospy.Subscriber(incoming_topic_name, gm.Pose, lambda d: self.record_pose(d))
        poses_topic_name = '/' + name + '/other_agent_poses'
        quadrant_topic_name = '/' + name + '/quadrant_values'
        self.poses_pub = rospy.Publisher(poses_topic_name, gm.PoseArray, queue_size = 10)
        self.quad_pub = rospy.Publisher(quadrant_topic_name, gm.Quaternion, queue_size = 10)

    def record_pose(self, data):
        data.position.x += self.xi#get them into a global reference frame
        data.position.y += self.yi

        self.pose = data

    def publish(self, poses, quads):
        self.poses_pub.publish(poses)
        self.quad_pub.publish(quads)

def run():
    rospy.init_node('position_calculator')
    agent_list = [Agent('aadi8', 0, 0), Agent('aadi9', 0, -1)]
    rospy.sleep(2)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        for agent in agent_list:
            rospy.loginfo('Agent %s is checking its surroundings'%agent.name)
            a_x = agent.pose.position.x
            a_y = agent.pose.position.y
            nearby = gm.PoseArray();
            nearby.header.stamp = rospy.Time.now()
            nearby.header.frame_id = '1'
            nearby.poses = []
            quadrants = gm.Quaternion(0, 0, 0, 0)#just using a quaternion to hold four values
            #x|y
            #---
            #w|z
            for other in agent_list:
                if other != agent:
                    rospy.loginfo('It found another agent, %s'%other.name)

                    o_x = other.pose.position.x #o_x, o_y, a_x, a_y are all in the global frame
                    o_y = other.pose.position.y
                    xdist = o_x - a_x
                    ydist = o_y - a_y
                    hdist = sqrt(xdist**2 + ydist**2)
                    if hdist < RADIUS:
                        #now get the other's position into frame centered on (a_x, a_y)
                        relative = gm.Pose()
                        relative.orientation = other.pose.orientation;#not even gonna try and do position right
                        relative.position.x = xdist
                        relative.position.y = ydist
                        nearby.poses.append(relative)
                        rospy.loginfo(nearby)
                        inverse_dist = 1/hdist
                        if xdist > 0 and ydist >= 0: 
                            print 'in q1'
                            quadrants.x += inverse_dist
                        elif xdist >= 0 and ydist < 0:
                            print 'in q2'
                            quadrants.y += inverse_dist
                        elif xdist < 0 and ydist <= 0:
                            print 'in q3'
                            quadrants.z += inverse_dist
                        else:
                            print 'in q4'
                            quadrants.w += inverse_dist


            agent.publish(nearby, quadrants);

        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass

