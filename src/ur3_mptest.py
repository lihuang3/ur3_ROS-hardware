#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, time, numpy as np
import roslib; roslib.load_manifest('ur_driver')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import geometry_msgs.msg

import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from time import sleep
tracker = Tracker()
client = None



class ur3_mp:
    def __init__(self):

        try:
            inp = raw_input("Continue? y/n: ")[0]
            if (inp == 'y'):
                rospy.init_node("ur3_mp", anonymous=True, disable_signals=True)
                self.client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
                print "Waiting for server..."
                self.client.wait_for_server()
                print "Connected to server"
                print "Please make sure that your robot can move freely between these poses before proceeding!"

                self.default_pose_execute()
                self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
                self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)


            else:
                print "Halting program"
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def default_pose_execute(self):
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        self.phase = 1
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 320.0
        self.cy = 240.0
        self.points=[]

        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.3)
        self.arm.set_max_velocity_scaling_factor(.35)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = -0.1123
        wpose.position.y = -0.4111
        wpose.position.z = 0.2168
        wpose.orientation.x = -0.7068
        wpose.orientation.y = 0.0090
        wpose.orientation.z = 0.7073
        wpose.orientation.w = 0.0086

        self.waypoints.append(deepcopy(wpose))


        # self.arm.set_pose_target(wpose)


        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()
        Q0 = [1.5713225603103638, -2.0962370077716272, -1.3499930540667933, -1.265561882649557, 1.57, 1.596241985951559]
        Q = [2.700089454650879, -1.6774938742267054, -1.9622791449176233, -1.0725882689105433, 1.5723614692687988, 1.693281888961792]

        Q1 = [2.6995975971221924, -1.960052792225973, -2.1507018248187464, -0.601605240498678, 1.57, 1.694779311810629]

        self.default_joint_states = Q0

        # self.default_joint_states[0] = Q0[0]
        # self.default_joint_states[1] = Q0[1]
        # self.default_joint_states[2] = Q0[2]
        # self.default_joint_states[3] = Q0[3]
        # self.default_joint_states[4] = Q0[4]
        # self.default_joint_states[5] = Q0[5]

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()
        sleep(2)
        self.arm.execute(plan)

        self.transition_states = deepcopy(self.default_joint_states)
        self.transition_states = Q



        # Specify end states (drop object)
        self.end_joint_states = deepcopy(self.default_joint_states)
        self.end_joint_states = Q1
        # self.end_joint_states[1] = -1.3705


    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def tracking_callback(self, msg):

        self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        if len(self.pointx)>9:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1

        if (self.track_flag ):
            self.execute()
            self.default_pose_flag = False
        else:
            if not self.default_pose_flag:
                self.track_flag = False
                self.execute()
                self.default_pose_flag = True



    def execute(self):
        if self.track_flag:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # wpose.position.x = -0.5215
            # wpose.position.y = 0.2014
            # wpose.position.z = 0.4102


            # if len(self.pointx)>8:
            #     if len(self.pointx)==9:
            #         x_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
            #         wpose.position.x += 2 * x_speed
            #         wpose.position.z = 0.05
            #
            #
            #     else:
            #         if len(self.pointx)==11:
            #             tracker.flag2 = 1
            #             self.cxy_pub.publish(tracker)
            #
            #         if len(self.pointx)<12:
            #             x_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
            #             wpose.position.x += (x_speed-self.error_x*0.015/105)
            #
            #         else:
            #             if tracker.flag2:
            #                 self.track_flag=False
            #             transition_pose = deepcopy(start_pose)
            #             transition_pose.position.z = 0.4000
            #
            #             self.waypoints.append(deepcopy(transition_pose))
            #
            #             self.arm.set_start_state_to_current_state()
            #             plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
            #             self.arm.execute(plan)
            #
            #             self.arm.set_max_acceleration_scaling_factor(.15)
            #             self.arm.set_max_velocity_scaling_factor(.25)
            #
            #
            #
            #             self.arm.set_joint_value_target(self.transition_pose)
            #             self.arm.set_start_state_to_current_state()
            #             plan = self.arm.plan()
            #             self.arm.execute(plan)
            #
            #             self.arm.set_joint_value_target(self.end_joint_states)
            #             self.arm.set_start_state_to_current_state()
            #             plan = self.arm.plan()
            #             self.arm.execute(plan)
            #
            #             if -0.1+0.02*self.object_cnt<0.2:
            #                 self.object_cnt += 1
            #
            #             self.waypoints = []
            #             start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            #             transition_pose = deepcopy(start_pose)
            #             transition_pose.position.x -= 0.1
            #             transition_pose.position.z = -0.1 + self.object_cnt*0.025
            #             self.waypoints.append(deepcopy(transition_pose))
            #
            #             self.arm.set_start_state_to_current_state()
            #             plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
            #             self.arm.execute(plan)
            #
            #             self.phase = 2
            #             tracker.flag2 = 0
            #             self.cxy_pub.publish(tracker)
            #
            #
            #
            # # Set the next waypoint to the right 0.5 meters
            # else:

            wpose.position.x -= self.error_x*0.025/105
            wpose.position.y += self.error_y*0.015/105
            print wpose.position.x
            print wpose.position.y
            self.waypoints.append(deepcopy(wpose))
            # wpose.position.z = 0.15
                #wpose.position.z = 0.4005

            # if self.phase == 1:
            #     self.waypoints.append(deepcopy(wpose))
            #
            #
            #     self.pointx.append(wpose.position.x)
            #     self.pointy.append(wpose.position.y)
            #
            #     # Set the internal state to the current state
            #     # self.arm.set_pose_target(wpose)
            #
            #     self.arm.set_start_state_to_current_state()
            #
            # Plan the Cartesian path connecting the waypoints

            """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
                    self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

               Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
               poses specified as waypoints. Configurations are computed for every eef_step meters;
               The jump_threshold specifies the maximum distance in configuration space between consecutive points
               in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
               the actual RobotTrajectory.

            """
            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
            print fraction
            print self.error_x*0.025/105, self.error_y*0.015/105
            # plan = self.arm.plan()
            self.phase = 1
            # If we have a complete plan, execute the trajectory
            if 1-fraction < 0.2 and not (abs(self.error_x*0.025/105)<0.005 and abs(self.error_y*0.015/105)<0.002):
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            elif (abs(self.error_x*0.025/105)<0.005 and abs(self.error_y*0.015/105)<0.002):
                self.phase = 2
            else:
                rospy.loginfo("Path planning failed")

            if self.phase ==2:
                self.track_flag=False
                start_pose = self.arm.get_current_pose(self.end_effector_link).pose

                wpose = deepcopy(start_pose)
                wpose.position.z = 0.1
                self.waypoints = []
                self.waypoints.append(deepcopy(wpose))

                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
                self.arm.execute(plan)

                tracker.flag2 = 1
                self.cxy_pub.publish(tracker)

                sleep(0.2)

                start_pose = self.arm.get_current_pose(self.end_effector_link).pose

                wpose = deepcopy(start_pose)
                wpose.position.z = 0.06
                self.waypoints = []
                self.waypoints.append(deepcopy(wpose))

                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
                self.arm.execute(plan)

                start_pose = self.arm.get_current_pose(self.end_effector_link).pose

                sleep(0.2)

                wpose = deepcopy(start_pose)
                wpose.position.z = 0.036    # 0.035
                self.waypoints = []
                self.waypoints.append(deepcopy(wpose))

                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.005, 0.0, True)
                self.arm.execute(plan)

                sleep(0.5)
                start_pose = self.arm.get_current_pose(self.end_effector_link).pose

                wpose = deepcopy(start_pose)
                wpose.position.z = 0.2168
                self.waypoints = []
                self.waypoints.append(deepcopy(wpose))

                self.arm.set_start_state_to_current_state()
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.005, 0.0, True)
                self.arm.execute(plan)



                self.arm.set_joint_value_target(self.transition_states)
                self.arm.set_start_state_to_current_state()
                plan = self.arm.plan()
                self.arm.execute(plan)

                sleep(0.5)

                self.arm.set_joint_value_target(self.end_joint_states)
                self.arm.set_start_state_to_current_state()
                plan = self.arm.plan()
                self.arm.execute(plan)


                sleep(1)

                tracker.flag2 = 0
                self.cxy_pub.publish(tracker)
                sleep(1)

                self.arm.set_joint_value_target(self.transition_states)
                self.arm.set_start_state_to_current_state()
                plan = self.arm.plan()
                self.arm.execute(plan)


        else:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []
            self.pointx = []
            self.pointy = []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # Set the next waypoint to the right 0.5 meters

            wpose.position.x = -0.1123
            wpose.position.y = -0.4111
            wpose.position.z = 0.2168
            wpose.orientation.x = -0.7068
            wpose.orientation.y = 0.0090
            wpose.orientation.z = 0.7073
            wpose.orientation.w = 0.0086

            self.pointx.append(wpose.position.x)
            self.pointy.append(wpose.position.y)
            self.waypoints.append(deepcopy(wpose))
            # Set the internal state to the current state
            # self.arm.set_pose_target(wpose)

            self.arm.set_start_state_to_current_state()

            # Plan the Cartesian path connecting the waypoints

            """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
                    self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

               Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
               poses specified as waypoints. Configurations are computed for every eef_step meters;
               The jump_threshold specifies the maximum distance in configuration space between consecutive points
               in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
               the actual RobotTrajectory.

            """
            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


            # plan = self.arm.plan()

            # If we have a complete plan, execute the trajectory
            if 1-fraction < 0.2:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed")
        # print self.points




mp=ur3_mp()

rospy.spin()
