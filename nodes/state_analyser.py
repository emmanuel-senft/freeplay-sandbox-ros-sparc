#!/usr/bin/env python
import numpy as np
import numpy.ma as ma
from scipy import ndimage
import random
import signal
import sys

import math
import heapq

import rospy
import tf
from threading import Event, Lock, Timer
import actionlib


from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid
from freeplay_sandbox_msgs.msg import ListFloatStamped
from visualization_msgs.msg import MarkerArray, Marker
import shapely.geometry 

DEBUG = False

MAP_HEIGHT=0.335

REFERENCE_FRAME="/sandtray"

DISTANCE_THRESHOLD = 0.001
DIAGONAL = math.sqrt(0.6*0.6+.35*.35)
PHYSICAL_MAP_WIDTH = 0.6
PHYSICAL_MAP_HEIGHT = 0.335
RESOLUTION = 0.005 #m per cell

class StateAnalyser(object):
    def __init__(self):
        self._tl = tf.TransformListener()
        rospy.sleep(0.5) # sleep a bit to make sure the TF cache is filled

        self._event_sub = rospy.Subscriber("sandtray/interaction_events", String, self.on_event)
        self._life_sub = rospy.Subscriber("sparc/life", ListFloatStamped, self.on_life)
        self._map_sub = rospy.Subscriber("map", OccupancyGrid, self.on_map)

        self._state_pub = rospy.Publisher("sparc/state", ListFloatStamped, queue_size = 5)
        self._trigger_state_pub = rospy.Publisher("sparc/trigger_state", ListFloatStamped, queue_size = 5)
        self._event_pub = rospy.Publisher("sandtray/interaction_events", String, queue_size = 5)

        self._stopping = False

        self._state=np.array([],dtype = float)
        self._state_label=[]
        self._trigger_state=np.array([],dtype = float)
        self._trigger_state_label = []
        self._characters = []
        self._targets = []
        self._initialised = False
        self._life = []
        self._steps_no_touch = 0

        self._xmax = 0
        self._xmin = 0
        self._current_touches = 0
        self._robot_touch = False

        self._map = None

        rospy.loginfo("Ready to play!")
        self._timer = Timer(0.5, self.get_state)
        
        self.get_state()

    def get_state(self):
        if self._stopping:
            return
        self._timer = Timer(0.5, self.get_state)
        self._timer.start()
        
        if not self._initialised or len(self._life) == 0:
            self._event_pub.publish(String("analyser_ready"))
            return

        index = 0
        for idx, character in enumerate(self._characters):
            for other in (self._characters+self._targets)[idx+1:]:
                self._state[index]=self.get_distance_objects(character,other)/DIAGONAL
                index+=1
        for value in self._life:
            self._state[index] = value
            index+=1

        if len(self._state_label) == 0:
            for idx, character in enumerate(self._characters):
                for other in (self._characters+self._targets)[idx+1:]:
                    self._state_label.append("d_"+character+"_"+other)
            for idx, value in enumerate(self._life):
                self._state_label.append("l_"+(self._characters+self._targets)[idx])
            #print self._state_label
            for c in self._characters:
                self._trigger_state_label.append("l_"+c)
            self._trigger_state_label.append("step_no_touch")
            self._trigger_state_label.append("robot_touch")
            self._trigger_state_label.append("child_touch")

        self._trigger_state[0:len(self._characters)]=np.array(self._life[0:len(self._characters)])

        #print "child touch" 
        #print self._current_touches
        #print "robot touch" 
        #print self._robot_touch

        if self._current_touches == 0 and self._robot_touch == 0:
            self._steps_no_touch += 1
        else:
            self._steps_no_touch = 0

        self._trigger_state[-3]=self._steps_no_touch
        self._trigger_state[-2]=self._robot_touch
        self._trigger_state[-1]=(self._current_touches > 0)

        self.publish_states()

    def on_life(self, message):
        self._life = message.data

    def on_map(self, message):
        self._map = np.flipud(np.ndarray.astype(np.reshape(message.data,(message.info.height, message.info.width)),dtype=bool))
        self._map = ndimage.binary_dilation(self._map,structure=ndimage.generate_binary_structure(2,2), iterations = 5).astype(self._map.dtype)

        
    def publish_states(self):
        message = ListFloatStamped()
        message.header.stamp = rospy.Time(0)
        message.header.frame_id = "sandtray"
        message.data = self._state
        self._state_pub.publish(message)

        message.data = self._trigger_state
        self._trigger_state_pub.publish(message)

    def get_pose(self, item, reference=REFERENCE_FRAME):
        if item not in self._tl.getFrameStrings():
            rospy.logwarn_throttle(20,"%s is not yet published." % item)
            return None
        if self._tl.canTransform(reference, item, rospy.Time(0)):
            (trans,rot) = self._tl.lookupTransform(reference, item, rospy.Time(0))
            return trans
        return None
    
    def run(self):
        rospy.spin()

    def on_event(self, event):
        arguments = event.data.split("_")
        if arguments[0] == "childrelease":
            self._current_touches -= 1
        elif  arguments[0] == "robotrelease":
            self._robot_touch = False
        elif  arguments[0] == "childtouch": 
            self._current_touches += 1
        elif  arguments[0] == "robottouch":
            self._robot_touch = True
        elif arguments[0] == "characters" and len(self._characters) == 0:
            for i in range(1,len(arguments)):
                self._characters.append(arguments[i].split(",")[0])
        elif arguments[0] == "targets" and len(self._targets) == 0:
            for i in range(1,len(arguments)):
                self._targets.append(arguments[i].split(",")[0])
            if len(self._characters) > 0 and len(self._targets) > 0:
                self._initialised = True
                self._state = np.zeros((len(self._characters) * (len(self._characters)+1))/2 + (len(self._targets)+1)*len(self._characters))
                self._trigger_state = np.zeros(len(self._characters) + 3)

    def signal_handler(self, signal, frame):
            self._stopping = True
            sys.exit(0)

    def dist(self, a, b):
        return pow(a[0]-b[0],2)+pow(a[1]-b[1],2)


    def get_distance_objects(self, name1, name2):
        return self.dist(self.get_pose(name1),self.get_pose(name2))

    def get_relative_pose(self, item, goal):
        pose = self.get_pose(item)
        if pose is None:
            return None
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = REFERENCE_FRAME
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        return self._tl.transformPose(item, goal_pose)

    def get_absolute_pose(self, pose):
        return self._tl.transformPose(REFERENCE_FRAME,pose)

    def get_distance_item_pose(self, item, pose):
        pose =pose.pose.position.x,pose.pose.position.y
        return self.dist(self.get_pose(item), pose)

    def get_pose_around(self, item):
        pose = self.get_pose(item)
        if pose == None:
            return None
        x = pose[0] + random.uniform(-DISTANCE_THRESHOLD,DISTANCE_THRESHOLD)/2
        y = pose[1] + random.uniform(-DISTANCE_THRESHOLD,DISTANCE_THRESHOLD)/2
        pose = x, y
        return pose
    
    def get_pose_close_to(self, origin, goal):
        origin = self.get_pose(origin)
        goal = self.get_pose(goal)
        if origin is None or goal is None:
            return None
        origin = np.array([origin[0], origin[1]])
        goal = np.array([goal[0], goal[1]])
        dist = self.dist(origin, goal)

        threshold = 4 * DISTANCE_THRESHOLD

        if dist > threshold:
            point = -math.sqrt(threshold/dist)*(goal-origin)+goal
            point = self.find_empty_around_point(point)
        else:
            point = origin + 0.1 * (goal-origin)

        return point

    def get_point_away(self, origin, goal):
        origin = self.get_pose(origin)
        goal = self.get_pose(goal)
        if origin is None or goal is None:
            return None
        origin = np.array([origin[0], origin[1]])
        goal = np.array([goal[0], goal[1]])
        dist = self.dist(origin, goal)

        motion = 10 * DISTANCE_THRESHOLD

        point = -math.sqrt(motion/dist)*(goal-origin)+origin
        point = self.find_empty_around_point(point)

        return point

    def find_empty_around_point(self, point):
        #Flip coordinates as table are accessed [y,x]
        print "Point in: " + str(point)
        point = point[::-1]
        #Change coordinates to have tile
        scale = np.array([-self._map.shape[0] / PHYSICAL_MAP_HEIGHT, self._map.shape[1] / PHYSICAL_MAP_WIDTH])
        point = point * scale
        #Cast to int
        point = np.ndarray.astype(point, int)

        transit_point = point
        print "Transit in: " + str(transit_point)
        radius = 0
        while not self.test_point(transit_point):
            radius += 1
            for i in range(-radius,radius):
                transit_point=point+[radius,i]
                if self.test_point(transit_point):
                    break
                transit_point=point+[-radius,i]
                if self.test_point(transit_point):
                    break
                transit_point=point+[i,radius]
                if self.test_point(transit_point):
                    break
                transit_point=point+[i,-radius]
                if self.test_point(transit_point):
                    break
        print "Transit out: " + str(transit_point)

        transit_point = np.ndarray.astype(transit_point, int)
        transit_point = transit_point / scale
        transit_point = transit_point[::-1]

        print "Point out: " + str(transit_point)

        return transit_point

    def test_point(self, point):
        try:
            return not self._map[point[0], point[1]]
        except IndexError:
            return False


if __name__ == "__main__":

    rospy.init_node('state_analyser')

    rospy.loginfo("Initializing TF...")
    analyser = StateAnalyser()
    signal.signal(signal.SIGINT, analyser.signal_handler)
    analyser.run()
