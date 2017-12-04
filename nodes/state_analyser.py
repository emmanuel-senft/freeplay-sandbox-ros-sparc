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

DEBUG = False

MAP_HEIGHT=0.335

REFERENCE_FRAME="/sandtray"

DISTANCE_THRESHOLD = 0.001
PHYSICAL_MAP_WIDTH = 0.6
PHYSICAL_MAP_HEIGHT = 0.335
DIAGONAL = PHYSICAL_MAP_WIDTH*PHYSICAL_MAP_WIDTH + PHYSICAL_MAP_HEIGHT*PHYSICAL_MAP_HEIGHT
RESOLUTION = 0.005 #m per cell

class StateAnalyser(object):
    def __init__(self):
        self._tl = tf.TransformListener()
        rospy.sleep(0.5) # sleep a bit to make sure the TF cache is filled

        self._interaction_event_sub = rospy.Subscriber("sandtray/interaction_events", String, self.on_interaction_event)
        self._nao_event_sub = rospy.Subscriber("nao/events", String, self.on_nao_event)
        self._life_sub = rospy.Subscriber("sparc/life", ListFloatStamped, self.on_life)
        self._map_sub = rospy.Subscriber("map", OccupancyGrid, self.on_map)
        self._gaze_sub = rospy.Subscriber("gazepose_0", PoseStamped, self.on_gaze)

        self._state_pub = rospy.Publisher("sparc/state", ListFloatStamped, queue_size = 5)
        self._event_pub = rospy.Publisher("sandtray/interaction_events", String, queue_size = 5)

        self._stopping = False

        self._state=np.array([],dtype = float)
        self._state_label=[]
        self._characters = []
        self._targets = []
        self._initialised = False
        self._life = []
        self._eye_pose = (0,0)
        self._progression = 0
        self._step = 0
        self._step_last_action_child = 0
        self._step_last_action_robot = 0
        self._step_last_feeding = 0
        self._step_last_death = 0

        self._characters_touched_child = []
        self._characters_touched_robot = []
        self._step_last_characters_touched_child = []
        self._step_last_characters_touched_robot = []

        self._child_focus = []
        self._step_last_focus = []
        self._focus_labels = ["sandtray", "robot", "other"]

        self._game_running = False

        self._xmax = 0
        self._xmin = 0
        self._current_touches = 0
        self._robot_touch = False
        self._robot_speaks = False

        self._map = None

        rospy.loginfo("Ready to play!")
        self._timer = Timer(0.5, self.get_state)
        self._timer.start()

    def get_state(self):
        if not self._initialised or len(self._life) == 0:
            self._event_pub.publish(String("analyser_ready"))
            return

        if self._stopping or not self._game_running:
            return
        self._timer = Timer(0.5, self.get_state)
        self._timer.start()
        
        self._step += 1
        index = 0
        for idx, character in enumerate(self._characters):
            for other in (self._characters+self._targets)[idx+1:]:
                self._state[index]=self.get_distance_objects(character,other)/DIAGONAL
                index+=1
        #Last time a character is touched by the child
        for idx,character in enumerate(self._characters):
            if self._characters_touched_child[idx]:
                self._state[index] = 1
            else:
                self._state[index] = self.get_decay(self._step_last_characters_touched_child[idx],10.)
            index+=1
        #Last time a character is touched by the robot
        for idx,character in enumerate(self._characters):
            if self._characters_touched_robot[idx]:
                self._state[index] = 1
            else:
                self._state[index] = self.get_decay(self._step_last_characters_touched_robot[idx],10.)
            index+=1

        self._state[index] = self._progression
        index+=1

        #Starts trigger states
        #Life of animals
        for value in self._life:
            self._state[index] = value
            index+=1

        #Focus of attention
        for idx,v in enumerate(self._child_focus):
            if v:
                self._state[index] = 1
            else:
                self._state[index] = self.get_decay(self._step_last_focus[idx],10.)
            index+=1

        #Touches
        if self._current_touches > 0:
            self._state[index] = 1
        else:
            self._state[index] = self.get_decay(self._step_last_action_child,10.)
        index+=1

        #Actions
        if self._robot_speaks or self._robot_touch:
            self._state[index] = 1
        else:
            self._state[index] = self.get_decay(self._step_last_action_robot,10.)
        index+=1
        self._state[index] = self.get_decay(self._step_last_feeding,10.)
        index+=1
        self._state[index] = self.get_decay(self._step_last_death,10.)

        self.publish_states()

    def get_decay(self, step, parameter):
        return np.exp((step-self._step)/parameter)

    def on_life(self, message):
        self._life = message.data

    def on_map(self, message):
        self._map = np.flipud(np.ndarray.astype(np.reshape(message.data,(message.info.height, message.info.width)),dtype=bool))
        self._map = ndimage.binary_dilation(self._map,structure=ndimage.generate_binary_structure(2,2), iterations = 5).astype(self._map.dtype)

    def on_gaze(self, message):
        self._eye_pose = (message.pose.position.x,message.pose.position.y)
        
    def publish_states(self):
        message = ListFloatStamped()
        message.header.stamp = rospy.Time(0)
        message.header.frame_id = "sandtray"
        message.data = self._state
        self._state_pub.publish(message)

    def get_pose(self, item, reference=REFERENCE_FRAME):
        #Was slowing down too much 
        #if item not in self._tl.getFrameStrings():
        #    rospy.logwarn_throttle(20,"%s is not yet published." % item)
        #    return None
        if self._tl.canTransform(reference, item, rospy.Time(0)):
            (trans,rot) = self._tl.lookupTransform(reference, item, rospy.Time(0))
            return trans
        return None
    
    def run(self):
        rospy.spin()

    def on_interaction_event(self, event):
        arguments = event.data.split("_")
        if arguments[0] == "start" or arguments[0] == "running":
            self._game_running = True
            self._progression = float(arguments[1])/float(arguments[2])
            self._step_last_characters_touched_child = np.zeros(len(self._characters))
            self._step_last_characters_touched_robot = np.zeros(len(self._characters))
            self._step_last_focus = np.zeros(len(self._focus_labels))
            self._characters_touched_child = np.full(len(self._characters), False, dtype=bool)
            self._characters_touched_robot = np.full(len(self._characters), False, dtype=bool)
            self._child_focus =  np.full(len(self._focus_labels), False, dtype=bool)
            self._step_last_feeding = 0
            self._step_last_death = 0
            self.get_state()

        elif arguments[0] == "stop":
            self._game_running = False
            self._robot_touch = False
        elif arguments[0] == "childrelease":
            self._current_touches -= 1
            #If the character moves while the child release, undefined is sent
            try:
                idx = self._characters.index(arguments[1])
            except:
                idx = np.where(self._characters_touched_child == True)
            self._step_last_characters_touched_child[idx] = self._step
            self._characters_touched_child[idx]=False
            if self._current_touches == 0:
                self._step_last_action_child = self._step
        elif  arguments[0] == "robotrelease":
            try:
                idx = self._characters.index(arguments[1])
            except:
                idx = np.where(self._characters_touched_robot == True)
            self._robot_touch = False
            self._characters_touched_robot[idx]=False
            self._step_last_characters_touched_robot[idx] = self._step
            self._step_last_action_robot = self._step
        elif  arguments[0] == "childtouch": 
            self._current_touches += 1
            self._characters_touched_child[self._characters.index(arguments[1])]=True
        elif  arguments[0] == "robottouch":
            self._characters_touched_robot[self._characters.index(arguments[1])]=True
            self._robot_touch = True
        elif arguments[0] == "characters" and len(self._characters) == 0:
            for i in range(1,len(arguments)):
                self._characters.append(arguments[i].split(",")[0])
        elif arguments[0] == "targets" and len(self._targets) == 0:
            for i in range(1,len(arguments)):
                self._targets.append(arguments[i].split(",")[0])
        elif arguments[0] == "animaleats":
            self._step_last_feeding = self._step
        elif arguments[0] == "animaldead":
            self._step_last_death = self._step
            self._characters_touched_child[self._characters.index(arguments[1])] = False
            self._characters_touched_robot[self._characters.index(arguments[1])] = False
        elif arguments[0] == "looking":
            for idx,l in enumerate(self._focus_labels):
                if l == arguments[1]:
                    self._child_focus[idx] = True
                elif self._child_focus[idx]:
                    self._child_focus[idx] = False
                    self._step_last_focus[idx] = self._step

        if len(self._characters) > 0 and len(self._targets) > 0 and not self._initialised:
            self.init_label()

    def on_nao_event(self, message):
        arguments = message.data.split("-")
        if arguments[0] == "blocking_speech_finished":
            self._step_last_action_robot = self._step
            self._robot_speaks = False
        if arguments[0] == "blocking_speech_started":
            self._robot_speaks = True

    def init_label(self):
        for idx, character in enumerate(self._characters):
            for other in (self._characters+self._targets)[idx+1:]:
                self._state_label.append("d_"+character+"_"+other)
        #Last time a character is touched by the child
        for character in self._characters:
            self._state_label.append("tc_"+character)
        #Last time a character is touched by the robot
        for character in self._characters:
            self._state_label.append("tr_"+character)
        #progression, round number
        self._state_label.append("g_progress")
        #State used for trigger
        for v in self._targets + self._characters: 
            self._state_label.append("l_"+v)
        for s in self._focus_labels:
            self._state_label.append("f_"+s)
        self._state_label.append("last_child_action")
        self._state_label.append("last_robot_action")
        self._state_label.append("last_feeding")
        self._state_label.append("last_death")

        self._state = np.zeros(len(self._state_label))

        self._step_last_characters_touched_child = self._step * np.ones(len(self._characters))
        self._step_last_characters_touched_robot = self._step * np.ones(len(self._characters))
        self._step_last_focus = self._step * np.ones(len(self._focus_labels))
        self._characters_touched_child = np.full(len(self._characters), False, dtype=bool)
        self._characters_touched_robot = np.full(len(self._characters), False, dtype=bool)
        self._child_focus =  np.full(len(self._focus_labels), False, dtype=bool)

        self._initialised = True

    def signal_handler(self, signal, frame):
            self._stopping = True
            self._timer.cancel()
            sys.exit(0)

    def dist(self, a, b):
        return pow(a[0]-b[0],2)+pow(a[1]-b[1],2)

    def get_distance_objects(self, name1, name2):
        pose1 = self.get_pose(name1)
        pose2 = self.get_pose(name2)
        if pose1 is None or pose2 is None or pose1[0] < 0 or pose2[0] < 0:
            return float("NaN")
        return self.dist(pose1,pose2)

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
            if self.dist(origin, goal) < self.dist(point, goal):
                return None
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
        point = point[::-1]
        #Change coordinates to have tile
        scale = np.array([-self._map.shape[0] / PHYSICAL_MAP_HEIGHT, self._map.shape[1] / PHYSICAL_MAP_WIDTH])
        point = point * scale
        #Cast to int
        point = np.ndarray.astype(point, int)

        transit_point = point
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

        transit_point = np.ndarray.astype(transit_point, int)
        transit_point = transit_point / scale
        transit_point = transit_point[::-1]

        return transit_point

    def test_point(self, point):
        if np.any(point<0):
            return False
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
