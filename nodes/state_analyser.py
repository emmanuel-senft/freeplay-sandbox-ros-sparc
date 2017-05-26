#!/usr/bin/env python
import numpy as np
import numpy.ma as ma
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
from visualization_msgs.msg import MarkerArray, Marker
import shapely.geometry 


MAP_HEIGHT=0.335

REFERENCE_FRAME="/sandtray"

ITEMS = ["zebra","elephant","ball","lion","giraffe","caravan","crocodile","hippo","boy","girl"]
STATIC_ITEMS = ["rocket","alternaterocket"]
COLORS = ["black","white","purple","blue","green","yellow","red","out"]

SPATIAL_THING = STATIC_ITEMS

DISTANCE_THRESHOLD = 0.001

class StateAnalyser(object):
    def __init__(self):
        self._tl = tf.TransformListener()
        rospy.sleep(0.5) # sleep a bit to make sure the TF cache is filled

        self._event_sub = rospy.Subscriber("sandtray/interaction_events", String, self.on_event)
        self._zones_sub = rospy.Subscriber("zones", MarkerArray, self.on_zones)

        self._state_pub = rospy.Publisher("sparc/state", Int32MultiArray, queue_size = 5)

        self._zones={}
        self._zone_types = set()
        self._zoneslock = Lock()
        self._stopping = False

        self._state=np.zeros(len(ITEMS)+3,dtype = int)
        self._states=[]

        self._xmax = 0
        self._xmin = 0
        self._current_touches = 0
        self._robot_touch = False

        rospy.loginfo("Ready to play!")
        self._timer = Timer(0.5, self.get_state)
        self._timer.start()
        
        self.get_state()

    def get_state(self):
        if self._stopping:
            return
        self._timer = Timer(0.5, self.get_state)
        self._timer.start()
        
        stateLabel=["items","state"]
        for idx, item in enumerate(ITEMS):
            self._state[idx] = self.get_closest(item)
            if DEBUG:
                print item + " " + str(self._state[idx])
        self._state[-2] = self._robot_touch
        self._state[-3] = self._current_touches > 0

        if len(self._states) > 0 and (self._state == self._states[-1]).all():
            self._state[-1] += 1
        else:
            self._state[-1] = 0
        self._states.append(np.array(self._state))

        self.publish_state(self._state,stateLabel)
    
    def publish_state(self,state,labels):
        message = Int32MultiArray()
        dim1 = MultiArrayDimension()
        dim2 = MultiArrayDimension()
        dim1.label = labels[0]
        dim1.size = state.shape[0]
        dim1.stride = state.size
        message.layout.dim.append(dim1)
        dim2.label = labels[1]
        dim2.size = 1#state.shape[1]
        dim2.stride = dim2.size
        message.layout.dim.append(dim2)
        for a in state.flat[:]:
            message.data.append(a)
        self._state_pub.publish(message)
        pass

    def get_closest(self, item, pose =None):
        if pose is None:
            pose = self.get_pose(item)
        if  pose is None:
            return -1 
        distances=[]
        for spatial_thing in SPATIAL_THING:
            if spatial_thing == item:
                distance.append(float('inf'))
            else:
                distances.append(self.dist(self.get_pose(spatial_thing), pose))

        mini = min(distances)

        if mini > DISTANCE_THRESHOLD:
            return -1

        return np.argpartition(distances,1)[0]

    def get_pose(self, item, reference=REFERENCE_FRAME):
        if item not in self._tl.getFrameStrings():
            rospy.logwarn_throttle(20,"%s is not yet published." % item)
            return None
        if self._tl.canTransform(reference, item, rospy.Time(0)):
            (trans,rot) = self._tl.lookupTransform(reference, item, rospy.Time(0))
            return trans
        return None

    def zone_item(self, item):
        pose = self.get_pose(item)
        if  pose is None:
            return -1,-1 
        pose = pose[0], pose[1]
        return self.isin_zone(pose)


    def isin_zone(self, pose):
        index = [0]
        for zone_type in enumerate(self._zone_types):
            if zone_type[1] in self._zones:
                if self.isin(pose, self._zones[zone_type[1]],index):
                    return zone_type[1],index[0]
        return -1,-1

    def isin(self, point, polygons, index=[0]):
        if point is None:
            return False

        x,y = point


        for idx, polygon in enumerate(polygons):
            n = len(polygon)
            inside = False

            p1x,p1y = polygon[0]
            for i in range (1,n+1):
                p2x,p2y = polygon[i % n]
                if y > min(p1y,p2y):
                    if y <= max(p1y,p2y):
                        if x <= max(p1x,p2x):
                            if p1y != p2y:
                                xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                                if p1x == p2x or x <= xinters:
                                    inside = not inside
                p1x,p1y = p2x,p2y
            if inside:
                index[0] = idx
                return True
        return False


    def run(self):
        rospy.spin()

    def on_event(self, event):
        if event.data == "release":
            self._current_touches -= 1
        if  event.data.split("_")[0] == "releasing":
            self._robot_touch = False
        if event.data == "touch": 
            self._current_touches += 1
        if event.data.split("_")[0] == "touching":
            self._robot_touch = True

    def on_zones(self, zones):
        self._zoneslock.acquire()
        self._zones.clear()
        xmax = 0
        ymin = 0
        for zone in zones.markers:
            if zone.action == Marker.ADD:
                name = zone.ns.split("_")[-1]
                self._zone_types.add(name)
                rospy.loginfo("Adding/updating a %s zone" % name)
                self._zones.setdefault(name, []).append([(p.x,p.y) for p in zone.points])
                for point in zone.points:
                    xmax = max(xmax, point.x)
                    ymin = min(ymin, point.y)
        self._xmax=xmax
        self._ymin=ymin
        self._zone_types.add(COLORS[-1])
        self._zones.setdefault(COLORS[-1],[]).append([(xmax,0),(xmax,ymin), (xmax/0.88,ymin),(xmax/0.88,0)])
        print self._zone_types

        print "xmax " + str(xmax) + " ymin " + str(ymin)
        self._zoneslock.release()

    def signal_handler(self, signal, frame):
            self._stopping = True
            sys.exit(0)

    def dist(self, a, b):
        return pow(a[0]-b[0],2)+pow(a[1]-b[1],2)

    def get_point_action(self, masked_action):
        zone_type = -1
        zone_id = -1
        
        if masked_action[2] is not ma.masked:
            zone_type = COLORS[masked_action[2]]
            if masked_action[3] is not ma.masked:
                zone_id = masked_action[3]
        closeto = []
        for i in range(3):
            if masked_action[4+2*i] is not ma.masked:
                name= SPATIAL_THING[masked_action[4+2*i]]
                index = -1
                if masked_action[5+2*i] is not ma.masked:
                    index = masked_action[5+2*i]
                closeto.append((name, index))
        print zone_type
        print zone_id
        print closeto

        
        candidates = []
        if zone_type == -1:
            candidates += self.get_points_in_polygon([(0,0),(self._xmax,0),(self._xmax, self._ymin),(0,self._ymin)],200)
        else:            
            if len(self._zones) == 0 or len(self._zones[zone_type])==0:
                return None
            if zone_id == -1:
                regions = random.sample(self._zones[zone_type],len(self._zones[zone_type]))
            else:
                regions = self._zones[zone_type]
                regions = [regions[zone_id]]

            for poly in regions:
                candidates += self.get_points_in_polygon(poly,50)

        best_dist = 1000
        best_candidate = None

        for c in candidates:
            if closeto:
                d=self.get_distance_pose_spatial_things(c,closeto)
                if d<best_dist:
                    best_dist =d
                    best_candidate = Point(x=c[0],y=c[1])
            else:
                return Point(x=c[0],y=c[1])

        return best_candidate

    def get_point_away(self):

        candidate = self.get_points_in_polygon([(0,0),(self._xmax,0),(self._xmax, self._ymin),(0,self._ymin)],1)
        while self.get_closest(candidate) != -1:
            candidate = self.get_points_in_polygon([(0,0),(self._xmax,0),(self._xmax, self._ymin),(0,self._ymin)],1)
        candidate = candidate[0]
        print candidate
        return candidate 


    def get_points_in_polygon(self, poly, n):
        points = []
        p=shapely.geometry.Polygon(poly)
        (minx, miny, maxx, maxy) = p.bounds

        while len(points)<n:
            point = shapely.geometry.Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
            if p.contains(point):
                points.append((point.x,point.y))
        return points


    def get_distance_pose_spatial_things(self, pose, spatial_things):
        distance=0
        for spatial_thing in spatial_things:
            if spatial_thing[0] in COLORS:
                mindist = 1000
                zone_type = spatial_thing[0]
                zone_id = spatial_thing[1]
                if zone_id == -1:
                    regions = random.sample(self._zones[zone_type],len(self._zones[zone_type]))
                else:
                    regions = self._zones[zone_type]
                    regions = [regions[zone_id]]
                for poly in regions:
                    for point in poly:
                        mindist = min(self.dist(point, pose),mindist)
                distance+=mindist
            if spatial_thing[0] in ITEMS:
                distance += self.dist(pose, self.get_pose(spatial_thing[0]))
        return distance

    def get_state_pose(self, item, stamped_pose):
        pose = self.get_pose(item)
        if pose is None:
            return None
        
        item_id = ITEMS.index(item)
        pose = self._tl.transformPose(REFERENCE_FRAME,stamped_pose)
        pose = pose.pose.position
        pose = pose.x, pose.y
        #zone_type, zone_index = self.isin_zone(pose)
        closest = self.get_closest(item, pose = pose)
        #return zone_type, zone_index, closest
        return closest
    
    def get_pose_close_to(self, item):
        pose = self.get_pose(item)
        if pose == None:
            return None
        x = pose[0] + random.uniform(-DISTANCE_THRESHOLD,DISTANCE_THRESHOLD)/math.sqrt(2)
        y = pose[1] + random.uniform(-DISTANCE_THRESHOLD,DISTANCE_THRESHOLD)/math.sqrt(2)
        print x
        print y
        pose = x, y
        return pose


if __name__ == "__main__":

    rospy.init_node('state_analyser')

    rospy.loginfo("Initializing TF...")
    analyser = StateAnalyser()
    signal.signal(signal.SIGINT, analyser.signal_handler)
    analyser.run()
