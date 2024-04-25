#! /usr/bin/env python
import rospy
import rospkg
import numpy as np
import math
from numpy.linalg import norm
import actionlib
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion, PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String, Int32, Float64, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from tf import transformations
from tf.transformations import quaternion_multiply
import tf
from nav_msgs.msg import OccupancyGrid
from pcl_arrow_detect import ArrowDetector
from numpy import nan
import cv2

path = rospkg.RosPack().get_path("motion_plan")
MARKERS_MAX = 50
ROOT_LINK = "root_link"
USE_ROBUST_ARROW_DETECT = 1
MAX_ARROW_DETECT = 3
ARROW_MAX_ITER = 7
CNT_AREA_THRES = 20
eps=1
SERVO_SLEEP = 5

class client:
    def __init__(self, sim=False):
        rospy.init_node("goal_client_node")
        rospy.loginfo(f"goal_client_node init sim:{sim}")
        self.sim = sim # TODO do all changes for sim
        # rospy.init_node("joint_commander")
        # rospy.loginfo("joint_commander init")

        # rospy.Subscriber('joint_commands', Float32MultiArray, callback=self.cam_cont_callback)

        self.joint = [rospy.Publisher('rover/joint1_position_controller/command', Float64, queue_size=60)]
        

        # define a client for to send goal requests to the move_base server through a SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for the action server to come up
        while (
            not self.ac.wait_for_server(rospy.Duration.from_sec(2.0))
            and not rospy.is_shutdown()
        ):
            rospy.loginfo("Waiting for the move_base action server to come up")
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            "map", ROOT_LINK, rospy.Time(0), rospy.Duration(10.0)
        )
        self.mapData = OccupancyGrid()  # map
        self.frame = None
        self.lidar_data = None
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallBack)
        rospy.Subscriber("/scan_filtered", LaserScan, self.lidar_callback)
        rospy.Subscriber('/LatLon',Float64MultiArray,self.gps_callback)

        self.arrow_detector = ArrowDetector(sim=self.sim)
        print("ArrowDetector Launched")
        self.marker_array_pub = rospy.Publisher(
            "/detected_arrow", MarkerArray, queue_size=10
        )
        self.led_pub = rospy.Publisher("/rover/tasks_status", String, queue_size=10)
        self.cam_servo_pub = rospy.Publisher('/rover/auto_cam_angle', Int32, queue_size=20)
        self.marker_array = MarkerArray()
        self.marker_count = 0
        self.completed_list = []
        self.last_good_location = self.bot_to_map(0, 0, (0, 0, 0, 1))
        self.cam_servo_angle = 0
        rospy.Rate(5).sleep()  #

        # rospy.spin()
   
    def target_detect(self):
        print(self.arrow_detector.cone_detect())
        found,val,cone_distance = self.arrow_detector.cone_detect()
        if cone_distance==0 or cone_distance==nan:
            q=q_from_vector3D(val)
            return found,q,cone_distance #sending quaternion
        return found,val,cone_distance #sending pos

    def detect_iter(self, far=True, max_iter=ARROW_MAX_ITER):
        #TODO: needs work for iroc
        # returns Found(0/1), position(x,y,z), theta(degrees; rover forward=0)
        found_arr, cnt_area_arr, pos_arr, orient_arr, timestamp_arr = [], [], [], [], []
        count = 0
        for i in range(max_iter):
            found, pos, orient, timestamp, cnt_area = self.arrow_detector.arrow_detect(far=far, visualize=False)
            found_arr.append(found)
            pos_arr.append(pos)
            orient_arr.append(orient)
            cnt_area_arr.append(cnt_area)
            timestamp_arr.append(timestamp)
            if found:
                count += 1
        if count >= MAX_ARROW_DETECT:
            var_arr = [cnt_area_arr[i] for i in range(len(cnt_area_arr))
                        if found_arr[i]]
            if np.var(np.array(var_arr)) > CNT_AREA_THRES:
                return False, None, None, timestamp
            else:
                found_arr.reverse()
                idx = len(found_arr) - found_arr.index(True) - 1
                print("foundddddddd\n\n\n")
                pt = pos_arr[idx]
                print(pt)
                angle = -self.cam_servo_angle * np.pi / 180
                X = (pt[0]*np.cos(angle)+pt[1]*np.sin(angle)) # take projections
                Y = (-pt[0]*np.sin(angle)+pt[1]*np.cos(angle))
                # posx,posy,_ = self.bot_to_map(X,Y,q=None,frame="mrt/camera_link")
                return True, (X,Y,0), orient_arr[idx], timestamp_arr[idx]
        return False, None, None, timestamp

    def mapCallBack(self, data):
        self.mapData = data

    # TODO change to ps, q format for all
    def bot_to_map(self, pos_x, pos_y, q, frame=ROOT_LINK, timestamp=None):
        '''
        Converts coordinates in an arbitrary reference frame to coordinates in map frame.
        
        Args : pos_x : X-Coordinate of goal position in meters
               pos_y : Y-Coordinate of goal position in meters
               frame is by default set to 'Root Link'. 
               q : Quaternion representing Orientation

        Returns : Coordinates and Orientation in Map Frame
        '''
        ps = PoseStamped() # POSE MESSAGE OF ROS. HAS ALL PARAMETERS OF LOCATION AND ORIENTATIOM
        new_ps = PoseStamped() #WHAT WE WANT TO OUTPUT

        # set up the POSE parameters FOR THE OLD POSE
        ps.header.frame_id = frame

        if timestamp == None:
            timestamp = rospy.Time.now()
        ps.header.stamp = timestamp
        ps.pose.position = Point(pos_x, pos_y, 0)
        ps.pose.orientation = recast_quaternion(q)

        success = False
        while not rospy.is_shutdown() and success == False:
            try:
                new_ps = self.listener.transformPose("map", ps) #CONVERTS OLD POSE TO NEW POSE IN THE MAP FRAME
                success = True
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                success = False
        new_ps.pose.orientation.x = 0  # TODO Check if advisable to do so
        new_ps.pose.orientation.y = 0
        return new_ps.pose.position.x, new_ps.pose.position.y, new_ps.pose.orientation # X,Y IN METERS, ORIENTATION IN QUATERNION, ALL IN MAP FRAME

    def send_goal(self, xGoal, yGoal, q=None, frame="map"):  # frame=ROOT_LINK
        # relative to the bot location
        # quaternion is a 4-tuple/list-x,y,z,w or Quaternion
        goal = MoveBaseGoal() # MESSAGE OBJECT

        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        # set up the frame parameters
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(xGoal, yGoal, 0) 
        goal.target_pose.pose.orientation = recast_quaternion(q)

        rospy.loginfo(
            "Sending goal location - [" + str(xGoal) + ", " + str(yGoal) + "] .."
        )
        self.add_arrow(xGoal, yGoal, q)
        self.ac.send_goal(goal) #SENDS GOAL MESSAGE TO ACTIONLIB 

    def move_to_goal(self, xGoal:float, yGoal:float, q=None , frame="map"):  # frame=ROOT_LINK #TODO Do Args : X-dist, Y-dist,quaternion(0,0,sin(theta/2),cos(theta/2)) and Returns 
        # Q is Tuple of 4 arguments
        #Root LInk is the own frame of reference of the rover
        '''
        Args : xGoal : X-Dist of Goal
               yGoal : Y-Dist of Goal
               q : Quaternion representing Orientation of the rover. Tuple of 4 args
               frame : Reference frame of Coordinates        
        '''
        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q) # Converts Goal Coordinates from Rover's Frame to Global Map Frame
            frame = "map"
        self.send_goal(xGoal, yGoal, q, frame)

        self.ac.wait_for_result(rospy.Duration(150))

        if self.ac.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def find_xy_off_goal(self, xGoal, yGoal, q=None, frame="map", off_dist=0.5,ahead=0.75):
        goal_initial=self.find_off_goal(xGoal, yGoal, q=q, frame=frame, offset=(-0.0, off_dist, 0, 0))
        goal_final=just_ahead(*goal_initial,off_dist=ahead)
        return goal_final
        #print('move to off goal',goal_initial,goal_final)
        # return self.move_to_goal(*goal_final)

    def find_off_goal(self, xGoal, yGoal, q=None, frame="map", offset=(0, 0, 0, 0)):
        if frame == ROOT_LINK:
            xGoal, yGoal, q = self.bot_to_map(xGoal, yGoal, q)
            frame = "map"
        q = uncast_quaternion(q)
        x0, y0, _ = self.bot_to_map(0, 0, (0, 0, 0, 1))  # Bot location in map
        offset = quaternion_multiply(q, offset)
        offset = quaternion_multiply(offset, transformations.quaternion_inverse(q))
        x1, y1 = xGoal + offset[0], yGoal + offset[1]
        #cell1 = get_cell_status(self.mapData, [x1, y1])
        x2, y2 = xGoal - offset[0], yGoal - offset[1]
        #cell2 = get_cell_status(self.mapData, [x2, y2])
        if (norm([x1 - x0, y1 - y0]) < norm([x2 - x0, y2 - y0])):
            x, y = x1, y1
        else:
            x,y = x2, y2   
        x_c,y_c = x,y    
        cell_c = get_cell_status(self.mapData,[x,y]) 
        multiplier = 1   
        i = 1
        while cell_c != 0:
            x = x_c + 0.2 * offset[0] * i * multiplier    
            y = y_c + 0.2 * offset[1] * i * multiplier
            cell_c = get_cell_status(self.mapData,[x,y]) 
            i+=1
        if cell_c == 0:
            i = 0
        print("locations of x,y",x,y)
        return x, y, q

    def add_vert_arrow(self,x,y,q,color=(0,1,0)):
        q_right = (0, -np.sqrt(0.5), 0, np.sqrt(0.5))
        q_right = quaternion_multiply(uncast_quaternion(q), q_right)
        self.add_arrow(x, y, q_right, color=color)

    def cancel_goal(self):
        self.ac.cancel_goal()
        rospy.loginfo("Goal cancelled")

    def recovery(self, far=True):
        rospy.loginfo("Initiating recovery")
        found, pos, orient, timestamp = self.arrow_detect(far)
        j = 0
        while found == False and j < 6:
            x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
            q = uncast_quaternion(q)
            q = quaternion_multiply(q, (0, 0, np.sin(0.1), np.cos(0.1)))
            self.move_to_goal(x, y, q)
            rospy.sleep(1.0)
            found, pos, orient, timestamp = self.arrow_detect(far)
            j += 1
        j = 0
        while found == False and j < 12:
            x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
            q = uncast_quaternion(q)
            q = quaternion_multiply(q, (0, 0, np.sin(-0.1), np.cos(-0.1)))
            self.move_to_goal(x, y, q)
            rospy.sleep(1.0)
            found, pos, orient, timestamp = self.arrow_detect(far)
            j += 1
        if found:
            orient2 = orient + 90 if orient < 0 else orient - 90
            q = (
                0,
                0,
                np.sin(np.pi * orient2 / (2 * 180)),
                np.cos(np.pi * orient2 / (2 * 180)),
            )
            posx, posy, q = self.bot_to_map(pos[0], pos[1], q, timestamp=timestamp)  # map frame
        if found == False or pos is None or self.is_complete(posx, posy, q):
            rospy.loginfo("Failed. Moving to last known good location")
            self.move_to_goal(*self.last_good_location)
            return False, None, None, timestamp
        else:
            return found, pos, orient, timestamp

    def urc_recovery_oscillate(self,type:int=1,iterations:int=3+1+3,angle:float=0.6,move_rover:bool=False,full_rotation:bool=True): 
        #iterations represents counter_clockwise=3 + reset_to_center=1 + clockwise=3 ; angle to move in rad
        '''
          Args : type : Type of Goal(Takes Value 1 or 2)
               : iterations : How many times oscillate
               : Angle : Unit is Radians, Angle per Step
               : Move Rover : True : Rover Moves, False : Camera Moves
               : Full Rotation : If True, Rover will rotate 180 and again camera will move -90 to +90.
        '''
        found,pts=self.ar_detect()
        j = 1 # Iterator 
        discovered=[0,[]] 
        '''
        If You find a tag, store in discovered. First element represents found, second element is list of list of coordinates of center of found tags IN Map Frame
        '''
        rotations = 0 # Monitors number of rotations of bot
        while j < iterations+1:
            if move_rover:
                x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1)) # Will give current location and quat of bot in Map Frame
                q = uncast_quaternion(q)
                q = quaternion_multiply(q, (0, 0, np.sin(angle/2), np.cos(angle/2)))
                if j%((iterations+1)/2)==0:# Executes only only once , when j = 4, to reverse direction of rotation. 
                    #When executed, it will reset rover back to original orientation, and flip the angle.
                    angle = -angle
                    q = quaternion_multiply(q, (0, 0, np.sin(((iterations+1)/2)*angle/2), np.cos(((iterations+1)/2)*angle/2)))# Resets rover back to original orientation
                self.move_to_goal(x, y, q)
            else:
                # Check if cam_servo_angle is b/w -90 and +90 or not.If overshoot , then set to +90 or -90.
                if j%((iterations+1)/2)==0:
                    angle = -angle
                    self.move_cam_servo(int(((iterations-1)/2)*angle*180/np.pi)) 
                else:
                    self.move_cam_servo(int(angle*180/np.pi))
            rospy.sleep(1.0)
            found, pts = self.ar_detect()
            print(found, pts)
            print('recovery stage ',j)
            j += 1
            if found==type:
                self.move_cam_servo(0,absolute_pose=True)
                break
            if found==1 and type==2:
                if discovered[0]==1:
                    comp_x,comp_y=pts[0][0],pts[0][1]
                    if abs(comp_x-discovered[1][0][0])<eps and abs(comp_y-discovered[1][0][1])<eps: #hecks if the new aruco detected is the same as previous one
                        pass
                    else:
                        discovered[0]+=1    
                        discovered[1].append([pts[0][0],pts[0][1]])
                if discovered[0]==2:
                    print('found in urc_recovery_final_stage',discovered[0],discovered[1])
                    return discovered[0],discovered[1]
                else: # discover[0] is currently 0
                    discovered[0]+=1
                    discovered[1].append([pts[0][0],pts[0][1]])    

            if full_rotation and rotations==0:
                j=1
                self.move_cam_servo(0,absolute_pose=True)
                x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
                q = uncast_quaternion(q)
                q = quaternion_multiply(q, (0, 0, 1, 0))
                self.move_to_goal(x, y, q)
                
        # while found == False and j < 12:
        #     x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
        #     q = uncast_quaternion(q)
        #     q = quaternion_multiply(q, (0, 0, np.sin(-0.2), np.cos(-0.2)))
        #     self.move_to_goal(x, y, q)
        #     rospy.sleep(1.0)
        #     found, theta, pts = self.ar_detect()
        #     print(found, theta, pts)
        #     print('recovery stage ',j)
        #     j += 1
        self.move_cam_servo(0,absolute_pose=True)
        if found>0:
            return found, [[i[0],i[1]] for i in pts]
        else:
            return 0, None


    def irc_recovery_oscillate(self,type:int=1,iterations:int=3+1+3,angle:float=0.6,move_rover:bool=False,full_rotation:bool=False, far:bool=True): 
        #iterations represents counter_clockwise=3 + reset_to_center=1 + clockwise=3 ; angle to move in rad
        rospy.loginfo("Initiating recovery")
        found, pos, orient, timestamp = self.arrow_detect(far)
        j = 1 # Iterator 
        while j < iterations+1:
            if move_rover:
                x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1)) # Will give current location and quat of bot in Map Frame
                q = uncast_quaternion(q)
                q = quaternion_multiply(q, (0, 0, np.sin(angle/2), np.cos(angle/2)))
                if j%((iterations+1)/2)==0:# Executes only only once , when j = 4, to reverse direction of rotation. 
                    #When executed, it will reset rover back to original orientation, and flip the angle.
                    angle = -angle
                    q = quaternion_multiply(q, (0, 0, np.sin(((iterations+1)/2)*angle/2), np.cos(((iterations+1)/2)*angle/2)))# Resets rover back to original orientation
                self.move_to_goal(x, y, q)
            else:
                # Check if cam_servo_angle is b/w -90 and +90 or not.If overshoot , then set to +90 or -90.
                if j%((iterations+1)/2)==0:
                    angle = -angle
                    self.move_cam_servo(0,absolute_pose=True) 
                else:
                    self.move_cam_servo(int(angle*180/np.pi))
            rospy.sleep(1.0)
            found, pos, orient, timestamp = self.arrow_detect(far)
            print(found, pos, orient)
            print('recovery stage ',j)
            j += 1
            if found:
                self.move_cam_servo(0,absolute_pose=True)
                break
        
        if found:
            orient2 = orient + 90 if orient < 0 else orient - 90
            q = (
                0,
                0,
                np.sin(np.pi * orient2 / (2 * 180)),
                np.cos(np.pi * orient2 / (2 * 180)),
            )
            posx, posy, q = self.bot_to_map(pos[0], pos[1], q, timestamp=timestamp)  # map frame
        
        self.move_cam_servo(0,absolute_pose=True)
        
        if found == False or pos is None or self.is_complete(posx, posy, q):
            rospy.loginfo("Failed. Moving to last known good location")
            self.move_to_goal(*self.last_good_location)
            return False, None, None, timestamp
        else:
            return found, pos, orient, timestamp


    def urc_recovery(self,type=1,move_by_rad=0.6,move_rover=True):
        rospy.loginfo("Initiating recovery")
        found, pts = self.ar_detect()
        j = 0
        discovered=[0,[]]
        while j < 11:
            if move_rover:
                x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
                q = uncast_quaternion(q)
                q = quaternion_multiply(q, (0, 0, -np.sin(move_by_rad/2), np.cos(move_by_rad/2)))
                self.move_to_goal(x, y, q)
            else:
                self.move_cam_servo(int(move_by_rad*180/np.pi) % 360)
            rospy.sleep(1.0)
            found, pts = self.ar_detect()
            print(found, pts)
            print('recovery stage ',j)
            j += 1
            if found==type:
                self.move_cam_servo(0,absolute_pose=True)
                break
            if found==1 and type==2:
                if discovered[0]==1:
                    comp_x,comp_y=pts[0][0],pts[0][1]
                    if abs(comp_x-discovered[1][0][0])<eps and abs(comp_y-discovered[1][0][1])<eps:
                        pass
                    else:
                        discovered[0]+=1
                        
                        # print('conv',pts,list(self.bot_to_map(pts[0][0],pts[0][1],q=None,frame="mrt/camera_link")))
                        discovered[1].append([pts[0][0],pts[0][1]])
                if discovered[0]==2:
                    print('rec return',discovered[0],discovered[1])
                    return discovered[0],discovered[1]
                else:
                    discovered[0]+=1
                    # print('conv',pts,list(self.bot_to_map(pts[0][0],pts[0][1],q=None,frame="mrt/camera_link")))
                    discovered[1].append([pts[0][0],pts[0][1]])

        j = 0
        # while found == False and j < 12:
        #     x, y, q = self.bot_to_map(0, 0, (0, 0, 0, 1))
        #     q = uncast_quaternion(q)
        #     q = quaternion_multiply(q, (0, 0, np.sin(-0.2), np.cos(-0.2)))
        #     self.move_to_goal(x, y, q)
        #     rospy.sleep(1.0)
        #     found, theta, pts = self.ar_detect()
        #     print(found, theta, pts)
        #     print('recovery stage ',j)
        #     j += 1
        self.move_cam_servo(0,absolute_pose=True)
        if found>0:
            return found, [[i[0],i[1]] for i in pts]
        else:
            return 0, None


    def add_arrow(
        self, pos_x, pos_y, q, color=(0.2, 0.5, 1.0), pos_z=0
    ):  # color = (r,g,b), in [0,1]
        marker = make_arrow_marker(
            Pose(Point(pos_x, pos_y, pos_z), recast_quaternion(q)),
            self.marker_count,
            color,
        )

        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        if self.marker_count > MARKERS_MAX:
            self.marker_array.markers.pop(0)

        self.marker_array.markers.append(marker)

        # Renumber the marker IDs
        id = 0
        for m in self.marker_array.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        self.marker_array_pub.publish(self.marker_array)

        self.marker_count += 1

def recast_quaternion(quaternion):
    if quaternion is None:
        q = Quaternion(0, 0, 0, 1)
    elif (
        isinstance(quaternion, list)
        or isinstance(quaternion, tuple)
        or isinstance(quaternion, np.ndarray)
    ):
        q = Quaternion(*quaternion)
    elif isinstance(quaternion, Quaternion):
        q = quaternion
    else:
        print("Quaternion in incorrect format: ", type(quaternion))
        q = Quaternion(0, 0, 0, 1)
    return q


def uncast_quaternion(quaternion):
    if quaternion is None:
        q = (0, 0, 0, 1)
    elif (
        isinstance(quaternion, list)
        or isinstance(quaternion, tuple)
        or isinstance(quaternion, np.ndarray)
    ):
        q = tuple(quaternion)
    elif isinstance(quaternion, Quaternion):
        q = quaternion
        q = (q.x, q.y, q.z, q.w)  # lazy+readable code
    else:
        print("Quaternion in incorrect format: ", type(quaternion))
        q = (0, 0, 0, 1)
    return q


def q_from_vector3D(point):
    # http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    q = Quaternion()
    # calculating the half-way vector.
    u = [1, 0, 0]
    norm = np.linalg.norm(point)
    v = np.asarray(point) / norm
    if np.all(u == v):
        q.w = 1
        q.x = 0
        q.y = 0
        q.z = 0
    elif np.all(u == -v):
        q.w = 0
        q.x = 0
        q.y = 0
        q.z = 1
    else:
        half = [u[0] + v[0], u[1] + v[1], u[2] + v[2]]
        q.w = np.dot(u, half)
        temp = np.cross(u, half)
        q.x = temp[0]
        q.y = temp[1]
        q.z = temp[2]
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm == 0:
        norm = 1
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return q


def diff(q1, q2):  # accepts tuples
    q1 = uncast_quaternion(q1)
    q2 = uncast_quaternion(q2)
    q1_inv = transformations.quaternion_inverse(q1)
    diff_q = quaternion_multiply(q2, q1_inv)
    return abs(
        transformations.euler_from_quaternion(diff_q)[2]
    )  # return yaw between two angles (quaternions)


def get_cell_status(mapData, pt):
    # returns grid value at point "pt"- shape:(2)
    # map data:  100 occupied      -1 unknown       0 free
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data

    index = (np.floor((pt[1] - Xstarty) / resolution) * width) + (
        np.floor((pt[0] - Xstartx) / resolution)
    )
    if np.isnan(index) == True:
        print("Error: index is NaN, check : ", resolution, pt, width, Xstartx, Xstarty)
        return 100

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100


def just_ahead(pos_x, pos_y, q, off_dist=0.5):
    q = recast_quaternion(q)
    offset = quaternion_multiply((q.x, q.y, q.z, q.w), (off_dist, 0, 0, 0))
    offset = quaternion_multiply(
        offset, transformations.quaternion_inverse((q.x, q.y, q.z, q.w))
    )
    x, y = pos_x + offset[0], pos_y + offset[1]
    return x, y, q


def make_arrow_marker(ps, id, color=(0.2, 0.5, 1.0)):
    # make a visualization marker array for the occupancy grid
    m = Marker()
    m.action = Marker.ADD
    m.header.frame_id = "map"
    m.header.stamp = rospy.Time.now()
    m.ns = "arrows"
    m.id = id
    m.type = Marker.ARROW
    m.pose = ps
    m.scale = Point(1, 0.1, 0.1)
    m.color.r = color[0]
    m.color.g = color[1]
    m.color.b = color[2]
    m.color.a = 1

    return m

def main():
    sin45 = 1 / np.sqrt(2)
    my_client = client()
    my_client.move_to_off_goal(4, 0, q=(0, 0, sin45, sin45))  # 4,0
    my_client.move_to_off_goal(4, 6)  # 4,6
    my_client.move_to_off_goal(8, 6, q=(0, 0, sin45, -sin45))  # 8,6
    my_client.move_to_off_goal(8, 2)  # 8,2
    my_client.move_to_off_goal(
        12, 2, q=(0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8))
    )  # 12,2
    my_client.move_to_off_goal(
        12, 6, q=(0, 0, sin45, sin45)
    )  # 12,6#check for error, if any
    my_client.move_to_off_goal(8, 10, q=(0, 0, 1, 0))  # 8,10
    my_client.move_to_goal(-2, 10, q=(0, 0, 1, 0))  # -2,10

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Closing")
