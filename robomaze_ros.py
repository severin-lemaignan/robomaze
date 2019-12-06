#! /usr/bin/env python

import yaml
import time
import random
import cv2
import numpy as np
from math import cos, sin,pi,floor,sqrt

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import tf

RES_ROOT="res/"
FLOOR=1
TOP=2
BOTTOM=3
LEFT=4
RIGHT=5
TOP_LEFT_CVX=6
TOP_LEFT_CCV=7
TOP_RIGHT_CVX=8
TOP_RIGHT_CCV=9
BOTTOM_LEFT_CVX=10
BOTTOM_LEFT_CCV=11
BOTTOM_RIGHT_CVX=12
BOTTOM_RIGHT_CCV=13

BOTTOM_TOP_RIGHT=14
BOTTOM_TOP_LEFT=15
BOTTOM_LEFT_RIGHT=16
TOP_LEFT_RIGHT=17
BOTTOM_TOP=18
LEFT_RIGHT=19

ROBOT=cv2.imread(RES_ROOT + "walle_top.png",cv2.IMREAD_UNCHANGED)
ROBOT_ALPHA = ROBOT[:,:,3]
ROBOT_RGB = ROBOT[:,:,:3]

TILES = {
        FLOOR: [cv2.imread(RES_ROOT + "floor2.jpg"),cv2.imread(RES_ROOT + "floor.jpg"),cv2.imread(RES_ROOT + "floor.jpg"),cv2.imread(RES_ROOT + "floor.jpg")],
        TOP: cv2.imread(RES_ROOT + "top.jpg"),
        BOTTOM: cv2.imread(RES_ROOT + "bottom.jpg"),
        LEFT: cv2.imread(RES_ROOT + "left.jpg"),
        RIGHT: cv2.imread(RES_ROOT + "right.jpg"),
        TOP_LEFT_CVX: cv2.imread(RES_ROOT + "top_left_cvx.jpg"),
        TOP_LEFT_CCV: cv2.imread(RES_ROOT + "top_left_ccv.jpg"),
        TOP_RIGHT_CVX: cv2.imread(RES_ROOT + "top_right_cvx.jpg"),
        TOP_RIGHT_CCV: cv2.imread(RES_ROOT + "top_right_ccv.jpg"),
        BOTTOM_LEFT_CVX: cv2.imread(RES_ROOT + "bottom_left_cvx.jpg"),
        BOTTOM_LEFT_CCV: cv2.imread(RES_ROOT + "bottom_left_ccv.jpg"),
        BOTTOM_RIGHT_CVX: cv2.imread(RES_ROOT + "bottom_right_cvx.jpg"),
        BOTTOM_RIGHT_CCV: cv2.imread(RES_ROOT + "bottom_right_ccv.jpg"),

        BOTTOM_TOP_RIGHT:cv2.imread(RES_ROOT + "bottom_top_right.jpg"),
        BOTTOM_TOP_LEFT:cv2.imread(RES_ROOT + "bottom_top_left.jpg"),
        BOTTOM_LEFT_RIGHT:cv2.imread(RES_ROOT + "bottom_left_right.jpg"),
        TOP_LEFT_RIGHT:cv2.imread(RES_ROOT + "top_left_right.jpg"),
        BOTTOM_TOP:cv2.imread(RES_ROOT + "bottom_top.jpg"),
        LEFT_RIGHT:cv2.imread(RES_ROOT + "left_right.jpg"),
        }
        
class Maze:
    height= 20
    width = 20
    TILESIZE=5 #m
    M2PX=7 # TILESIZE[m] * M2PX = TILESIZE[px]

    data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,0,0,0,
            0,1,1,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,0,0,
            0,1,0,0,1,0,1,1,1,0,0,0,1,1,0,0,1,1,0,0,
            0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,
            0,1,0,0,0,0,0,0,1,0,0,0,1,1,0,0,1,0,0,0,
            0,1,1,0,1,0,1,0,0,0,1,0,0,1,1,0,1,0,1,0,
            0,1,0,0,1,0,1,1,1,0,1,1,0,1,0,0,0,0,1,0,
            0,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,
            0,1,1,1,1,0,0,1,1,1,1,0,0,0,0,0,1,0,1,0,
            0,1,1,0,1,0,0,0,0,0,1,0,1,1,1,0,0,0,1,0,
            0,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,
            0,1,0,0,1,0,1,1,0,0,0,0,0,1,1,0,1,0,1,0,
            0,1,1,1,1,0,1,1,1,1,1,1,0,1,1,0,1,0,1,0,
            0,1,1,1,1,1,1,1,1,1,1,0,0,1,1,0,1,1,1,0,
            0,1,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,
            0,1,1,0,1,0,1,1,0,0,1,0,0,1,1,0,1,0,1,0,
            0,1,1,1,1,0,0,1,1,1,1,1,0,1,0,0,1,0,1,0,
            0,1,1,1,1,1,1,1,0,1,1,0,0,1,1,1,1,1,1,0,
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    @staticmethod
    def is_obstacle(x_m,y_m):
        """x_m and y_m must be provided in meters
        """

        x = int(floor(round(x_m)/Maze.TILESIZE))
        y = int(floor(round(y_m)/Maze.TILESIZE))

        return Maze.is_obstacle_raw(x,y)

    @staticmethod
    def is_obstacle_raw(x,y):
        """x_m and y_m must be provided in meters
        """

        if x >= 0 and \
           y >= 0 and \
           x < Maze.width and \
           y < Maze.height and \
           Maze.data[x + (Maze.height - y - 1) * Maze.width]:
            return False
        return True

    @staticmethod
    def get_surrounding_obstacles_raw(x,y):
        obs = []
        for x,y in [(x,y+1),(x,y-1),(x-1,y),(x+1,y),(x-1,y+1),(x+1,y+1),(x-1,y-1),(x+1,y-1)]:
            obs.append(Maze.is_obstacle_raw(x,y))
        return obs
    @staticmethod
    def get_neighbour_cells(x,y):
        """x, y are in meters
        """
        cx, cy = int(floor(x/Maze.TILESIZE)) * Maze.TILESIZE, int(floor(y/Maze.TILESIZE)) * Maze.TILESIZE

        # note: no need to include the diagonals - they will be visited from the side cells anyway
        return (cx-Maze.TILESIZE,cy), (cx,cy-Maze.TILESIZE), (cx,cy+Maze.TILESIZE), (cx+Maze.TILESIZE,cy)

    @staticmethod
    def get_edge_tile(x,y):

        # Each of the parameter is True if the corresponding neighbouring
        # cells contains an obstacle.
        #import pdb;pdb.set_trace()
        up,down,left,right,upleft,upright,downleft,downright = Maze.get_surrounding_obstacles_raw(x,y)

        if up and down and left and right:
            return None
        if up and down and left and not right:
            return TILES[RIGHT]
        if up and down and not left and right:
            return TILES[LEFT]
        if up and not down and left and right:
            return TILES[BOTTOM]
        if not up and down and left and right:
            return TILES[TOP]
        if up and not down and not left and right:
            return TILES[BOTTOM_LEFT_CVX]
        if up and not down and left and not right:
            return TILES[BOTTOM_RIGHT_CVX]
        if not up and down and not left and right:
            return TILES[TOP_LEFT_CVX]
        if not up and down and left and not right:
            return TILES[TOP_RIGHT_CVX]

        if not up and not down and left and not right:
            return TILES[BOTTOM_TOP_RIGHT]
        if not up and not down and not left and right:
            return TILES[BOTTOM_TOP_LEFT]
        if not up and not down and left and right:
            return TILES[BOTTOM_TOP]
        if up and down and not left and not right:
            return TILES[LEFT_RIGHT]
        if not up and down and not left and not right:
            return TILES[TOP_LEFT_RIGHT]
        if up and not down and not left and not right:
            return TILES[BOTTOM_LEFT_RIGHT]



        return None

    @staticmethod
    def overlay_transparent(background, overlay, x, y):

        background_width = background.shape[1]
        background_height = background.shape[0]

        if x >= background_width or y >= background_height:
            return background

        h, w = overlay.shape[0], overlay.shape[1]

        if x + w > background_width:
            w = background_width - x
            overlay = overlay[:, :w]

        if y + h > background_height:
            h = background_height - y
            overlay = overlay[:h]

        if overlay.shape[2] < 4:
            overlay = np.concatenate(
                [
                    overlay,
                    np.ones((overlay.shape[0], overlay.shape[1], 1), dtype = overlay.dtype) * 255
                ],
                axis = 2,
            )

        overlay_image = overlay[..., :3]
        mask = overlay[..., 3:] / 255.0

        background[y:y+h, x:x+w] = (1.0 - mask) * background[y:y+h, x:x+w] + mask * overlay_image

        return background

    @staticmethod
    def render(robots):
        
        # make sure we always generate the same random pattern
        random.seed(1)

        img = np.zeros((Maze.height*Maze.TILESIZE*Maze.M2PX,Maze.width*Maze.TILESIZE*Maze.M2PX,3), np.uint8)
        img[:] = (140,178,250)


        for y in range(Maze.height):
            for x in range(Maze.width):
                px = x * Maze.TILESIZE * Maze.M2PX
                px2 = (x+1) * Maze.TILESIZE * Maze.M2PX
                py = (Maze.height - y - 1) * Maze.TILESIZE * Maze.M2PX
                py2 = (Maze.height - y) * Maze.TILESIZE * Maze.M2PX

                if Maze.is_obstacle(x*Maze.TILESIZE, y*Maze.TILESIZE):

                    #import pdb;pdb.set_trace()
                    tile = Maze.get_edge_tile(x,y)
                    if tile is not None:
                        img[py:py2,px:px2] = tile
                    else:
                        cv2.rectangle(img, (px, py), (px2,py2), (0,0,0),-1)
                else:
                    img[py:py2,px:px2] = random.choice(TILES[FLOOR])

        for name, robot in list(robots.items()):
            X=int(robot.x * Maze.M2PX)
            Y=int((Maze.height * Maze.TILESIZE - robot.y) * Maze.M2PX)

            for hit in robot.hitpoints:
                hx = int(hit[0] * Maze.M2PX)
                hy  = int(hit[1] * Maze.M2PX)
                cv2.line(img, (X,Y), (hx+X,-hy+Y) , (180,180,180), 1)
                cv2.circle(img, (hx+X,-hy+Y), 2,(220,220,220),-1)


            rotmat = cv2.getRotationMatrix2D((ROBOT.shape[0]/2, ROBOT.shape[1]/2), robot.theta * 180/pi, 1)
            rotated_robot = cv2.warpAffine(ROBOT, rotmat,ROBOT.shape[:2])
            #rotated_robot = ROBOT
            w,h,_ = rotated_robot.shape

            Maze.overlay_transparent(img, rotated_robot,X-w/2,Y-h/2)

            cv2.putText(img, name, (X, Y+int(Robot.RADIUS * Maze.M2PX) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 200, 180), 2) #font stroke

        return img

    @staticmethod
    def show(robots):

        img = Maze.render(robots)
        cv2.imshow('Maze',img)
        key = cv2.waitKey(1)

        if key == 10:
            import pdb;pdb.set_trace()
        if key == 83: # left
            robot.w -= 0.1
        if key == 81: # right
            robot.w += 0.1
        if key == 82: # up
            robot.v += 1
        if key == 84: # down
            robot.v -= 1
        if key == 9: # tab
            pass
        if key == 115: # s
            cv2.imwrite("map.png",img)
            with open("map.yaml",'w') as f:
                yaml.dump({'image':'map.png',
                           'resolution':1./Maze.M2PX,
                           'origin': [0.0, 0.0, 0.0],
                           'occupied_thresh': 0.65,
                           'free_thresh': 0.196,
                           'negate': 0},f,default_flow_style=False)
            print("ROS map generated (map.yaml + map.png)")



class Robot:
    
    RADIUS=0.25 * Maze.TILESIZE

    MAXLIFE=10
    def __init__(self, name="WallE", x=0., y=0., theta=0.):

        self.name = name
        self.base_frame = "%s_base_link" % self.name
    
        self.life = Robot.MAXLIFE

        self.lastinteraction = 0
        self.created = time.time()
        self.age = 0

        self.x = x
        self.y = y
        self.theta = theta

        self.v = 0.
        self.w = 0.

        # laser scan parameters
        self.angle_min = -45. * pi/180
        self.angle_max = 46. * pi/180
        self.angle_increment = 2. * pi/180
        self.range_min = 0. #m
        self.range_max = 60. #m

        self.ranges = []
        self.hitpoints = []

        self.br = tf.TransformBroadcaster()

        self.odom_pub = rospy.Publisher('%s/odom' % self.name, Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "%s_odom" % self.name
        self.odom_msg.child_frame_id = self.base_frame
        self.odom_msg.pose.pose.position.z = 0.
        self.odom_msg.pose.pose.orientation.x = 0.
        self.odom_msg.pose.pose.orientation.y = 0.

        self.scan_pub = rospy.Publisher('%s/scan' % self.name, LaserScan, queue_size=1)
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.base_frame
        self.scan_msg.angle_min = self.angle_min
        self.scan_msg.angle_max = self.angle_max
        self.scan_msg.angle_increment = self.angle_increment
        self.scan_msg.time_increment = 0.
        self.scan_msg.range_min = self.range_min
        self.scan_msg.range_max = self.range_max

        self.cmdvel_sub = rospy.Subscriber('%s/cmd_vel' % self.name, Twist, self.cmd_vel)

        self.last = time.time()

    def setpose(self, x, y, theta):
        self.x=x
        self.y=y
        self.theta=theta

    def cmd_vel(self,msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def step(self, publish_odom=False, publish_tf=False, publish_laser=False):

        now = time.time()
        dt = now - self.last
        self.last = now

        ntheta = self.theta + dt * self.w
        nx = self.x + dt * cos(ntheta) * self.v
        ny = self.y + dt * sin(ntheta) * self.v

        head_x, head_y = nx + cos(ntheta) * Robot.RADIUS, ny + sin(ntheta) * Robot.RADIUS


        # if we do not hit an obstacle, move and re-compute laserscan
        if not Maze.is_obstacle(head_x,head_y):
            self.theta = ntheta 
            self.x = nx
            self.y = ny

            self.ranges, self.hitpoints = self.raycasting()

        qx,qy,qz,qw = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        if publish_odom:
            self.odom_msg.header.stamp = rospy.Time.now() 
            self.odom_msg.pose.pose.position.x = self.x
            self.odom_msg.pose.pose.position.y = self.y

            self.odom_msg.pose.pose.orientation.z = qz
            self.odom_msg.pose.pose.orientation.w = qw

            self.odom_msg.twist.twist.linear.x = self.v
            self.odom_msg.twist.twist.angular.z = self.w

            self.odom_pub.publish(self.odom_msg)

        if publish_tf:

            self.br.sendTransform((self.x, self.y, 0),
                                  (qx,qy,qz,qw),
                                  rospy.Time.now(),
                                  self.base_frame,
                                  "%s_odom" % self.name)

        if publish_laser:
            self.scan_msg.scan_time = dt
            self.scan_msg.ranges = self.ranges

            self.scan_pub.publish(self.scan_msg)

    @staticmethod
    def line_intersection(line1, line2):

        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            return None

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y


    @staticmethod
    def ccw(A,B,C):
        Ax, Ay = A
        Bx, By = B
        Cx, Cy = C
        return (Cy-Ay) * (Bx-Ax) > (By-Ay) * (Cx-Ax)

    @staticmethod
    def segment_intersect(A,B,C,D):
        """ Return true if line segments AB and CD intersect
        """
        return Robot.ccw(A,C,D) != Robot.ccw(B,C,D) and Robot.ccw(A,B,C) != Robot.ccw(A,B,D)


    def get_ray_point(self, d, angular_offset=0.):
        return self.x + cos(self.theta + angular_offset) * d, self.y + sin(self.theta + angular_offset) * d

    def dist_to_robot(self, point):
        px,py = point
        return (self.x-px) * (self.x-px) + (self.y-py)*(self.y-py)


    def cell_intersection(self, maze_x, maze_y, alpha=0., debug_img=None):

        A = maze_x, maze_y
        B = (maze_x + Maze.TILESIZE),maze_y
        C = (maze_x + Maze.TILESIZE), (maze_y + Maze.TILESIZE)
        D = maze_x, (maze_y + Maze.TILESIZE)

        next_cell = None

        if debug_img is not None:
            cv2.line(debug_img, A, B , (150,250,150), 1)
            cv2.line(debug_img, C, B , (150,250,150), 1)
            cv2.line(debug_img, A, D , (150,250,150), 1)
            cv2.line(debug_img, C, D , (150,250,150), 1)


        R0 = self.x, self.y
        R1 = self.get_ray_point(self.range_max, angular_offset=alpha)

        intersections = {}

        for s,direction in [((A,B),"up"), ((B,C),"right"), ((C,D),"down"), ((D,A),"left")]:
            intersect = Robot.segment_intersect(R0, R1, *s)
            if intersect:
                intersection = Robot.line_intersection(s, (R0, R1))
                intersections[self.dist_to_robot(intersection)] = (intersection, direction)
                if debug_img is not None:
                    px,py = intersection
                    cv2.circle(debug_img, (int(px),int(py)), 3,(10,50,10),-1)

        # the ray does not cross this cell
        if not intersections:
            return None

        max_dist = max(intersections.keys())
        min_dist = min(intersections.keys())

        # our ray did not traversed the cell: out of range!
        if len(intersections) != 2:
            return self.range_max*self.range_max + 1, None

        # the ray crosses the cell, but the cell is empty. Recursively looks for a wall
        if not Maze.is_obstacle(maze_x, maze_y):
            intersection, direction = intersections[max_dist]
            next_maze_x, next_maze_y = maze_x, maze_y 
            if direction == "up":
                next_maze_y -= Maze.TILESIZE
            if direction == "down":
                next_maze_y += Maze.TILESIZE
            if direction == "right":
                next_maze_x += Maze.TILESIZE
            if direction == "left":
                next_maze_x -= Maze.TILESIZE

            return self.cell_intersection(next_maze_x, next_maze_y, alpha, debug_img)

        # the ray hits the cell, which is an obstacle. Returns the distance + coordinate relative to robot.
        x,y = intersections[min_dist][0]
        return min_dist, (x-self.x, y-self.y)



    def raycasting(self, debug_img = None):
        rays = np.arange(self.angle_min, self.angle_max, self.angle_increment)
        #rays=[-0.3]

        ranges = []
        hitpoints = []
        for alpha in rays:
            for maze_x, maze_y in Maze.get_neighbour_cells(self.x, self.y):
                hit = self.cell_intersection(maze_x, maze_y, alpha, debug_img)
                if hit is not None:
                    dist, point = hit
                    ranges.append(sqrt(dist))
                    if point is not None: # might be None if we are out of the range of the laser
                        hitpoints.append(point)
                    break
        
        #print(ranges)

        return ranges, hitpoints


for k,v in TILES.items():
    if type(TILES[k]) == list:
        TILES[k] = [cv2.resize(t,(Maze.TILESIZE*Maze.M2PX, Maze.TILESIZE*Maze.M2PX)) for t in TILES[k]]
    else:
        TILES[k] = cv2.resize(TILES[k],(Maze.TILESIZE*Maze.M2PX, Maze.TILESIZE*Maze.M2PX))

ROBOT = cv2.resize(ROBOT,(int(2*Robot.RADIUS*Maze.M2PX), int(2*Robot.RADIUS*Maze.M2PX)))

if __name__ == "__main__":

    rospy.init_node("robomaze")

    robots = {}

    spawn_idx = 0
    SPAWN_POINTS = [(1.5, 1.47, 2),
                    (16.8, 17.5, -102),
                    (12.8, 15.8, -90),
                    (7.4, 11.5, -2),
                    (10.4, 10.5, 102),
                    (7.6, 6.2, -38),
                    (14.1, 9.2, 193),
                    (1.6, 5.2, 34),
                    (2.1, 10.4, -22),
                    (2.4, 18.4, 22),
                    ]
    def on_new_robot(msg):
        global spawn_idx

        name = msg.data

        if name in robots:
            rospy.logerr("Robot <%s> already exists!" % name)
            return
        
        x,y,t = SPAWN_POINTS[spawn_idx]
        spawn_idx = (spawn_idx + 1) % len(SPAWN_POINTS)
        robots[name] = Robot(name, x * Maze.TILESIZE,y * Maze.TILESIZE, t * pi/180.)

    new_robot_sub = rospy.Subscriber("create_robot", String, on_new_robot)
    #new_robot_service = rospy.Service('create_robot', String, on_new_robot)

    last = time.time()

    rate = rospy.Rate(10) # 10hz

    #cv2.namedWindow("Maze", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Maze", cv2.WINDOW_AUTOSIZE)

    while not rospy.is_shutdown():


        for name,robot in list(robots.items()):
            robot.step(publish_odom=True, publish_tf=True, publish_laser=True)

        Maze.show(robots)

        rate.sleep()

    cv2.destroyAllWindows()

