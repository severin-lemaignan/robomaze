import time
import cv2
import numpy as np
from math import cos, sin,pi,floor,sqrt

import rospy
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf

HEIGHT= 7
WIDTH = 6
TILESIZE=10 #m
M2PX=10 # TILESIZE[m] * M2PX = TILESIZE[px]


MAZE = [1,1,1,1,1,1,
        1,0,0,0,0,1,
        1,0,0,0,1,1,
        1,0,0,0,0,1,
        1,0,0,0,1,1,
        1,1,1,0,0,1,
        1,1,1,1,1,1]

class Robot:
    def __init__(self, name="WallE", x=0., y=0., theta=0.):

        self.name = name
        self.base_frame = "%s_base_link" % self.name

        self.x = x
        self.y = y
        self.theta = theta

        self.v = 0.
        self.w = 0.

        # laser scan parameters
        self.angle_min = -45. * pi/180
        self.angle_max = 46. * pi/180
        self.angle_increment = 5. * pi/180
        self.range_min = 0. #m
        self.range_max = 50. #m

        self.br = tf.TransformBroadcaster()

#        self.odom_pub = rospy.Publisher('%s/odom' % self.name, Odometry, queue_size=1)
#        self.odom_msg = Odometry()
#        self.odom_msg.header.frame_id = "map"
#        self.odom_msg.child_frame_id = "%s_odom" % self.name
#        self.odom_msg.pose.pose.position.z = 0.
#        self.odom_msg.pose.pose.orientation.x = 0.
#        self.odom_msg.pose.pose.orientation.y = 0.

        self.scan_pub = rospy.Publisher('%s/scan' % self.name, LaserScan, queue_size=1)
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.base_frame
        self.scan_msg.angle_min = self.angle_min
        self.scan_msg.angle_max = self.angle_max
        self.scan_msg.angle_increment = self.angle_increment
        self.scan_msg.time_increment = 0.
        self.scan_msg.range_min = self.range_min
        self.scan_msg.range_max = self.range_max


    def cmd_vel(self,v,w):
        self.v = v
        self.w = w

    def step(self,dt):

        self.theta += dt * self.w
        self.x += dt * cos(self.theta) * self.v
        self.y += dt * sin(self.theta) * self.v

#        self.odom_msg.header.stamp = rospy.Time.now() 
#        self.odom_msg.pose.pose.position.x = self.x
#        self.odom_msg.pose.pose.position.y = self.y
#        self.odom_msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, self.theta)
#        #self.odom_msg.pose.pose.orientation.z = sin(self.theta)
#        #self.odom_msg.pose.pose.orientation.w = cos(self.theta)
#
#        self.odom_pub.publish(self.odom_msg)

        self.br.sendTransform((self.x, self.y, 0),
                tf.transformations.quaternion_from_euler(0, 0, self.theta),
                rospy.Time.now(),
                self.base_frame,
                "odom")

        self.ranges, self.hitpoints = self.raycasting()

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
    def is_obstacle(x_px,y_px):

        x = int(floor(round(x_px)/TILESIZE))
        y = int(floor(round(y_px)/TILESIZE))

        if x >= 0 and y >= 0 and x < WIDTH and y < HEIGHT and not MAZE[x + y * WIDTH]:
            return False
        return True

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
        B = (maze_x + TILESIZE),maze_y
        C = (maze_x + TILESIZE), (maze_y + TILESIZE)
        D = maze_x, (maze_y + TILESIZE)

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

        max_dist = max(intersections.iterkeys())
        min_dist = min(intersections.iterkeys())

        # our ray did not traversed the cell: out of range!
        if len(intersections) != 2:
            return self.range_max + 1, None

        # the ray crosses the cell, but the cell is empty. Recursively looks for a wall
        if not Robot.is_obstacle(maze_x, maze_y):
            intersection, direction = intersections[max_dist]
            next_maze_x, next_maze_y = maze_x, maze_y 
            if direction == "up":
                next_maze_y -= TILESIZE
            if direction == "down":
                next_maze_y += TILESIZE
            if direction == "right":
                next_maze_x += TILESIZE
            if direction == "left":
                next_maze_x -= TILESIZE

            return self.cell_intersection(next_maze_x, next_maze_y, alpha, debug_img)

        # the ray hits the cell, which is an obstacle. Returns the distance + coordinate relative to robot.
        x,y = intersections[min_dist][0]
        return min_dist, (x-self.x, y-self.y)


    @staticmethod
    def get_neighbour_cells(x,y):
        cx, cy = int(floor(round(x)*1./TILESIZE)) * TILESIZE, int(floor(round(y)*1./TILESIZE)) * TILESIZE

        # note: no need to include the diagonals - they will be visited from the side cells anyway
        return (cx-TILESIZE,cy), (cx,cy-TILESIZE), (cx,cy+TILESIZE), (cx+TILESIZE,cy)

    def raycasting(self, debug_img = None):
        rays = np.arange(robot.angle_min, robot.angle_max, robot.angle_increment)
        #rays=[-0.3]

        ranges = []
        hitpoints = []
        for alpha in rays:
            for maze_x, maze_y in self.get_neighbour_cells(self.x, self.y):
                hit = self.cell_intersection(maze_x, maze_y, alpha, debug_img)
                if hit is not None:
                    dist, point = hit
                    ranges.append(sqrt(dist))
                    if point is not None: # might be None if we are out of the range of the laser
                        hitpoints.append(point)
                    break
        
        #print(ranges)

        return ranges, hitpoints


def showmaze():

    show_maze = True

    last = time.time()

    while True:

        now = time.time()
        dt = now - last
        robot.step(dt)

        last = now

        img = np.zeros((HEIGHT*TILESIZE*M2PX,WIDTH*TILESIZE*M2PX,3), np.uint8)
        img[:] = (255,255,255)

        if show_maze:
            for i in range(HEIGHT):
                for j in range(WIDTH):
                    if MAZE[j + WIDTH * i]:
                        cv2.rectangle(img, (j*TILESIZE*M2PX, i*TILESIZE*M2PX), 
                                        ((j+1)*TILESIZE*M2PX, (i+1)*TILESIZE*M2PX), 
                                        (128,0,255),-1)

        X=int(robot.x * M2PX)
        Y=int(robot.y * M2PX)

        cv2.circle(img, (X,Y), int(TILESIZE*M2PX/4),(255,128,0),-1)
        cv2.line(img, (X,Y), (int(X + cos(robot.theta) * TILESIZE*M2PX/2), int(Y + sin(robot.theta) * TILESIZE*M2PX/2)), (255,255,0), 3)

        for hit in robot.hitpoints:
            hx, hy = int(hit[0] * M2PX), int(hit[1] * M2PX)
            cv2.line(img, (X,Y), (hx+X,hy+Y) , (200,200,200), 1)
            cv2.circle(img, (hx+X,hy+Y), 5,(10,10,10),-1)



        cv2.imshow('Maze',img)
        key = cv2.waitKey(15)

        if key == 27:
            break
        if key == 83: # left
            robot.w += 0.1
        if key == 81: # right
            robot.w -= 0.1
        if key == 82: # up
            robot.v += 1
        if key == 84: # down
            robot.v -= 1
        if key == 9: # tab
            show_maze = not show_maze

    cv2.destroyAllWindows()


if __name__ == "__main__":

    rospy.init_node("robomaze")

    robot = Robot("WallE", 1.5 * TILESIZE,1.47 * TILESIZE, 46 * pi/180)

    showmaze()


