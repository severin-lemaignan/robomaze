#! /usr/bin/env python3

from os import stat
import yaml
import time
import random
import cv2
import numpy as np
from math import tanh, cos, sin, pi, floor, sqrt

RES_ROOT = "res/"
FLOOR = 1
TOP = 2
BOTTOM = 3
LEFT = 4
RIGHT = 5
TOP_LEFT_CVX = 6
TOP_LEFT_CCV = 7
TOP_RIGHT_CVX = 8
TOP_RIGHT_CCV = 9
BOTTOM_LEFT_CVX = 10
BOTTOM_LEFT_CCV = 11
BOTTOM_RIGHT_CVX = 12
BOTTOM_RIGHT_CCV = 13

BOTTOM_TOP_RIGHT = 14
BOTTOM_TOP_LEFT = 15
BOTTOM_LEFT_RIGHT = 16
TOP_LEFT_RIGHT = 17
BOTTOM_TOP = 18
LEFT_RIGHT = 19

ROBOT = cv2.imread(RES_ROOT + "walle_top.png", cv2.IMREAD_UNCHANGED)
ROBOT_ALPHA = ROBOT[:, :, 3]
ROBOT_RGB = ROBOT[:, :, :3]

FREE_SPACE = "free_space"
RFID_TAG = "rfid_tag"

RFID_RANGE = 10

TILES = {
        FLOOR: [cv2.imread(RES_ROOT + "floor2.jpg"), cv2.imread(RES_ROOT + "floor.jpg"), cv2.imread(RES_ROOT + "floor.jpg"), cv2.imread(RES_ROOT + "floor.jpg")],
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

        BOTTOM_TOP_RIGHT: cv2.imread(RES_ROOT + "bottom_top_right.jpg"),
        BOTTOM_TOP_LEFT: cv2.imread(RES_ROOT + "bottom_top_left.jpg"),
        BOTTOM_LEFT_RIGHT: cv2.imread(RES_ROOT + "bottom_left_right.jpg"),
        TOP_LEFT_RIGHT: cv2.imread(RES_ROOT + "top_left_right.jpg"),
        BOTTOM_TOP: cv2.imread(RES_ROOT + "bottom_top.jpg"),
        LEFT_RIGHT: cv2.imread(RES_ROOT + "left_right.jpg"),
        }


class Maze:
    height = 20  # in tiles
    width = 20  # in tiles
    TILESIZE = 2  # m
    M2PX = 16  # TILESIZE[m] * M2PX = TILESIZE[px]

    #  y=height
    #  |
    #  |
    #  x=0,y=0 -------------->   x=width
    #
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0,
            0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0,
            0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0,
            0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 3, 1, 1, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 5, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 1, 0, 1, 2, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 4, 0, 1, 1, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0,
            0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    @staticmethod
    def is_obstacle(x_m, y_m):
        """x_m and y_m must be provided in meters
        """

        x = int(floor(round(x_m)/Maze.TILESIZE))
        y = int(floor(round(y_m)/Maze.TILESIZE))

        return Maze.is_obstacle_raw(x, y)

    @staticmethod
    def is_obstacle_raw(x, y):

        if x >= 0 and \
           y >= 0 and \
           x < Maze.width and \
           y < Maze.height and \
           Maze.data[x + (Maze.height - 1 - y) * Maze.width]:
              # print(
              #     f"Maze data at ({x}, {y}): {Maze.data[x + (Maze.height - y - 1) * Maze.width]}")
               return False
        return True

    @staticmethod
    def get_item(x_m,y_m):
        """x_m and y_m must be provided in meters
        """

        x = int(floor(round(x_m)/Maze.TILESIZE))
        y = int(floor(round(y_m)/Maze.TILESIZE))

        return Maze.get_item_raw(x,y)

    @staticmethod
    def get_item_raw(x,y):
        """x_m and y_m must be provided in meters
        """

        if x >= 0 and \
           y >= 0 and \
           x < Maze.width and \
           y < Maze.height:
                id = Maze.data[x + (Maze.height - y - 1) * Maze.width]

                if id == 1:
                    return FREE_SPACE, None
                else:
                    return RFID_TAG, "rfid_%s" % id

        return None, None

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
        return Maze.ccw(A,C,D) != Maze.ccw(B,C,D) and Maze.ccw(A,B,C) != Maze.ccw(A,B,D)


    @staticmethod
    def get_ray_point(x,y, theta, dist):
        return x + cos(theta) * dist, y + sin(theta) * dist

    @staticmethod
    def dist(a, b):
        x,y=a
        px,py = b
        return (x-px) * (x-px) + (y-py)*(y-py)

    @staticmethod
    def to_px(p):
        """
        Converts a point in meters into pixel coordinates, with (0, 0) at the bottom left, Y up, X right
        """
        x, y = p
        return int(x*Maze.M2PX), int((Maze.height * Maze.TILESIZE - y) * Maze.M2PX - 1)

    @staticmethod
    def cell_intersection(x, y, theta, maze_x, maze_y, max_range=10, debug_img=None):

        #print(f"Looking for intersections with cell at {maze_x}, {maze_y} from {x},{y}")

        A = maze_x, maze_y
        B = (maze_x + Maze.TILESIZE),maze_y
        C = (maze_x + Maze.TILESIZE), (maze_y + Maze.TILESIZE)
        D = maze_x, (maze_y + Maze.TILESIZE)

        if debug_img is not None:
            print(f"Cell ({maze_x},{maze_y}) is obstacle: {Maze.is_obstacle(maze_x, maze_y)}")

            cv2.line(debug_img, Maze.to_px(A), Maze.to_px(B) , (150,250,150), 1)
            cv2.line(debug_img, Maze.to_px(C), Maze.to_px(B) , (150,250,150), 1)
            cv2.line(debug_img, Maze.to_px(A), Maze.to_px(D) , (150,250,150), 1)
            cv2.line(debug_img, Maze.to_px(C), Maze.to_px(D) , (150,250,150), 1)


        R0 = x, y
        R1 = Maze.get_ray_point(x,y,theta,max_range)

        if debug_img is not None:
            cv2.line(debug_img, Maze.to_px(R0), Maze.to_px(R1) , (150,150,250), 1)

        intersections = {}

        for s,direction in [((A,B),"up"), ((B,C),"right"), ((C,D),"down"), ((D,A),"left")]:
            intersect = Maze.segment_intersect(R0, R1, *s)
            if intersect:
                intersection = Maze.line_intersection(s, (R0, R1))
                intersections[Maze.dist((x,y), intersection)] = (intersection, direction)
                if debug_img is not None:
                    print(f"Ray ({x},{y},theta={theta*180/pi}) -> Intersection with cell ({maze_x},{maze_y}) at d={sqrt(Maze.dist((x,y), intersection))}m")
                    cv2.circle(debug_img, Maze.to_px(intersection), 3,(150,150,150),-1)
                    cv2.imshow('Maze',debug_img)
                    cv2.waitKey()



        # the ray does not cross this cell
        if not intersections:
            return None

        max_dist = max(intersections.keys())
        min_dist = min(intersections.keys())

        # less than 2 intersections? our ray did not fully traverse the cell: out of range!
        if len(intersections) == 1:
            return max_range*max_range + 1, None

        #print(f"Checking if {maze_x},{maze_y} is obstacle...")
        if Maze.is_obstacle(maze_x, maze_y):

            # the cell is an obstacle. Returns the distance + coordinate relative to robot.

            hx,hy = intersections[min_dist][0]
            if debug_img is not None:
                #print(f"Yes. Keeping point at d={sqrt(min_dist)}m")
                cv2.circle(debug_img, Maze.to_px((hx,hy)), 3,(10,50,10),-1)

            return min_dist, (hx-x, hy-y)


        else:
            #print(f"No. Continuing")
            # the ray crosses the cell, but the cell is empty. Recursively looks for a wall
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

            return Maze.cell_intersection(x,y, theta, next_maze_x, next_maze_y, max_range, debug_img)


    @staticmethod
    def raycasting(x,y,theta, max_range=10, debug_img = None):


        ranges = []
        hitpoints = []
        for maze_x, maze_y in Maze.get_neighbour_cells(x, y):
            hit = Maze.cell_intersection(x,y,theta,maze_x, maze_y, max_range, debug_img)
            if hit is not None:
                dist, point = hit
                ranges.append(sqrt(dist))
                if point is not None: # might be None if we are out of the range of the laser
                    hitpoints.append(point)
                break

        return ranges, hitpoints


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

        rfid_overlay = np.zeros((Maze.height*Maze.TILESIZE*Maze.M2PX,Maze.width*Maze.TILESIZE*Maze.M2PX,4), np.uint8)
        rfid_overlay[:] = (0,0,0,0)

        rfid_power = np.zeros((Maze.height*Maze.TILESIZE*Maze.M2PX,Maze.width*Maze.TILESIZE*Maze.M2PX,4), np.uint8)
        rfid_power[:] = (0,0,0,0)


        for Y in range(Maze.height):
            for X in range(Maze.width):
                x = X * Maze.TILESIZE # in meters
                y = Y * Maze.TILESIZE # in meters
                px = X * Maze.TILESIZE * Maze.M2PX # in pixels
                w = Maze.TILESIZE * Maze.M2PX # in pixels
                px2 = (X+1) * Maze.TILESIZE * Maze.M2PX
                py = (Maze.height - Y - 1) * Maze.TILESIZE * Maze.M2PX
                py2 = (Maze.height - Y) * Maze.TILESIZE * Maze.M2PX
                #print(f"Tile at ({x}m,{y}m)")

                if Maze.is_obstacle(x,y):

                    tile = Maze.get_edge_tile(X,Y)
                    if tile is not None:
                        img[py:py2,px:px2] = tile
                    else:
                        cv2.rectangle(img, (px, py), (px2,py2), (0,0,0),-1)
                else:

                    item_type, name = Maze.get_item(x, y)

                    if item_type == FREE_SPACE:
                        #img[py:py2,px:px2] = random.choice(TILES[FLOOR])
                        pass
                    else: # RFID tag!
                        #img[py:py2,px:px2] = random.choice(TILES[FLOOR])

                        rfid_x, rfid_y = x + Maze.TILESIZE/2, y + Maze.TILESIZE/2 # in meters
                        rfid_X, rfid_Y = Maze.to_px((rfid_x,rfid_y))

                        rfid_hitpoints = []
                        for theta in np.arange(0, 2*pi, 2*pi/10):
                            d, hitpoint = Maze.raycasting(rfid_x, rfid_y,theta,max_range=RFID_RANGE,debug_img=None)
                            rfid_hitpoints += hitpoint
                        for hit in rfid_hitpoints:
                                hx = int(hit[0] * Maze.M2PX)
                                hy  = int(hit[1] * Maze.M2PX)
                                #cv2.line(img, (rfid_X,rfid_Y), (hx+rfid_X,-hy+rfid_Y) , (180,180,180), 1)
                                #cv2.circle(img, (hx+rfid_X,-hy+rfid_Y), 2,(220,220,220),-1)


                        #cv2.circle(rfid_overlay, (rfid_X, rfid_Y), RFID_RANGE * Maze.M2PX,(200,100,50,70),-1)
                        rfid = RFID(rfid_x, rfid_y)
                        rfid.sample(rfid_power)
                        cv2.putText(rfid_overlay, name, (rfid_X - 20, rfid_Y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 100, 80,200), 1) #font stroke

                        cv2.circle(img, (rfid_X, rfid_Y), 5,(100,100,220),-1)

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

            Maze.overlay_transparent(img, rotated_robot,X-int(w/2),Y-int(h/2))

            cv2.putText(img, name, (X, Y+int(Robot.RADIUS * Maze.M2PX) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 200, 180), 2) #font stroke

        Maze.overlay_transparent(img, rfid_power,0,0)
        Maze.overlay_transparent(img, rfid_overlay,0,0)

        return img

    @staticmethod
    def show(robots):

        img = Maze.render(robots)
        cv2.imshow('Maze',img)


class RFID:

    MAX_ATTENUATION=60 #dB

    def __init__(self, x, y, max_range=10):
        self.x=x # in meters
        self.y=y # in meters
        self.range = max_range

    def power(self, x, y):
        dist = sqrt((x-self.x)*(x-self.x) + (y-self.y)*(y-self.y))

        if dist > self.range:
            return -RFID.MAX_ATTENUATION

        # attenuation function (limit -> -MAX_ATTENUATION dB)
        p = -RFID.MAX_ATTENUATION*tanh(dist/3)
        return p

    def sample(self, img,grid_size=0.5):

        for dx in np.arange(self.x - self.range, self.x + self.range, grid_size):
            for dy in np.arange(self.y - self.range, self.y + self.range, grid_size):
                if dx < 0 or dy < 0 or dx >= Maze.width * Maze.TILESIZE or dy >= Maze.height * Maze.TILESIZE:
                    continue

                p = self.power(dx,dy)

                if p > -RFID.MAX_ATTENUATION:

                    c = int((1 - (-p/RFID.MAX_ATTENUATION)) * 200)

                    if c > img[Maze.to_px((dx,dy))][3]:
                        #if img[Maze.to_px((dx,dy))][3] > 0:
                        #    print(f"old c = {img[Maze.to_px((dx,dy))][3]}, new c = {c}")
                        cv2.rectangle(img, Maze.to_px((dx-grid_size/2, dy-grid_size/2)), Maze.to_px((dx+grid_size/2,dy+grid_size/2)), (0,50,150,c),-1)
    

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

        self.last = time.time()

    def setpose(self, x, y, theta):
        self.x=x
        self.y=y
        self.theta=theta

    def cmd_vel(self,msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def step(self):

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
    def on_new_robot(name):
        global spawn_idx


        if name in robots:
            print("Robot <%s> already exists!" % name)
            return
        
        x,y,t = SPAWN_POINTS[spawn_idx]
        spawn_idx = (spawn_idx + 1) % len(SPAWN_POINTS)
        robots[name] = Robot(name, x * Maze.TILESIZE,y * Maze.TILESIZE, t * pi/180.)

    DEFAULT_ROBOT = "robot"
    on_new_robot(DEFAULT_ROBOT)

    last = time.time()

    #cv2.namedWindow("Maze", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Maze", cv2.WINDOW_AUTOSIZE)

    while True:

        key = cv2.waitKey(16)

        robot = robots[DEFAULT_ROBOT]
        if key > 0:
            if key == 27:
                break
            if key == 82: # up
                robot.v = 3.0
            if key == 84: # down
                robot.v = -3.0

            if key == 81: # left
                robot.w = 0.8
            if key == 83: # right
                robot.w = -0.8
            if key == 115: # s
                cv2.imwrite("map.png",Maze.render(robots))
                with open("map.yaml",'w') as f:
                    yaml.dump({'image':'map.png',
                            'resolution':1./Maze.M2PX,
                            'origin': [0.0, 0.0, 0.0],
                            'occupied_thresh': 0.65,
                            'free_thresh': 0.196,
                            'negate': 0},f,default_flow_style=False)
                print("ROS map generated (map.yaml + map.png)")

        else:
            robot.v = 0
            robot.w = 0


        for name,robot in list(robots.items()):
            robot.step()

        Maze.show(robots)


    cv2.destroyAllWindows()

