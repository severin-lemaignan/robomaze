import cv2
import numpy as np
from math import cos, sin,pi,floor

HEIGHT= 7
WIDTH = 6
TILESIZE=100

maze = [1,1,1,1,1,1,
        1,0,0,0,0,1,
        1,0,0,0,1,1,
        1,0,0,0,0,1,
        1,0,0,0,1,1,
        1,1,1,0,0,1,
        1,1,1,1,1,1]

robot_x,robot_y,robot_theta = 1.5 * TILESIZE,1.47 * TILESIZE, 46 * pi/180

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

def is_obstacle(x_px,y_px):

    x = int(floor(round(x_px)/TILESIZE))
    y = int(floor(round(y_px)/TILESIZE))

    if x >= 0 and y >= 0 and x < WIDTH and y < HEIGHT and not maze[x + y * WIDTH]:
        return False
    return True

def ccw(A,B,C):
    Ax, Ay = A
    Bx, By = B
    Cx, Cy = C
    return (Cy-Ay) * (Bx-Ax) > (By-Ay) * (Cx-Ax)

# Return true if line segments AB and CD intersect
def segment_intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def get_ray_point(x, y, theta, d):
    return x + cos(theta) * d, y + sin(theta) * d

def dist_to_robot(point):
    px,py = point

    return (robot_x-px) * (robot_x-px) + (robot_y-py)*(robot_y-py)


def cell_intersection(maze_x,maze_y, x, y, theta, max_dist, debug_img = None):

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


    R0 = x,y
    R1 = get_ray_point(x,y,theta, max_dist)

    intersections = {}

    for s,direction in [((A,B),"up"), ((B,C),"right"), ((C,D),"down"), ((D,A),"left")]:
        intersect = segment_intersect(R0, R1, *s)
        if intersect:
            intersection = line_intersection(s, (R0, R1))
            intersections[dist_to_robot(intersection)] = (intersection, direction)
            if debug_img is not None:
                px,py = intersection
                cv2.circle(debug_img, (int(px),int(py)), 3,(10,50,10),-1)

    # the ray does not cross this cell
    if not intersections:
        return None

    max_dist = max(intersections.iterkeys())
    min_dist = min(intersections.iterkeys())

    if len(intersections) != 2:
        return intersections[min_dist][0]

    # the ray crosses the cell, but the cell is empty. Recursively looks for a wall
    if not is_obstacle(maze_x, maze_y):
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

        return cell_intersection(next_maze_x, next_maze_y,x,y,theta,max_dist, debug_img)

    # the ray hits the cell, which is an obstacle. Returns the coordinate.
    return intersections[min_dist][0]

    
        
def get_neighbour_cells(x,y):
    cx, cy = int(floor(round(x)*1./TILESIZE)) * TILESIZE, int(floor(round(y)*1./TILESIZE)) * TILESIZE

    # note: no need to include the diagonals - they will be visited from the side cells anyway
    return (cx-TILESIZE,cy), (cx,cy-TILESIZE), (cx,cy+TILESIZE), (cx+TILESIZE,cy)

def raycasting(x,y,theta, max_dist = 6 * TILESIZE, debug_img = None):
    rays = np.arange(-45 * pi/180,46 * pi/180,5 * pi/180)
    #rays=[-0.3]

    hitpoints = []
    for alpha in rays:
        for maze_x, maze_y in get_neighbour_cells(x, y):
            hit = cell_intersection(maze_x, maze_y, x, y, theta + alpha, max_dist, debug_img)
            if hit:
                hitpoints.append(hit)
                break
    
    return hitpoints


def showmaze():
    global robot_x, robot_y, robot_theta

    show_maze = True
    while True:
        img = np.zeros((HEIGHT*TILESIZE,WIDTH*TILESIZE,3), np.uint8)
        img[:] = (255,255,255)

        if show_maze:
            for i in range(HEIGHT):
                for j in range(WIDTH):
                    if maze[j + WIDTH * i]:
                        cv2.rectangle(img, (j*TILESIZE, i*TILESIZE), 
                                        ((j+1)*TILESIZE, (i+1)*TILESIZE), 
                                        (128,0,255),-1)

        X=int(robot_x)
        Y=int(robot_y)

        cv2.circle(img, (X,Y), TILESIZE/4,(255,128,0),-1)
        cv2.line(img, (X,Y), (int(X + cos(robot_theta) * TILESIZE/2), int(Y + sin(robot_theta) * TILESIZE/2)), (255,255,0), 3)

        hitpoints = raycasting(robot_x, robot_y, robot_theta) #, debug_img=img)
        for hit in hitpoints:
            hx, hy = int(hit[0]), int(hit[1])
            cv2.line(img, (X,Y), (hx,hy) , (200,200,200), 1)
            cv2.circle(img, (hx,hy), 5,(10,10,10),-1)



        cv2.imshow('Maze',img)
        key = cv2.waitKey()
        print(key)
        if key == 27:
            break
        if key == 83: # left
            robot_theta += 0.1
        if key == 81: # right
            robot_theta -= 0.1
        if key == 82: # up
            robot_x += 10 * cos(robot_theta)
            robot_y += 10 * sin(robot_theta)
        if key == 84: # down
            robot_x += -10 * cos(robot_theta)
            robot_y += -10 * sin(robot_theta)
        if key == 9: # tab
            show_maze = not show_maze


    cv2.destroyAllWindows()


showmaze()


