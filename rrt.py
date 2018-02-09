import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
from random import *


'''
# Set up matplotlib to create a plot with an empty square

def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    # ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax
'''


def setupPlot():
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    return fig, ax


# Make a patch for a single pology
def createPolygonPatch(polygon, color):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch


# Render the problem
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)
    plt.show()


# distance formula
def dist2(vx, wx, vy, wy):
    return np.square(vx - wx) + np.square(vy - wy)


def CollissionFreeLine((px, py), (x_min, y_min), robot, obstacles):
    verts = [(px, py), (x_min, y_min), (px, py)]
    codes = [Path.MOVETO, Path.LINETO, Path.CLOSEPOLY]

    line_path = Path(verts, codes)
    obstacle_paths = []
    for j in range(len(obstacles)):
        verts_o = []
        codes_o = []
        for i in range(len(obstacles[j])):
            verts_o.append(obstacles[j][i])
            if i == 0:
                codes_o.append(Path.MOVETO)
            else:
                codes_o.append(Path.LINETO)
        verts_o.append(verts_o[0])
        codes_o.append(Path.CLOSEPOLY)
        obstacle_paths.append(Path(verts_o, codes_o))

    for i in range(len(obstacle_paths)):
        if line_path.intersects_path(obstacle_paths[i], filled=True):
            return False
        else:
            continue
    return True


def findPoints(px, py, pi, points, adjListMap):
    min_dist = 1000
    x_min = -5
    y_min = -5
    i_min = -1
    j_min = -1
    for i in adjListMap:
        if adjListMap[i]:
            wx = points[i][0]
            wy = points[i][1]
            for j in adjListMap[i]:
                vx = points[j][0]
                vy = points[j][1]
                l2 = dist2(vx, wx, vy, wy)
                if(l2 == 0):
                    x = float(format(vx, '.1f'))
                    y = float(format(vy, '.1f'))
                else:
                    t = ((px - vx) * (wx - vx) + (py - vy) * (wy - vy)) / l2
                    tmin = np.minimum(1, t)
                    t = np.maximum(0, tmin)
                    x = float(format((vx + t * (wx - vx)), '.1f'))
                    y = float(format((vy + t * (wy - vy)), '.1f'))
                distxp = dist2(px, x, py, y)
                if(distxp < min_dist):
                    min_dist = distxp
                    x_min = x
                    y_min = y
                    i_min = i
                    j_min = j

    if(points[i_min] == (x_min, y_min)):
        adjListMap[i_min].append(pi)
        adjListMap[pi].append(i_min)
        return points, adjListMap
    elif(points[j_min] == (x_min, y_min)):
        adjListMap[j_min].append(pi)
        adjListMap[pi].append(j_min)
        return points, adjListMap
    else:
        m = len(points) + 1
        points[m] = (x_min, y_min)
        adjListMap[m] = [pi]
        adjListMap[pi].append(m)
        adjListMap[i_min].append(m)
        adjListMap[j_min].append(m)
        adjListMap[m].append(i_min)
        adjListMap[m].append(j_min)
        adjListMap[i_min].remove(j_min)
        adjListMap[j_min].remove(i_min)
        return points, adjListMap


def findPoints2(px, py, pi, points, adjListMap, robot, obstacles):
    min_dist = 1000
    x_min = -5
    y_min = -5
    i_min = -1
    j_min = -1
    for i in adjListMap:
        if adjListMap[i]:
            wx = points[i][0]
            wy = points[i][1]
            for j in adjListMap[i]:
                vx = points[j][0]
                vy = points[j][1]
                l2 = dist2(vx, wx, vy, wy)
                if(l2 == 0):
                    x = float(format(vx, '.1f'))
                    y = float(format(vy, '.1f'))
                else:
                    t = ((px - vx) * (wx - vx) + (py - vy) * (wy - vy)) / l2
                    tmin = np.minimum(1, t)
                    t = np.maximum(0, tmin)
                    x = float(format((vx + t * (wx - vx)), '.1f'))
                    y = float(format((vy + t * (wy - vy)), '.1f'))
                distxp = dist2(px, x, py, y)
                if(distxp < min_dist and isCollisionFree(robot, (x, y), obstacles)):
                    min_dist = distxp
                    x_min = x
                    y_min = y
                    i_min = i
                    j_min = j

    flag = CollissionFreeLine((px, py), (x_min, y_min), robot, obstacles)

    if min_dist < 1000 and flag:
        if(points[i_min] == (x_min, y_min)):
            adjListMap[i_min].append(pi)
            adjListMap[pi].append(i_min)
            return points, adjListMap
        elif(points[j_min] == (x_min, y_min)):
            adjListMap[j_min].append(pi)
            adjListMap[pi].append(j_min)
            return points, adjListMap
        else:
            m = len(points) + 1
            points[m] = (x_min, y_min)
            adjListMap[m] = [pi]
            adjListMap[pi].append(m)
            adjListMap[i_min].append(m)
            adjListMap[j_min].append(m)
            adjListMap[m].append(i_min)
            adjListMap[m].append(j_min)
            adjListMap[i_min].remove(j_min)
            adjListMap[j_min].remove(i_min)
            return points, adjListMap
    else:
        return points, adjListMap
    # return points,adjListMap


# Grow a simple RRT
def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()

    newPoints = points

    for i in points:
        adjListMap[i] = []

    adjListMap[1] = [2]
    adjListMap[2] = [1]
    for i in points.keys():
        if not (adjListMap[i]):
            px = points[i][0]
            py = points[i][1]
            newPoints, adjListMap = findPoints(px, py, i, newPoints, adjListMap)

    return newPoints, adjListMap


# Perform basic search -> (BFS)
def basicSearch(tree, start, goal):
    path = []
    visited = []
    queue = [[start]]
    if start == goal:
        return
    while queue:
        temp = queue.pop(0)
        node = temp[-1]
        if node not in visited:
            neighbors = tree[node]
            for nb in neighbors:
                path = list(temp)
                path.append(nb)
                queue.append(path)
                if nb == goal:
                    return path
            visited.append(node)
    return "Error: No possible path"


# Display the RRT and Path
# start goal and polygons were set to None
def displayRRTandPath(points, tree, path, robotStart='None', robotGoal='None', polygons='None'):
    fig, ax = setupPlot()

    if(robotStart != 'None' and robotGoal != 'None' and polygons != 'None'):
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    for i in tree:
        xi = []
        yi = []
        for j in range(len(tree[i])):
            xi.append(points[tree[i][j]][0])
            yi.append(points[tree[i][j]][1])
        for k in range(len(xi)):
            xc = [points[i][0], xi[k]]
            yc = [points[i][1], yi[k]]
            plt.plot(xc, yc, color='black')
            # plt.plot(xc,yc,'-o',color='black')
        # patch=patches.Patch()
    xp = []
    yp = []
    for j in path:
        xp.append(points[j][0])
        yp.append(points[j][1])
    plt.plot(xp, yp, '-o', color='orange')
    plt.show()

    return


# Collision checking
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.
    # define robot path
    verts_b = [(0, 0), (0, 10), (10, 10), (10, 0), (0, 0)]
    codes_b = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
    boundary_path = Path(verts_b, codes_b)
    verts_r = []
    codes_r = []
    # codes_r.append(Path.MOVETO)
    for i in range(len(robot)):
        x = robot[i][0] + point[0]
        y = robot[i][1] + point[1]
        r_adjusted = (x, y)
        verts_r.append(r_adjusted)
        if i == 0:
            codes_r.append(Path.MOVETO)
        else:
            codes_r.append(Path.LINETO)
    verts_r.append(verts_r[0])
    codes_r.append(Path.CLOSEPOLY)
    robot_path = Path(verts_r, codes_r)

    if not boundary_path.contains_path(robot_path):
        return False

    obstacle_paths = []
    for j in range(len(obstacles)):
        verts_o = []
        codes_o = []
        for i in range(len(obstacles[j])):
            verts_o.append(obstacles[j][i])
            if i == 0:
                codes_o.append(Path.MOVETO)
            else:
                codes_o.append(Path.LINETO)
        verts_o.append(verts_o[0])
        codes_o.append(Path.CLOSEPOLY)
        obstacle_paths.append(Path(verts_o, codes_o))

    for i in range(len(obstacle_paths)):
        if robot_path.intersects_path(obstacle_paths[i], filled=True):
            return False
        else:
            continue
    return True


# The full RRT algorithm
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    path = []

    j = 100
    while j < 5500:
        points[1] = startPoint
        xy = 10 * np.random.random((1, 2))
        temp = (xy[0, 0], xy[0, 1])
        while not (isCollisionFree(robot, temp, obstacles)) or not (CollissionFreeLine(points[1], temp, robot, obstacles)):
            xy = 10 * np.random.random((1, 2))
            temp = (xy[0, 0], xy[0, 1])
        points[2] = temp
        k = 2
        while k < j:
            xy = 10 * np.random.random((1, 2))
            temp = (xy[0, 0], xy[0, 1])
            if isCollisionFree(robot, temp, obstacles):
                k = k + 1
                points[k] = temp
            else:
                continue

        points[k] = goalPoint
        newPoints = dict()
        adjListMap = dict()

        # Your code goes here
        newPoints = points

        for i in points:
            adjListMap[i] = []

        adjListMap[1] = [2]
        adjListMap[2] = [1]

        for i in points.keys():
            if not (adjListMap[i]):
                px = points[i][0]
                py = points[i][1]
                newPoints, adjListMap = findPoints2(px, py, i, newPoints, adjListMap, robot, obstacles)

        # print len(points)
        path = basicSearch(adjListMap, 1, k)
        if path:
            print j
            # displayRRTandPath(newPoints, adjListMap, path, robotStart, robotGoal, obstacles)
            return points, adjListMap, path
            # displayRRTandPath(newPoints, adjListMap, path, robotStart, robotGoal, obstacles)
        else:
            j = j + 100
            continue

    # displayRRTandPath(newPoints, adjListMap, path, robotStart, robotGoal, obstacles)
    return points, adjListMap, path


if __name__ == "__main__":

    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0:
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x, y)):
        return (x + x1, y + y1)

    def goal((x, y)):
        return (x + x2, y + y2)

    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)
    for f in range(21, 501):
        points[f] = (float(format(random() * 10., '.1f'))), float(format(random() * 10., '.1f'))

    # Printing the points
    print ""
    print "The input points are:"
    print str(points)
    print ""

    points, adjListMap = growSimpleRRT(points)
    # print "POINTS: ", points, "\n\nADJLISTMAP: ", adjListMap

    # Search for a solution
    path = basicSearch(adjListMap, 1, 20)
    print "\nPATH : ", path

    # Your visualization code
    displayRRTandPath(points, adjListMap, path)

    # Solve a real RRT problem
    points, tree, path = RRT(robot, obstacles, (x1, y1), (x2, y2))
    print "\nPATH : ", path

    # Your visualization code
    displayRRTandPath(points, tree, path, robotStart, robotGoal, obstacles)
