# Implimentaion of RRT algorithm
# example to run this program:
# python rrt.py robot_env_01.txt 1.0 2.0 8.5 7
import sys

import math
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from random import *


# Set up matplotlib to create a plot with an empty square
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1, 1, 1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0, 0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
    ))
    return fig, ax


# Make a patch for a single pology
def createPolygonPatch(polygon, color):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0] / 10., xy[1] / 10.))
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


# distance between two points
def dist(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


# Returns angle bewteen three points in degrees
def angle(p1, p2, q):
    return math.degrees(math.acos((dist(p1, p2)**2 + dist(p1, q)**2 - dist(p2, q)**2) / (2 * dist(p1, p2) * dist(p1, q))))


# Returns the point on a line segment closest to the new point
def closePoint(p0, p1, q):
    yChange = p0[1] - p1[1]
    xChange = p0[0] - p1[0]
    m1 = float(yChange) / xChange
    b1 = p0[1] - (m1 * p0[0])

    m2 = -1 * (float(xChange) / yChange)
    b2 = q[1] - (m2 * q[0])
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    point = (x, y)
    return point


# Grow a simple RRT
def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()

    adjListMap[1] = []
    newPoints[1] = (5, 5)
    temp = 0

    for p in range(1, len(points) + 1):
        temp = 0
        subList = []
        shortest1 = 100
        shortest2 = 100
        for v in range(1, len(newPoints) + 1):
            if dist(points[p], newPoints[v]) < shortest1:
                shortest1 = dist(points[p], newPoints[v])
                temp = v
                subList = adjListMap[temp]
        if len(newPoints) >= 2 and subList:
            subList = adjListMap[temp]
            # print "Sublist: ", subList
            for m in range(0, len(subList)):
                if angle(newPoints[temp], newPoints[subList[m]], points[p]) < 90:
                    point = closePoint(newPoints[temp], newPoints[subList[m]], points[p])
                    if dist(points[p], point) < shortest2:
                        before = temp
                        after = subList[m]
                        shortest2 = dist(points[p], point)
        # print "short 1", shortest1
        # print "short 2", shortest2
        if shortest1 < shortest2:
            newPoints[len(newPoints) + 1] = points[p]
            adjListMap[len(adjListMap) + 1] = [temp]
            adjListMap[temp].append(len(adjListMap))
        else:
            adjListMap[before].remove(after)
            adjListMap[after].remove(before)
            newPoints[len(newPoints) + 1] = point
            newPoints[len(newPoints) + 1] = points[p]
            adjListMap[len(adjListMap) + 1] = [before]
            adjListMap[len(adjListMap)].append(after)
            adjListMap[before].append(len(adjListMap))
            adjListMap[after].append(len(adjListMap))
            adjListMap[len(adjListMap) + 1] = [len(adjListMap)]
            adjListMap[len(adjListMap) - 1].append(len(adjListMap))
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
def displayRRTandPath(points, tree, path, robotStart=None, robotGoal=None, polygons=None):

    fig, ax = setupPlot()

    if robotStart is not None and robotGoal is not None and polygons is not None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    if not path:
        return

    for n in range(1, len(tree) + 1):
        subList = tree[n]
        for m in range(0, len(subList)):
            end = subList[m]
            sPoint = points[n]
            ePoint = points[end]
            line = [(sPoint[0] * .1, sPoint[1] * .1), (ePoint[0] * .1, ePoint[1] * .1)]
            (line_xs, line_ys) = zip(*line)
            ax.add_line(Line2D(line_xs, line_ys, linewidth=1, color='black'))

    x = []
    y = []
    for p in range(0, len(path)):
        currentPoint = points[path[p]]
        x.append(currentPoint[0] * .1)
        y.append(currentPoint[1] * .1)
    ax.add_line(Line2D(x, y, linewidth=1, color='orange'))
    plt.show()

    return


# Collision checking
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.

    return False


# The full RRT algorithm
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    tree = dict()
    path = []
    # Your code goes here.

    return points, tree, path


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
    # drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (3, 2)
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
    '''
    for f in range(21, 200):
        points[f] = (random() * 10, random() * 10)
    '''
    # Printing the points
    print ""
    print "The input points are:"
    print str(points)
    print ""

    points, adjListMap = growSimpleRRT(points)
    print "POINTS: ", points, "\n\nADJLISTMAP: ", adjListMap
    # Search for a solution
    path = basicSearch(adjListMap, 1, 20)
    print "\nPATH : ", path

    # print closePoint((1., 5.), (2., 3.), (2., 4.))
    # Your visualization code
    displayRRTandPath(points, adjListMap, path)

    # Solve a real RRT problem
    RRT(robot, obstacles, (x1, y1), (x2, y2))

    # Your visualization code
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles)
