import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point
from tf.transformations import euler_from_quaternion
from Queue import PriorityQueue
import math
import time


#Add additional imports for each of the message types used
#https://docs.python.org/2/library/heapq.html - Go to this url to see Priority Queue documentation


# Map Values
#  100  Wall
#  0  Empty
#  -1  Unknown

#defining a node for traversing through the points
class Node:
    def __init__(self, xVal, yVal):
        self.point = Point()
        self.point.x = xVal
        self.point.y = yVal
        self.visited = False
        #print(self.visited) #Uncomment for debugging
        self.cost = 9999999.999 #Making it v v large

    def isExpanded(self):
        self.visited = True

    def __eq__(self, other):
        return (self.point.x == other.point.x and self.point.y == other.point.y)
    
    #check "unhashable instance" error later
    def __hash__(self):
        return hash(self.point)

class Astar:
    def __init__(self, start, goal, occup, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints):
        print 'Initializing'
        self.start = Node(start.x, start.y)
        self.start.cost = 0
        self.goal = Node(goal.x, goal.y)
        self.frontier = PriorityQueue()
        self.path = []
        self.occup = occup
        self.pub_end = pub_end
        self.pub_path = pub_path
        self.pub_visited = pub_visited
        self.pub_frontier = pub_frontier
        self.pub_waypoints = pub_waypoints

        #Gotta adjust to 37 38
        #Sets up all the nodes in map
        self.nodes = [[0 for x in range(37)] for y in range(37)] #Changed to 37 because starts from 0
        for i in range(37):
            for j in range(37):
                self.nodes[i][j] = Node(i, j)

        publishVisitedCells([self.start, self.goal], self.pub_end)
        

    def runAStar(self):
        print "Performing A star........."
        self.frontier.put((0, (self.start, [self.start]))) #Added/initialized Path list here
        frontierList = []
        visited = set() #placeholder for already visited nodes
        frontierList.append(self.start) #need to put the start in    

        while (not self.frontier.empty()):
            
            (p, (current, path)) = self.frontier.get()
            frontierList.remove(current)
            print "Current removed from frontier"
            visited.add(current)
            print "Added to visit done"
            publishVisitedCells(visited, self.pub_visited)
            print "Visit Complete"
            current.isExpanded() #check neighborus
            print 'Current point = ', current.point.x, current.point.y#, current.cost, point

            if (current == self.goal): #Fix end condition
                self.path = path
                break
            for n in self.getNeighbour(current):
                # if n in self.visited:
                #     continue
                f_val = self.heuristic(n) + current.cost + self.costTo(current, n)
                if ((current.cost + self.costTo(current, n)) < n.cost) and not self.isOccupied(n) :
                    lvals = [k for k in path]
                    lvals.append(n)
                    #determine the first cost
                    n.cost = current.cost + self.costTo(current, n)
                    self.frontier.put((f_val, (n, lvals)))
                    frontierList.append(n)
            
            publishVisitedCells(frontierList, self.pub_frontier)
            #rospy.sleep(0.1)
        print visited
        #visitCells(visited, self.pub_visited)
        emptyList = []
        publishVisitedCells(self.path, self.pub_path)    
        waypoints = findWaypoints(self.path)    
        publishVisitedCells(waypoints, self.pub_waypoints)

        cells = GridCells()
        cells.header.frame_id = 'map'
        cells.cell_width = 0.3
        cells.cell_height = 0.3

        
        pub_visited.publish(cells)
        pub_frontier.publish(cells)
        publishVisitedCells(waypoints, self.pub_waypoints)
        print "Visit pub done........."

    def heuristic(self, pnt):
        return (self.goal.point.x - pnt.point.x)*(self.goal.point.x - pnt.point.x) + (self.goal.point.y - pnt.point.y)*(self.goal.point.y - pnt.point.y)

    def costTo(self, pos1, pos2):
        if (self.isOccupied(pos1) or self.isOccupied(pos2)):
           return 999999999.999
        else:
            return 1

    def getNeighbour(self, parent):
        nlist = []
        #Assigns to 4 neighbouring all 4 points
        neighbour1 = self.makePoint(parent.point.x + 1, parent.point.y)
        neighbour2 = self.makePoint(parent.point.x - 1, parent.point.y)
        neighbour3 = self.makePoint(parent.point.x, parent.point.y + 1)
        neighbour4 = self.makePoint(parent.point.x, parent.point.y - 1)
        
        if (neighbour1 != -1 and not neighbour1.visited):
            nlist.append(neighbour1)
        if (neighbour2 != -1 and not neighbour2.visited):
            nlist.append(neighbour2)
        if (neighbour3 != -1 and not neighbour3.visited):
            nlist.append(neighbour3)
        if (neighbour4 != -1 and not neighbour4.visited):
            nlist.append(neighbour4)

        return nlist

    def makePoint(self, x_value, y_value):
        #p = Point()
        #p.x = x;
        #p.y = y;
        if (x_value < 37 and x_value >= 0 and y_value >= 0 and y_value < 37):
            return self.nodes[x_value][y_value]
        return -1

    def isOccupied(self, p):
        if self.occup.data[int(p.point.x + p.point.y*self.occup.info.height)] == 100:
            return True
        else:
            return False


def publishVisitedCells(pointlist, pub):
    
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = 0.3
    cells.cell_height = 0.3

    for pnt in pointlist:
        point = Point()
        point.x = pnt.point.x*0.3 + 0.95 - 0.3 #Change values to shift offsets
        point.y = pnt.point.y*0.3 - 0.15 + 0.3 #0.3 is the resolution
        cells.cells.append(point)
        #rospy.sleep(0.001) #Use this to visualize slowly

        #print pub
        
    pub.publish(cells)
    #rospy.sleep(0.001)

def findWaypoints(path):
    ctr=1
    list_waypoints = []

    # for i in range(0,height): 
    #     ctr = ctr + 1
    #     for j in range(1,width):
    #         ctr = ctr + 1
    while ctr + 1 < len(path):
        prev = path[ctr-1]
        current = path[ctr]
        next = path[ctr+1]
        print "Waypoints = "
        print " prev->(", prev.point.x, prev.point.y, ")  Current-> (", current.point.x, current.point.y,")Next-(" ,next.point.x, next.point.y, ")"
        
        if prev.point.x != next.point.x and prev.point.y != next.point.y:
            
            ctr = ctr + 1
            list_waypoints.append(current)

        ctr = ctr + 1
    print list_waypoints #Notprinting the list correctly

    print "++++++++++++++++++++++++++++++++++"


    # for lists in list_waypoints:
    #     print lists.point.x


    return list_waypoints



def updateMapCallback(occupancy_grid):
    global occup
    global isMapUpdated

    print 'Got the map OccupancyGrids!!'
    occup = occupancy_grid
    isMapUpdated = True


if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('LabPathPlanning')

    global pub
    global grid
    global occup
    global map_sub
    global pub_visited
    global pub_frontier
    global isMapUpdated
    global pub_end
    global pub_path

    startPoint = Point()
    startPoint.x = 5 
    startPoint.y = 3
    goalPoint = Point()
    goalPoint.x = 2 
    goalPoint.y = 12 

    isMapUpdated = False
   
    map_sub = rospy.Subscriber('/map', OccupancyGrid, updateMapCallback, queue_size=1)

    pub_end = rospy.Publisher('/Goalpts', GridCells, queue_size=10)
    pub_path = rospy.Publisher('/Pathpts', GridCells, queue_size=10)
    pub_visited = rospy.Publisher('/Visitedpts', GridCells, queue_size=10)
    pub_frontier = rospy.Publisher('/Frontierpts', GridCells, queue_size=10)
    pub_waypoints = rospy.Publisher('/Waypoints', GridCells, queue_size=10) #Nathaniel's the best for fixing this!!
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(2, 0))
    
    
    #print occup.info.width, occup.data[int(p1.x + p1.y*occup.info.width)]
    myAlg = Astar(startPoint, goalPoint, occup, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)
    #(self, start, goal, occup, pub_end, pub_path, pub_visited, pub_frontier, pub_waypoints)
    myAlg.runAStar()
    print "Finally Dooooooooonnnnne!!!"

