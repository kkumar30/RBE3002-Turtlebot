import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point
from tf.transformations import euler_from_quaternion
from Queue import PriorityQueue
#Add additional imports for each of the message types used
#https://docs.python.org/2/library/heapq.html - Go to this url to see Priority Queue documentation


# Map Values
# – 100 – Wall
# – 0 – Empty
# – -1 – Unknown

function A*(start_x, start_y,goal_x, goal_y)
	#initially only the starting node is known
	closedSet = PriorityQueue() #Set of evaluated nodes, makes it a heapSort

##MAIN PSEUDOCODE STARTS HERE 
	openSet = Queue()

	cameFrom := the empty map

	gCost := map with default value of 100

	gCost[start]:=0

	fCost[start]:=heuristic_cost_estimate(start,goal)

	while openSet is not empty
		current:= the node in the openSet having the lowest fCost[] value

		if current = goal
			return reconstruct_path(cameFrom,current)

		openSet.Remove(current)
		closedSet.Add(current)


		for each neighbor of current

			if neighbor in closedSet
			#Ignore the neighbor- already evaluated
			continue
			#dist from start to a neighbour	
			tentative_gCost: = gCost[current] + dist_between(current, neighbor)

			#if it discovers a new node
			if neighbor not in openSet
				openSet.Add(neighbor)
			else if tentative_gCost >= gCost[neighbour]
				#This is not a better graph
				continue

			cameFrom[neighbor] := current
			gCost[neighbor] := tentative_gCost
			fCost[neighbor] := gCost[neighbor]

	return failed



def talker():
    global pub
    rospy.init_node('LabPathPlanning')        
    sub = rospy.Subscriber("/map", OccupancyGrid, simpleMapCallback)
    pub = rospy.Publisher("/myGridCells", GridCells, queue_size=1)
    rospy.sleep(1)

    while (1 and not rospy.is_shutdown()):
        publishGridCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")

# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('LabPathPlanning')
    #These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
