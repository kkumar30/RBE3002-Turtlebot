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
import math
import time


#Add additional imports for each of the message types used
#https://docs.python.org/2/library/heapq.html - Go to this url to see Priority Queue documentation


# Map Values
#  100  Wall
#  0  Empty
#  -1  Unknown

def findhCost(current_pose, goal_pose, graph):

	return math.sqrt((current_pose[0] - goal_pose[0])**2 + (current_pose[1] - goal_pose[1])**2)

def findfCost(current_pose, start_pose, graph):

	return abs((current_pose[0] - start_pose[0]) + abs(current_pose[1] - start_pose[1]))

def findCost(current_pose, start_pose, goal_pose, graph):

	if graph[current_pose[0]][current_pose[1]] == 0:

		return (findhCost(current_pose, goal_pose, graph) + findfCost(current_pose, start_pose, graph))
	else :
		return graph[current_pose[0]][current_pose[1]]

def Astar(start_pose, goal_pose, graph):


	frontNodes = [] #Frontier nodes
	checkedNodes = [] #Checked nodes
	checkedNodes.append(start_pose)
	
	graph	

	nodestocheck = [] # Priority queue of Frontier

	path = [] # List of nodes to move 
	
	goalfound = False

	current_pose = start_pose # Start searching at the start point

	graph[current_pose[0]][current_pose[1]] = findhCost(current_pose, goal_pose, graph) + findfCost(current_pose, start_pose, graph)

	checked = False

	while not goalfound: # This loop finds all needed node costs until it finds the goal
	
		print current_pose
		temp_node = [current_pose[0] + 1, current_pose[1]]
		checked = False
		for node in checkedNodes:
			if node[0] == temp_node[0] and  node[1] == temp_node[1] or temp_node[0] < 0 or temp_node[1] < 0 or temp_node[0] > 4 or temp_node[1] > 4:
				checked = True
		if checked == False and graph[temp_node[0]][temp_node[1]] != 100:
			graph[temp_node[0]][temp_node[1]] = findhCost(temp_node, goal_pose, graph) + findfCost(temp_node, start_pose, graph)
			findCost(temp_node, start_pose, goal_pose, graph)
			frontNodes.append(temp_node)
			temp_node.append(current_pose) #node
			checkedNodes.append(temp_node)
				
		
		temp_node = [current_pose[0] - 1, current_pose[1]]
		checked = False
		for node in checkedNodes:	
			if node[0] == temp_node[0] and  node[1] == temp_node[1] or temp_node[0] < 0 or temp_node[1] < 0 or temp_node[0] > 4 or temp_node[1] > 4:
				checked = True
		if checked == False and graph[temp_node[0]][temp_node[1]] != 100:
			graph[temp_node[0]][temp_node[1]] = findhCost(temp_node, goal_pose, graph) + findfCost(temp_node, start_pose, graph)		
			findCost(temp_node, start_pose, goal_pose, graph)
			frontNodes.append(temp_node)
			temp_node.append(current_pose)
			checkedNodes.append(temp_node)
			
		temp_node = [current_pose[0], current_pose[1] + 1]
		
		checked = False		
		for node in checkedNodes:
			if node[0] == temp_node[0] and  node[1] == temp_node[1] or temp_node[0] < 0 or temp_node[1] < 0 or temp_node[0] > 4 or temp_node[1] > 4:
				checked = True
		
		if checked == False and graph[temp_node[0]][temp_node[1]] != 100:
			graph[temp_node[0]][temp_node[1]] = findhCost(temp_node, goal_pose, graph) + findfCost(temp_node, start_pose, graph)
			findCost(temp_node, start_pose, goal_pose, graph)
			frontNodes.append(temp_node)
			temp_node.append(current_pose)
			checkedNodes.append(temp_node)

		
		temp_node = [current_pose[0], current_pose[1] - 1]
		
		checked = False		
		for node in checkedNodes:
			if node[0] == temp_node[0] and  node[1] == temp_node[1] or temp_node[0] < 0 or temp_node[1] < 0 or temp_node[0] > 4 or temp_node[1] > 4:
				checked = True
		if checked == False and graph[temp_node[0]][temp_node[1]] != 100:
			graph[temp_node[0]][temp_node[1]] = findhCost(temp_node, goal_pose, graph) + findfCost(temp_node, start_pose, graph)
			findCost(temp_node, start_pose, goal_pose, graph)
			frontNodes.append(temp_node)
			temp_node.append(current_pose)
			checkedNodes.append(temp_node)

		
		bestnode = frontNodes[0]
		for node in frontNodes:
			if graph[node[0]][node[1]] <= graph[bestnode[0]][bestnode[1]]:
				bestnode = node
		frontNodes.remove(bestnode)
		current_pose = [bestnode[0], bestnode[1]]
		
		
		

		if bestnode[0] == goal_pose[0] and  bestnode[1] == goal_pose[1]:
			goalfound = True
		
	
	current_pose = bestnode	
	path.append(current_pose)
	startfound = False
	while not startfound: # This loop finds the path throught the checkes nodes
		
		path.append(current_pose)
		if current_pose == start_pose:
			startfound = True
			break
		for node in checkedNodes:
			if node[0] == current_pose[2][0] and  node[1] == current_pose[2][1]:
				current_pose = node
				break
	
	nodepath = []
	for i in range (1,len(path)):
		nodepath.append([path[len(path)-i][0],path[len(path)-i][1]])

	
		
		

	return nodepath
