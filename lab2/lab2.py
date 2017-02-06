# Fall 2012 6.034 Lab 2: Search
#
# Your answers for the true and false questions will be in the following form.  
# Your answers will look like one of the two below:
#ANSWER1 = True
#ANSWER1 = False

# 1: True or false - Hill Climbing search is guaranteed to find a solution
#    if there is a solution
ANSWER1 = False

# 2: True or false - Best-first search will give an optimal search result
#    (shortest path length).
#    (If you don't know what we mean by best-first search, refer to
#     http://courses.csail.mit.edu/6.034f/ai3/ch4.pdf (page 13 of the pdf).)
ANSWER2 = False

# 3: True or false - Best-first search and hill climbing make use of
#    heuristic values of nodes.
ANSWER3 = True

# 4: True or false - A* uses an extended-nodes set.
ANSWER4 = True

# 5: True or false - Breadth first search is guaranteed to return a path
#    with the shortest number of nodes.
ANSWER5 = True

# 6: True or false - The regular branch and bound uses heuristic values
#    to speed up the search for an optimal path.
ANSWER6 = False

# Import the Graph data structure from 'search.py'
# Refer to search.py for documentation
from search import Graph

## Optional Warm-up: BFS and DFS
# If you implement these, the offline tester will test them.
# If you don't, it won't.
# The online tester will not test them.

def bfs(graph, start, goal):
    return bfsUtil(graph, start, goal)

## Once you have completed the breadth-first search,
## this part should be very simple to complete.
def dfs(graph, start, goal):
    return dfsUtil(graph, start, goal)


## Now we're going to add some heuristics into the search.  
## Remember that hill-climbing is a modified version of depth-first search.
## Search direction should be towards lower heuristic values to the goal.
def hill_climbing(graph, start, goal):
    return hill_climbing_util(graph, start, goal)

## Now we're going to implement beam search, a variation on BFS
## that caps the amount of memory used to store paths.  Remember,
## we maintain only k candidate paths of length n in our agenda at any time.
## The k top candidates are to be determined using the 
## graph get_heuristic function, with lower values being better values.
def beam_search(graph, start, goal, beam_width):
    #return beam_search_util2(graph, start, goal, beam_width)
    raise NotImplementedError

## Now we're going to try optimal search.  The previous searches haven't
## used edge distances in the calculation.

## This function takes in a graph and a list of node names, and returns
## the sum of edge lengths along the path -- the total distance in the path.
def path_length(graph, node_names):
    raise NotImplementedError

def branch_and_bound(graph, start, goal):
    raise NotImplementedError

def a_star(graph, start, goal):
    raise NotImplementedError

## It's useful to determine if a graph has a consistent and admissible
## heuristic.  You've seen graphs with heuristics that are
## admissible, but not consistent.  Have you seen any graphs that are
## consistent, but not admissible?

def is_admissible(graph, goal):
    raise NotImplementedError

def is_consistent(graph, goal):
    raise NotImplementedError

HOW_MANY_HOURS_THIS_PSET_TOOK = ''
WHAT_I_FOUND_INTERESTING = ''
WHAT_I_FOUND_BORING = ''

def bfsUtil(graph, start, goal):

    queue = list([])
    queue.append([start])

    extended_list = []
    while len(queue) > 0:
        path = queue.pop(0) # always from first is removed, -1 will remove the last element from the list

        if (path[0] == goal):
            return path[::-1]

        if (path[0] not in extended_list):
            extended_list.append(path[0])
            connectedNodes = graph.get_connected_nodes(path[0])

            for link in connectedNodes:
                temp_path = [link]
                temp_path += path
                queue.append(temp_path)

def dfsUtil(graph, start, goal):

    queue = list([])

    queue.append([start])
    extended_list = []

    while queue is not None:
        path = queue.pop(0) # always from first is removed, -1 will remove the last element from the list

        if (path[0] == goal):
            #print " found path : ", path
            return path[::-1]

        if (path[0] not in extended_list):
            extended_list.append(path[0])
            connectedNodes = graph.get_connected_nodes(path[0])

            for link in connectedNodes:
                temp_path = [link]
                temp_path += path
                queue.insert(0, temp_path)

def hill_climbing_util(graph, start, goal):
    queue = list([])

    queue.append([(start, 0)])
    extended_list = []

    while len(queue) > 0:
        path = queue.pop(0) # always from first is removed, -1 will remove the last element from the list

        if (path[0][0] == goal):
            result = []
            for item in path:
                result.append(item[0])
            return result[::-1]

        if (path[0][0] not in extended_list): #Explore the topic
            extended_list.append(path[0][0])
            connectedNodes = graph.get_connected_nodes(path[0][0])

            temp_queue=[]
            for targetNode in connectedNodes:
                if (targetNode not in extended_list):
                    link = graph.get_heuristic(targetNode, goal)

                    temp_path = [(targetNode, link)]
                    temp_path += path
                    if (temp_queue is None or len(temp_queue) == 0):
                        temp_queue.insert(0, temp_path)
                    else:
                        i=0
                        while ( i < len(temp_queue) and temp_queue[i][0][1] < temp_path[0][1]):
                            i = i + 1

                        #Should be inserted at i
                        if (i < len(temp_queue) and temp_queue[i][0][1] == temp_path[0][1]):
                            if (temp_queue[i][0][0] < temp_path[0][0]):
                                temp_queue.insert(i+1, temp_path)
                            else:
                                temp_queue.insert(i, temp_path)
                        else:
                            temp_queue.insert(i, temp_path)
            queue[:0] = temp_queue

def beam_search_util(graph, start, goal, beam_width):
    MARKER = 999
    #print graph
    #print " Beam width = ", beam_width
    queue = list([])
    queue.append([(start, 0, MARKER)])

    extended_list = []

    while (len(queue) > 0):

        neighbour_heuristic = list()
        while(True): #Last element of last tuple of last element in queue itself
            path = queue.pop(0)
            #print " Popped : ", path[0][0], " connected nodes = ", graph.get_connected_nodes(path[0][0])

            if (path[0][0] == goal):
                #print " found path : ", path
                result = []
                for item in path:
                    #print item
                    result.append(item[0])
                    #print " Result = ", result
                print " Returning result : ", result[::-1]
                return result[::-1]

            if (path[0][0] not in extended_list):
                extended_list.append(path[0][0])
                #print  " Extended list = ", extended_list
                connectedNodes = graph.get_connected_nodes(path[0][0])
                #print "========Heuristic======="
                for node in connectedNodes:
                    if (node not in extended_list):
                        temp_path = [(node, graph.get_heuristic(node, goal), 0)]
                        temp_path += path
                        neighbour_heuristic.append(temp_path)
                #        print " Node = ", node, " heuristic = ",graph.get_heuristic(node, goal)
                #print " Heuristic received : ", neighbour_heuristic

            if(path[-1][-1] == MARKER):
                break

        #Now sort all neighbours according to heuristic. At this point all the neighbours of level above will be available
        #print " Heuristic received : ", neighbour_heuristic
        neighbour_heuristic = sorted(neighbour_heuristic, cmp=sortedHeuristic)
        #print " Sorted heuristic ", neighbour_heuristic

        if (beam_width < len(neighbour_heuristic)):
            k = min(beam_width, len(neighbour_heuristic))
            neighbour_heuristic = neighbour_heuristic[0:k] #Get top elements only

        queue = neighbour_heuristic
        #print " Effective size of queue = ", len(queue)
        #print " Queue Before : ", queue
        for i in range(0, len(queue)):
            temp_marker_list = list(queue[i][-1])
            temp_marker_list[-1] = 0
            queue[i][-1] = tuple(temp_marker_list)

        if (len(queue) > 0):
            temp_marker_list = list(queue[i][-1])
            temp_marker_list[-1] = MARKER
            queue[i][-1] = tuple(temp_marker_list)
        #print " Queue After : ", queue
        #print "---------------------------------------------------------------"
    return []

def sortedHeuristic(elementA, elementB):

    # Negative for less than
    # Zero for equal
    # Positive for greater than
    #0 : Node Name
    #1 : Heuristic
    #2 : Marker Value

    # Either of elementA or elementB is of the form [('C', 3, 0), ('A', 5, 0), ('S', 0, 0)]
    #print "comparing ", elementA, " and ", elementB
    if (elementA[0][1] > elementB[0][1]): #Higher heuristic value should come first
        return -1
    else:
        if (elementA[0][1] < elementB[0][1]):
            return 1
        else:
            if (elementA[0][0] < elementB[0][0]): #Lower choronological order should come first
                return -1
            elif (elementA[0][0] > elementB[0][0]):
                return 1
            else:
                return 0

def beam_search_util2(graph, start, goal, beam_width):
    #print "beam_width %d", beam_width
    pathList = [(start,)]
    reachedGoal = False
    if start == goal:
      print "Returning result ", [start]
      return [start]
    while len(pathList) > 0:
        pathList = pathList[:beam_width]
        #print "path = ", pathList
        #print [ graph.get_heuristic(path[-1],goal) for path in pathList ]
        newPaths = []
        while len(pathList) > 0:
            pathToExtend = pathList[0]
            pathList.remove(pathList[0])
            nodeToExtend = pathToExtend[-1]
            newNodes = graph.get_connected_nodes(nodeToExtend)
            if len(pathToExtend) > 1:
                newNodes = [ node for node in newNodes if node not in pathToExtend]
            if goal in newNodes:
                goalPath = pathToExtend + (goal,)
                print "Returning result ", list(goalPath)
                return list(goalPath)
            newPaths += [ pathToExtend + (node,) for node in newNodes ]
        pathList.extend(newPaths)
        quickSort(graph,goal,pathList)
        #print "sortedPathsList: " + str(pathList)
        #print [graph.get_heuristic(path[-1],goal) for path in pathList]

        #print "newPathsList" + str(newPaths)
        # quickSort(graph,goal,newPaths,1)
        # #print "newPathsList sorted by len" + str(newPaths)
    return []

def quickSort(graph,goal,alist,sortType=0):
   quickSortHelper(graph,goal,alist,0,len(alist)-1,sortType)

def quickSortHelper(graph,goal,alist,first,last,sortType):
   if first<last:

       splitpoint = partition(graph,goal,alist,first,last)

       quickSortHelper(graph,goal,alist,first,splitpoint-1,sortType)
       quickSortHelper(graph,goal,alist,splitpoint+1,last,sortType)

def partition(graph,goal,alist,first,last):
   pivotvalue = graph.get_heuristic(alist[first][-1],goal) #- len(alist[first])

   leftmark = first+1
   rightmark = last

   done = False
   while not done:

       while leftmark <= rightmark and \
               graph.get_heuristic(alist[leftmark][-1],goal)  <= pivotvalue:
           leftmark = leftmark + 1

       while graph.get_heuristic(alist[rightmark][-1],goal) >= pivotvalue and \
               rightmark >= leftmark:
           rightmark = rightmark -1

       if rightmark < leftmark:
           done = True
       else:
           temp = alist[leftmark]
           alist[leftmark] = alist[rightmark]
           alist[rightmark] = temp

   temp = alist[first]
   alist[first] = alist[rightmark]
   alist[rightmark] = temp


   return rightmark













