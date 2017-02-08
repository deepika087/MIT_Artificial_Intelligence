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
    return beam_search_util(graph, start, goal, beam_width)
    #raise NotImplementedError

## Now we're going to try optimal search.  The previous searches haven't
## used edge distances in the calculation.

## This function takes in a graph and a list of node names, and returns
## the sum of edge lengths along the path -- the total distance in the path.
def path_length(graph, node_names):
    return path_length_util(graph, node_names)

    #raise NotImplementedError

def branch_and_bound(graph, start, goal):
    return branch_and_bound_util(graph, start, goal)
    #raise NotImplementedError

def a_star(graph, start, goal):
    return a_star_util(graph, start, goal)
    #raise NotImplementedError

## It's useful to determine if a graph has a consistent and admissible
## heuristic.  You've seen graphs with heuristics that are
## admissible, but not consistent.  Have you seen any graphs that are
## consistent, but not admissible?

def is_admissible(graph, goal):
    """
    edges=[Edge e1 from S to A with length 6, Edge e2 from A to B with length 4, Edge e3 from B to F with length 7, Edge e4 from C to D with length 6, Edge e5 from C to A with length 3, Edge e6 from E to D with length 7, Edge e7 from D to H with length 6, Edge e8 from S to C with length 2, Edge e9 from B to D with length 2, Edge e10 from E to G with length 25, Edge e11 from E to C with length 5]
    """
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

"""
Chooses the best(minimum value) heuristic and delete the other options according to NPTEL but MIT assignment considers back tracking
"""
def hill_climbing_util(graph, start, goal):
    queue = list([])

    queue.append([(start, 0)])
    extended_list = []

    while len(queue) > 0:
        #print "queue : ", queue

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

"""
Chooses best available options and delete the rest. Uses only heuristic and minimum value of heursittic avaiable is picked
"""
def beam_search_util(graph, start, goal, beam_width):
    #print " Graph : ", graph
    #print " start Node : ", start, " goal :", goal

    queue = list([])

    queue.append([(start, 0)])
    extended_list = []

    temp_queue=[]
    while len(queue) > 0:
        #print "queue : ", queue

        path = queue.pop(0) # always from first is removed, -1 will remove the last element from the list

        #print " Popped : ", path[0][0]
        if (reachedGoalState(queue, goal)):
            result = []
            _p = list(filter(lambda x: x[0][0] == goal, queue))
            #print " Filtered result : ", _p

            for item in _p[0]:
                result.append(item[0])
            print " Found result : ", result[::-1]
            return result[::-1]

        if (path[0][0] not in extended_list): #Explore the topic
            extended_list.append(path[0][0])
            connectedNodes = graph.get_connected_nodes(path[0][0])

            for c_node in connectedNodes:
                if (c_node not in extended_list):
                    #print " Neighbour node of ", path[0][0], " is ", c_node, " heuristic = ", graph.get_heuristic(c_node, goal)
                    temp_path = [(c_node, graph.get_heuristic(c_node, goal))]
                    temp_path += path
                    temp_queue.append(temp_path)

        if (len(temp_queue) > 1):
            temp_queue = sorted(temp_queue, cmp=beamSearchComparator)
        #print " Sorted queue : ", temp_queue

        if (len(queue) == 0 and len(temp_queue) == 0):
            break

        if (len(queue) == 0):
            #print " =============Resetting the queue==============="
            queue = temp_queue[0:beam_width]
            temp_queue = []
        #print " Trimmed queue is ", queue
    return []

def reachedGoalState(queue, goal):
    for _ in queue:
        if (_[0][0] == goal):
            return True
    return False

def beamSearchComparator(x, y):
    x=x[0]
    y=y[0]
    if (x[1] < y[1]):
        return 1
    elif (x[1] > y[1]):
        return -1
    else:
        if (x[0] < y[0]):
            return -1
        elif(x[1] > y[1]):
            return 1
        else:
            return 0

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

def path_length_util(graph, node_names):

    result = 0
    startNode = node_names[0]

    for target in node_names[1:]:
        edge_line = graph.get_edge(startNode, target)
        result = result + edge_line.length
        startNode = target
    return result

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

"""
Branch and Bound is also called UCS. It selects cummulative minimum from all global fringe options available.
"""
def branch_and_bound_util(graph, start, goal):
    queue = list([])

    extended_list = []

    queue.append([(start, 0)])

    while(len(queue) > 0):
        path = queue.pop(0) #Pop front always

        startNode = path[0][0]
        distance_so_far = path[0][1]

        if (startNode == goal):
            result = []
            for item in path:
                result.append(item[0])
            return result[::-1]


        if (startNode not in extended_list):
            extended_list.append(startNode)

            connectedNodes = graph.get_connected_nodes(startNode)

            for c_node in connectedNodes:
                c_edge = graph.get_edge(startNode, c_node)
                if (c_node not in extended_list):
                    temp_path=[(c_node, c_edge.length + distance_so_far)]
                    temp_path += path
                    queue.append(temp_path)

            queue = sorted(queue, cmp=sortedCummulativePath)

"""
Global options from the fringe will be considered. add sorting will be done via cummulative distance & overall heuristic
"""
def a_star_util(graph, start, goal):
    queue = list([])

    extended_list = []

    queue.append([(start, 0)])

    while(len(queue) > 0):
        path = queue.pop(0) #Pop front always

        startNode = path[0][0]
        distance_so_far = path[0][1]

        if (startNode == goal):
            result = []
            for item in path:
                result.append(item[0])
            return result[::-1]

        if (startNode not in extended_list):
            extended_list.append(startNode)

            connectedNodes = graph.get_connected_nodes(startNode)
            for c_node in connectedNodes:
                c_edge = graph.get_edge(startNode, c_node)
                if (c_node not in extended_list):
                    temp_path=[(c_node, c_edge.length + distance_so_far + graph.get_heuristic(c_node, goal))]
                    temp_path += path
                    queue.append(temp_path)

            queue = sorted(queue, cmp=sortedCummulativePath)

def sortedCummulativePath(x, y):
    x=x[0]
    y=y[0]

    if (x[1] < y[1]):
        return -1
    elif (x[1] > y[1]):
        return 1
    else: #Values are equal
        if (x[0] < y[0]):
            return -1
        elif (x[0] > y[0]):
            return 1
        else:
            return 0













