#DFS to explore a graph represented as an adjacency list
#node class represents a node in the graph -> position stores the node's position in the real world
#starts at current node and checks neighbours to determine if a node has been visited or if its a dead end
#backtrack is called when there are no valid neighbours. Null nodes are marked as dead ends
#node_detection simulates the rover's movements and returns a new node and weight of path using code from ESP32
#start acts as an entry point of the exploration process


#TO DO
#figure out what to do with dead ends which are marked (pruning)
#Communicate with ESP32 for backtracking code in the backtrack function
#Integrate with ESP32 to get co-ordinates in node_detection function



class Node:
    def __init__(self, left=None, right=None, forward=None, back=None, position=None):
        self.left = left
        self.right = right
        self.forward = forward
        self.back = back
        self.position = position #hold the position of the node which is updated in the node_detection function
        self.is_dead_end = False #Intialise as not a dead end


graph = {}  # The graph represented as an adjacency list
coords = [(0,0), (1,1)]
weights = []

def depthfirst(current_node):
    global graph
    neighbors = [current_node.left, current_node.forward, current_node.right]

    # Check if the node has been visited before or if it's a dead end
    if not any(neighbors) or current_node in graph: 
        current_node.is_dead_end = True #Mark the current node as a dead end
        backtrack(current_node)
       
    else:
        # Iterate through neighbours
        for neighbor in neighbors:
            if neighbor: #checks whether a neighbour exists or not
                neighbor.back = current_node

                # use ESP32 code to move the rover and get the weight of the path
                #simulate the rover's movement, detect the neighbouring node znd determine the weight of the path between the current and detected path
                detected_node, weight = node_detection()  # Assume node_detection() returns a new node and the weight of the path
                
                # Update the graph with detected node and weight of path
                #adds an entry for the current_node if it doesn't already exist in graph
                if current_node not in graph:
                    graph[current_node] = {}
                graph[current_node][detected_node] = weight

                # Recurse on the new node to continue DFS
                depthfirst(detected_node) 




def backtrack(current_node):
    #check if current node has a valid back neighbour
    if current_node.back is not None:


        # Communicate with ESP32 code to rotate the rover and move!!!!!!!!


        #update current_node var to back neighbour 
        #moves the traversal back to the previous node, simulating the rover's rotation and movement
        current_node = current_node.back




#walls represents the prescence or absence of walls in different directions relative to the current node
#position specifies the co-ordinates of the node within the maze -> tuple (x,y)
def node_detection(walls, position):
    # implement logic to return a new node and the weight of the path according to the walls and position parameters

    for node in graph:
        if node.position == position:
            #node already exists so return it and weight = 0
            return node, 0
    
    #if the node doesn't exist, create it
    new_node = Node(position = position)

    #Determine which walls exist and set the node attributes
    if walls[0]:
        new_node.left = Node() #create new node to the left
    if walls[1]:
        new_node.forward = Node() #create new node in front
    if walls[2]:
        new_node.right = Node() #creare new node to the right

   
   #--------------- NEED TO CORRECTLY IMPLEMENT WITH ESP32 -------------
    weight = read from array ???  #actual weight of path

    return new_node, weight
   


#initiates the traversal of the graph
def start():
    # Create a start node
    start_node = Node(back=True)  # Assume it always starts with a wall behind it
    depthfirst(start_node)






#expected graph will look like:








#algorithm traverses left node first then forards then right
#it is a depth first search so it fully explores one path before backtracking and exploring other parts