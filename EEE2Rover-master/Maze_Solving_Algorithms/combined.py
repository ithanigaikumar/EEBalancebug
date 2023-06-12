import heapq
class Node:
    def __init__(self, label=None, left=None, right=None, forward=None, back=None, position=None):
        self.label = label
        self.left = left
        self.right = right
        self.forward = forward
        self.back = back
        self.position = position  # hold the position of the node which is updated in the node_detection function
        self.is_dead_end = False  # Initialise as not a dead end

    def __str__(self):
        return self.label


graph = {}  # The graph represented as an adjacency list

positions = [(0,0), (0,0.7), (0,2.3), (0,2.65), (0,3), (2,3), (2,2.65), (2,2), (1.3,1.5), (1.3, 0.7), (2,0.7), (2,0), (1.65,0), (1.65, 0.35), (1.3, 0), (1.3,2), (0.65, 1.5)]
weights = [0.7, 1.6, 0.35, 0.35, 2, 0.35, 2.1, 0.86, 0.8, 0.7, 0.7, 0.35, 0.35, 1.58, 0.82] 

positions_index = 0
weights_index = 0
labels = list(map(chr, range(65, 65+len(positions))))  # Generate labels as uppercase English letters

def depthfirst(current_node):
    global graph, weights_index
    neighbors = [current_node.left, current_node.forward, current_node.right]

    if not any(neighbors) or current_node in graph:
        current_node.is_dead_end = True
        backtrack(current_node)
    else:
        for neighbor in neighbors:
            if neighbor:
                neighbor.back = current_node
                detected_node, weight = node_detection()
                if current_node not in graph:
                    graph[current_node] = []
                graph[current_node].append((detected_node, weight))
                weights_index += 1  # Move to next weight in list
                depthfirst(detected_node)

def backtrack(current_node):
    if current_node.back is not None:
        current_node = current_node.back

def node_detection():
    global positions, positions_index, labels
    for node in graph:
        if node.position == positions[positions_index]:
            return node, weights[weights_index]
    new_node = Node(label=labels[positions_index], position=positions[positions_index])
    positions_index += 1  # Move to next position in list
    return new_node, weights[weights_index]

def start():
    start_node = Node(label=labels[0], back=True)
    depthfirst(start_node)


def shortest_path(graph, start, end):
    #intialise an empty priority queue 
    queue = []
    heapq.heappush(queue, (0, start))
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    #dict keeps track of node that comes before each node in the shortest path found so far
    #used at the end to reconstruct the shortest path
    previous_nodes = {node: None for node in graph}

    #continues until all nodes have been visited
    #each iteration removes the node with the smallest distance from the queue
    while queue:
        current_distance, current_node = heapq.heappop(queue)

        # Check if the current node distance is less than the recorded one
        if distances[current_node] < current_distance:
            continue

        #goes through each neighbour of current node
        for neighbour, weight in graph[current_node]:
            distance = current_distance + weight

            # If the calculated distance is less than the recorded one
            if distance < distances[neighbour]:
                distances[neighbour] = distance
                previous_nodes[neighbour] = current_node
                heapq.heappush(queue, (distance, neighbour))

    #create the shortest path by going backwards from the end node to the start node using previous_nodes dict
    #path is an array which is reversed to start from start node
    path = []
    while end:
        path.append(end)
        end = previous_nodes[end]
    path = path[::-1]  # Reverse path

    return path, distances[path[-1]]  # Return shortest path and distance to the end node

start()

# Pretty print graph
for node, neighbours in graph.items():
    print(f"{node}: {[(str(neighbour[0]), neighbour[1]) for neighbour in neighbours]}")

# Dijkstra's shortest path
start_node = list(graph.keys())[0]  # Assume the start node is the first one
end_node = list(graph.keys())[-1]  # Assume the end node is the last one
path, distance = shortest_path(graph, start_node, end_node)
print(f"The shortest path from {start_node} to {end_node} is: {[str(node) for node in path]} with a distance of: {distance}")
