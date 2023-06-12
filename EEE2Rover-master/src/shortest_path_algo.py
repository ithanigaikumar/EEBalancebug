#code uses a priority queue to keep track of nodes with the shortest dist from start node
#continuosly updates the distances and shortest path until it finds the shortest path to the end node


#NEED TO THINK ABOUT HOW WE IDENTIFY END NODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


import heapq

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

# Usage:
graph = {
    'A': [('B', 1), ('C', 3)],
    'B': [('A', 1), ('C', 2), ('D', 1)],
    'C': [('A', 3), ('B', 2), ('D', 2)],
    'D': [('B', 1), ('C', 2)],
}
start = 'A'
end = 'D'

path, distance = shortest_path(graph, start, end)
print(f"The shortest path from {start} to {end} is: {path} with a distance of: {distance}")
