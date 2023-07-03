import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # cost from start node to current node
        self.h = 0  # heuristic cost from current node to goal node
        self.f = 0  # total cost (g + h)

    def __lt__(self, other):
        return self.f < other.f

def calculate_heuristic(current, goal):
    # Calculate the Manhattan distance between current and goal position
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def get_path(current_node):
    # Reconstruct the path from goal to start by following the parent pointers
    path = []
    while current_node is not None:
        path.append(current_node.position)
        current_node = current_node.parent
    return path[::-1]  # Reverse the path to get it from start to goal

def astar(start, goal, obstacle_list):
    open_set = []
    closed_set = set()

    # Create start and goal nodes
    start_node = Node(start)
    goal_node = Node(goal)

    # Initialize the cost from start to start as 0
    start_node.g = 0

    # Calculate the heuristic cost from start to goal
    start_node.h = calculate_heuristic(start, goal)

    # Calculate the total cost from start to goal
    start_node.f = start_node.g + start_node.h

    # Add the start node to the open set
    heapq.heappush(open_set, start_node)

    while open_set:
        # Pop the node with the lowest total cost (f) from the open set
        current_node = heapq.heappop(open_set)

        # Check if the current node is the goal node
        if current_node.position == goal:
            return get_path(current_node)

        # Add the current node to the closed set
        closed_set.add(current_node.position)

        # Generate neighboring nodes
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Possible movements: up, down, right, left

        for neighbor in neighbors:
            neighbor_position = (current_node.position[0] + neighbor[0], current_node.position[1] + neighbor[1])

            # Check if the neighbor is within the grid boundaries
            if neighbor_position[0] < 0 or neighbor_position[0] >= len(obstacle_list) or \
               neighbor_position[1] < 0 or neighbor_position[1] >= len(obstacle_list[0]):
                continue

            # Check if the neighbor is an obstacle
            if obstacle_list[neighbor_position[0]][neighbor_position[1]] == 1:
                continue

            # Create the neighbor node
            neighbor_node = Node(neighbor_position, current_node)

            # Check if the neighbor is already in the closed set
            if neighbor_node.position in closed_set:
                continue

            # Calculate the cost from start to neighbor
            neighbor_node.g = current_node.g + 1

            # Calculate the heuristic cost from neighbor to goal
            neighbor_node.h = calculate_heuristic(neighbor_position, goal)

            # Calculate the total cost from start to goal passing through the neighbor
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            # Check if the neighbor is already in the open set with a lower total cost
            for open_node in open_set:
                if open_node.position == neighbor_node.position and open_node.f < neighbor_node.f:
                    break
            else:
                # Add the neighbor to the open set
                heapq.heappush(open_set, neighbor_node)

    return None  # No path found

# Example usage
start = (0, 0)
goal = (0, 3)
obstacle_list = [
    [0, 0, 1, 0, 0],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0]
]

path = astar(start, goal, obstacle_list)
if path is None:
    print("No path found")
else:
    print("Path:", path)
