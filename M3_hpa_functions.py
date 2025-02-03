import heapq
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np

class HPAStar:
    def __init__(self, grid, start_node, goal_node, cluster_size=5):
        self.grid = grid
        self.cluster_size = cluster_size
        self.graph = defaultdict(list)
        self.clusters = {}
        self.entry_points = []
        self.discarded_entry_points = []
        self.start_node = start_node
        self.goal_node = goal_node
        self.preprocess()

    def preprocess(self):
        """Abstract the grid into clusters and build the hierarchical graph."""
        self.clusters = self.build_clusters()
        self.create_abstract_graph()

    def build_clusters(self):
        """Divide the grid into clusters. Each cluster is the bounding box of cluster_size x cluster_size cells."""
        clusters = {}
        rows, cols = len(self.grid), len(self.grid[0])
        for r in range(0, rows, self.cluster_size):
            for c in range(0, cols, self.cluster_size):
                clusters[(r, c)] = [(r + i, c + j) for i in range(self.cluster_size) for j in range(self.cluster_size)]
        self.clusters = clusters
        return clusters

    def create_abstract_graph(self):
        """Generate an abstract graph by linking cluster entry points. Each inter-edge (edge between two clusters) has always a cost of 1 and needs to be added to the graph. The graph will contain all the nodes (entry points) and edges (connections between entry points) of the clusters with the value being the distance between points."""
        for cluster, nodes in self.clusters.items():
            entry_points = self.find_entry_points(nodes)

            # Check if start or goal node is in the cluster
            if self.start_node in nodes:
                entry_points.append(self.start_node)
            if self.goal_node in nodes:
                entry_points.append(self.goal_node)

            # Reorder the entry points by row and column
            entry_points = sorted(entry_points, key=lambda x: (x[0], x[1]))

            self.entry_points.extend(entry_points)

            # print("Cluster:", cluster)
            # print("Entry points:", entry_points)

            for ep1 in entry_points:
                for ep2 in entry_points:
                    if ep1 != ep2:
                        # Check if there is the transition between two entry points is possible (no obstacles in between)
                        if self.is_valid_transition(cluster, ep1, ep2):
                            cost = self.heuristic(ep1, ep2)
                            self.graph[ep1].append((cost, ep2))

                # Add inter-edges
                for neighbor in self.find_neighbor_clusters(cluster):
                    # print("Neighbor:", neighbor)
                    # print("Cluster:", cluster)
                    # print("Entry point:", ep1)
                    # Check if entry point is in edge with other cluster
                    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        if (ep1[0] + dr, ep1[1] + dc) in self.clusters[neighbor]:
                            neighbor_entry_points = self.find_entry_points(self.clusters[neighbor])
                            for neighbor_ep in neighbor_entry_points:
                                # Check if neighbor entry point is in edge with previous cluster and distance is 1
                                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                                    if (neighbor_ep[0] + dr, neighbor_ep[1] + dc) in self.clusters[cluster] and self.heuristic(ep1, neighbor_ep) == 1:
                                        print("Neighbor entry point:", neighbor_ep)
                                        cost = self.heuristic(ep1, neighbor_ep)
                                        self.graph[ep1].append((cost, neighbor_ep))

    def is_valid_transition(self, cluster, start, goal):
        print("IS VALID TRANSITION: ", start, goal)
        return bool(self.standard_search(cluster, start, goal))

    def find_entry_points(self, nodes):
        """Identify border entry points within a cluster. If a node is an obstacle, it is not considered an entry point. Also if a node is on the border of grid and has no connection to neighboring clusters, it is not considered an entry point."""
        entry_points = []
        discarded_entry_points = []
        for (r, c) in nodes:
            if any((r+dr, c+dc) not in nodes for dr, dc in [(-1,0), (1,0), (0,-1), (0,1)]) and self.grid[r][c] == 0:
                # Check if the node is on the border of the grid
                if (r == 0 or r == len(self.grid) - 1 or c == 0 or c == len(self.grid[0]) - 1):
                    # Check if the node has a connection to a neighboring cluster
                    connected = self.entry_point_has_neighbor_cluster((r, c))
                    if not connected:
                        discarded_entry_points.append((r, c))
                    # discarded_entry_points.append((r, c))
                    else:
                        entry_points.append((r, c))
                else:
                    entry_points.append((r, c)) if self.entry_point_has_neighbor_cluster((r, c)) else discarded_entry_points.append((r, c))

        self.discarded_entry_points.extend(discarded_entry_points)

        return entry_points

    def entry_point_has_neighbor_cluster(self, node):
        # Find to which cluster the node belongs
        node_cluster = self.find_node_cluster(node)

        # Check if the node has a connection to a neighboring cluster
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            # Get the cluster for the neighboring node
            neighbor_cluster = self.find_node_cluster((node[0] + dr, node[1] + dc))
            if neighbor_cluster and neighbor_cluster != node_cluster:
                # print("Neighbor cluster:", neighbor_cluster)
                # print("Node cluster:", node_cluster)
                # print("Node:", node)
                # print("Neighbor node:", (node[0] + dr, node[1] + dc))
                # Check if neighboring node is not an obstacle
                if self.grid[node[0] + dr][node[1] + dc] == 0:
                    return True
        return False

    def find_node_cluster(self, node):
        """Find the cluster to which the node belongs."""
        for cluster, nodes in self.clusters.items():
            if node in nodes:
                return cluster
        return None

    def find_neighbor_clusters(self, cluster):
        """Find neighboring clusters of a given cluster."""
        neighbors = set()
        # Having the cluster (r, c), the neighbors are (r-cluster_size, c), (r+cluster_size, c), (r, c-cluster_size), (r, c+cluster_size)
        for dr, dc in [(-self.cluster_size, 0), (self.cluster_size, 0), (0, -self.cluster_size), (0, self.cluster_size)]:
            neighbor = (cluster[0] + dr, cluster[1] + dc)
            if neighbor in self.clusters:
                neighbors.add(neighbor)
        return neighbors


    def heuristic(self, a, b):
        """Calculate the Manhattan distance between two points."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])


    def search(self, start, goal):
        """Perform hierarchical A* search."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            print("Current:", current)
            print("Neighbors:", self.graph.get(current, []))

            for cost, neighbor in self.graph.get(current, []):
                new_cost = cost_so_far[current] + cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

        return []

    def standard_search(self, cluster, start, goal):
        """
            Perform A* search using the grid, so it can be used for local path
            planning within a cluster.

            Args:
                cluster: Cluster to search
                start: Start node
                goal: Goal node

            Returns:
                Path from start to goal
        """
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, start, goal)

            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dr, current[1] + dc)
                if neighbor in self.clusters[cluster] and self.grid[neighbor[0]][neighbor[1]] == 0:
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, neighbor))
                        came_from[neighbor] = current

        return []


    def reconstruct_path(self, came_from, start, goal):
        """Reconstruct the path from start to goal."""
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from.get(current, start)
        path.append(start)
        path.reverse()
        return path


# Define a sample grid (0 = walkable, 1 = obstacle)
grid = [
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 1, 1, 1, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 1, 1, 1, 1, 0],
    [0, 1, 0, 1, 0, 0, 0, 1, 0, 0],
    [0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
    [0, 1, 1, 1, 1, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
]

# Define start and goal positions
start = (0, 0)  # Top-left corner
goal = (9, 9)   # Bottom-right corner


# Initialize HPA* with the grid and cluster size
hpa = HPAStar(grid, start, goal, cluster_size=5)

# Run hierarchical A* search
path = hpa.search(start, goal)

# Print the result
print("Path found:", path)

# Plot the grid and path
fig = plt.figure()

# Plot the grid, being y aixs the rows and x axis the columns
plt.imshow(grid, cmap='Greys', origin='upper')

# Plot the graph
for node, neighbors in hpa.graph.items():
    for cost, neighbor in neighbors:
        x_values = [node[1], neighbor[1]]
        y_values = [node[0], neighbor[0]]

        # Plot the connection
        plt.plot(x_values, y_values, color='blue', alpha=0.5)

        # Compute midpoint for labeling
        mid_x, mid_y = (node[1] + neighbor[1]) / 2, (node[0] + neighbor[0]) / 2

        # Add cost label at midpoint
        plt.text(mid_x, mid_y, f"{cost:.1f}", fontsize=8, color='black', ha='center', va='center', bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))

# Plot the entry points
plt.scatter([y for x, y in hpa.entry_points], [x for x, _ in hpa.entry_points], color='red', s=50, label='Entry Points')

# Plot the discarded entry points
plt.scatter([y for x, y in hpa.discarded_entry_points], [x for x, _ in hpa.discarded_entry_points], color='blue', s=50, label='Discarded Entry Points')

# Plot the path
if path:
    path_x, path_y = zip(*path)
    plt.plot(path_y, path_x, color='red', marker='o', linestyle='-', linewidth=2, markersize=5, label="Path")

# Add a legend (outside the plot) and add the type of elements for each color
plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

# Show the plot
plt.show()
