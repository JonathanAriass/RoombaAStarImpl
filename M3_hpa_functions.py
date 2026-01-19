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

    def is_walkable(self, row, col):
        """Check if a cell is walkable. 0=empty, 2=robot, 3=goal are all walkable."""
        if 0 <= row < len(self.grid) and 0 <= col < len(self.grid[0]):
            val = self.grid[row][col]
            return val == 0 or val == 2 or val == 3
        return False

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
        """Generate an abstract graph by linking cluster entry points.

        Uses segment endpoint optimization: instead of adding every walkable
        border cell, only add the first and last cell of each contiguous
        walkable segment. This dramatically reduces entry points.
        """
        all_entry_points = set()
        inter_edges = []  # Store inter-cluster edges to create

        # Find inter-cluster transitions using segment endpoints
        for cluster, nodes in self.clusters.items():
            nodes_set = set(nodes)
            min_r = min(n[0] for n in nodes)
            max_r = max(n[0] for n in nodes)
            min_c = min(n[1] for n in nodes)
            max_c = max(n[1] for n in nodes)

            # Check each border for transitions to neighbor clusters
            borders = [
                ([(min_r, c) for c in range(min_c, max_c + 1)], (-1, 0)),  # Top
                ([(max_r, c) for c in range(min_c, max_c + 1)], (1, 0)),   # Bottom
                ([(r, min_c) for r in range(min_r, max_r + 1)], (0, -1)),  # Left
                ([(r, max_c) for r in range(min_r, max_r + 1)], (0, 1)),   # Right
            ]

            for border_cells, (dr, dc) in borders:
                # Find contiguous walkable segments and only use endpoints
                segment = []
                for cell in border_cells:
                    neighbor = (cell[0] + dr, cell[1] + dc)
                    is_valid = (self.is_walkable(cell[0], cell[1]) and
                                self.is_walkable(neighbor[0], neighbor[1]) and
                                neighbor not in nodes_set)

                    if is_valid:
                        segment.append((cell, neighbor))
                    else:
                        # End of segment - add only endpoints
                        if segment:
                            # Add first endpoint
                            all_entry_points.add(segment[0][0])
                            all_entry_points.add(segment[0][1])
                            inter_edges.append(segment[0])
                            # Add last endpoint if segment has more than 1 cell
                            if len(segment) > 1:
                                all_entry_points.add(segment[-1][0])
                                all_entry_points.add(segment[-1][1])
                                inter_edges.append(segment[-1])
                            segment = []

                # Don't forget last segment
                if segment:
                    all_entry_points.add(segment[0][0])
                    all_entry_points.add(segment[0][1])
                    inter_edges.append(segment[0])
                    if len(segment) > 1:
                        all_entry_points.add(segment[-1][0])
                        all_entry_points.add(segment[-1][1])
                        inter_edges.append(segment[-1])

        # Add start and goal as entry points
        all_entry_points.add(self.start_node)
        all_entry_points.add(self.goal_node)

        self.entry_points = list(all_entry_points)

        # Create inter-cluster edges (cost 1 for adjacent cells)
        for cell1, cell2 in inter_edges:
            if (1, cell2) not in self.graph[cell1]:
                self.graph[cell1].append((1, cell2))
            if (1, cell1) not in self.graph[cell2]:
                self.graph[cell2].append((1, cell1))

        # Create intra-cluster edges (within same cluster)
        for cluster, nodes in self.clusters.items():
            nodes_set = set(nodes)
            cluster_entry_points = [ep for ep in all_entry_points if ep in nodes_set]

            for ep1 in cluster_entry_points:
                for ep2 in cluster_entry_points:
                    if ep1 != ep2:
                        if self.is_valid_transition(cluster, ep1, ep2):
                            cost = self.heuristic(ep1, ep2)
                            # Avoid duplicate edges
                            if (cost, ep2) not in self.graph[ep1]:
                                self.graph[ep1].append((cost, ep2))

    def is_valid_transition(self, cluster, start, goal):
        """Check if there is a valid path between two entry points within a cluster."""
        return bool(self.standard_search(cluster, start, goal))

    def find_entry_points(self, nodes):
        """Identify border entry points within a cluster using segment endpoints only.

        Instead of adding every border cell, we find contiguous walkable segments
        along each cluster border and only add the endpoints of each segment.
        This dramatically reduces the number of entry points.
        """
        entry_points = set()
        nodes_set = set(nodes)

        # Get cluster bounding box
        rows = [n[0] for n in nodes]
        cols = [n[1] for n in nodes]
        min_r, max_r = min(rows), max(rows)
        min_c, max_c = min(cols), max(cols)

        # Check each of the 4 borders for walkable segments
        borders = [
            # (cells along border, direction to neighbor)
            ([(min_r, c) for c in range(min_c, max_c + 1)], (-1, 0)),  # Top border
            ([(max_r, c) for c in range(min_c, max_c + 1)], (1, 0)),   # Bottom border
            ([(r, min_c) for r in range(min_r, max_r + 1)], (0, -1)),  # Left border
            ([(r, max_c) for r in range(min_r, max_r + 1)], (0, 1)),   # Right border
        ]

        for border_cells, (dr, dc) in borders:
            segment = []
            for (r, c) in border_cells:
                neighbor = (r + dr, c + dc)
                # Check if this cell can transition to neighbor cluster
                is_valid = (
                    self.is_walkable(r, c) and  # Current cell is walkable
                    neighbor not in nodes_set and  # Neighbor is in different cluster
                    self.is_walkable(neighbor[0], neighbor[1])  # Neighbor is walkable
                )

                if is_valid:
                    segment.append((r, c))
                else:
                    # End of segment - add endpoints
                    if segment:
                        entry_points.add(segment[0])
                        if len(segment) > 1:
                            entry_points.add(segment[-1])
                        segment = []

            # Don't forget last segment
            if segment:
                entry_points.add(segment[0])
                if len(segment) > 1:
                    entry_points.add(segment[-1])

        return list(entry_points)

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
                # Check if neighboring node is walkable
                if self.is_walkable(node[0] + dr, node[1] + dc):
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


    def search(self, start, goal, refine=True):
        """Perform hierarchical A* search.

        Args:
            start: Start node
            goal: Goal node
            refine: If True, refine the abstract path to get full walkable path

        Returns:
            Full path from start to goal (refined) or abstract path (entry points only)
        """
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        explored = 0

        while open_set:
            _, current = heapq.heappop(open_set)
            explored += 1
            if current == goal:
                abstract_path = self.reconstruct_path(came_from, start, goal)
                if refine:
                    return self.refine_path(abstract_path)
                return abstract_path

            for cost, neighbor in self.graph.get(current, []):
                new_cost = cost_so_far[current] + cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

        # HPA* search failed - will fall back to regular A*
        pass
        return []

    def refine_path(self, abstract_path):
        """Refine the abstract path by finding actual walkable paths between entry points.

        Args:
            abstract_path: List of entry points from hierarchical search

        Returns:
            Full walkable path with all intermediate nodes
        """
        if len(abstract_path) <= 1:
            return abstract_path

        full_path = []

        for i in range(len(abstract_path) - 1):
            start_node = abstract_path[i]
            end_node = abstract_path[i + 1]

            # Find which cluster(s) these nodes belong to
            start_cluster = self.find_node_cluster(start_node)
            end_cluster = self.find_node_cluster(end_node)

            if start_cluster == end_cluster:
                # Both nodes in same cluster - use standard search within cluster
                local_path = self.standard_search(start_cluster, start_node, end_node)
            else:
                # Nodes in different clusters (inter-cluster edge)
                # For adjacent entry points across clusters, the path is just the two nodes
                local_path = [start_node, end_node]

            # Append local path, avoiding duplicates at junctions
            if full_path and local_path and full_path[-1] == local_path[0]:
                full_path.extend(local_path[1:])
            else:
                full_path.extend(local_path)

        return full_path

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
                if neighbor in self.clusters[cluster] and self.is_walkable(neighbor[0], neighbor[1]):
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
# grid = [
#     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
#     [0, 1, 0, 1, 0, 1, 1, 1, 0, 0],
#     [1, 1, 0, 0, 0, 1, 0, 1, 1, 0],
#     [0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 1, 1, 1, 1, 0],
#     [0, 1, 0, 1, 0, 0, 0, 1, 0, 1],
#     [0, 1, 0, 0, 0, 1, 0, 1, 0, 1],
#     [0, 1, 1, 1, 1, 1, 0, 1, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
#     [1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
# ]

# # Define start and goal positions
# start = (0, 0)  # Top-left corner
# goal = (9, 9)   # Bottom-right corner


# # Initialize HPA* with the grid and cluster size
# hpa = HPAStar(grid, start, goal, cluster_size=5)

# # Run hierarchical A* search
# path = hpa.search(start, goal)

# # Print the result
# print("Path found:", path)

# # Plot the grid and path
# fig = plt.figure()

# # Plot the grid, being y aixs the rows and x axis the columns
# plt.imshow(grid, cmap='Greys', origin='upper')

# # Plot the graph
# for node, neighbors in hpa.graph.items():
#     for cost, neighbor in neighbors:
#         x_values = [node[1], neighbor[1]]
#         y_values = [node[0], neighbor[0]]

#         # Plot the connection
#         plt.plot(x_values, y_values, color='blue', alpha=0.5)

#         # Compute midpoint for labeling
#         mid_x, mid_y = (node[1] + neighbor[1]) / 2, (node[0] + neighbor[0]) / 2

#         # Add cost label at midpoint
#         plt.text(mid_x, mid_y, f"{cost:.1f}", fontsize=8, color='black', ha='center', va='center', bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))

# # Plot the entry points
# plt.scatter([y for x, y in hpa.entry_points], [x for x, _ in hpa.entry_points], color='red', s=50, label='Entry Points')

# # Plot the discarded entry points
# plt.scatter([y for x, y in hpa.discarded_entry_points], [x for x, _ in hpa.discarded_entry_points], color='blue', s=50, label='Discarded Entry Points')

# # Plot the path
# if path:
#     path_x, path_y = zip(*path)
#     plt.plot(path_y, path_x, color='red', marker='o', linestyle='-', linewidth=2, markersize=5, label="Path")

# # Add a legend (outside the plot) and add the type of elements for each color
# plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))

# # Show the plot
# plt.show()
