# import numpy as np
# import heapq
# import matplotlib.pyplot as plt
# from typing import List, Tuple, Dict

# # Tamaño del mapa y cluster
# MAP_SIZE = (20, 20)
# CLUSTER_SIZE = 5

# # Crear un mapa de prueba con obstáculos
# def generate_map(size: Tuple[int, int]) -> np.ndarray:
#     grid = np.zeros(size, dtype=int)
#     grid[5:15, 10] = 1  # Obstáculo vertical
#     return grid

# # Dividir el mapa en clusters
# def get_clusters(grid: np.ndarray, cluster_size: int) -> List[List[Tuple[int, int]]]:
#     clusters = []
#     rows, cols = grid.shape
#     for i in range(0, rows, cluster_size):
#         for j in range(0, cols, cluster_size):
#             cluster = [(x, y) for x in range(i, min(i+cluster_size, rows))
#                                 for y in range(j, min(j+cluster_size, cols))]
#             clusters.append(cluster)
#     return clusters

# # Identificar portales entre clusters
# def get_portals(clusters: List[List[Tuple[int, int]]], grid: np.ndarray) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
#     portals = {}
#     for cluster in clusters:
#         for x, y in cluster:
#             if grid[x, y] == 0:  # No es obstáculo
#                 neighbors = [(x+dx, y+dy) for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]
#                              if (x+dx, y+dy) in cluster and grid[x+dx, y+dy] == 0]
#                 if len(neighbors) < 4:  # Borde del cluster
#                     portals[(x, y)] = neighbors
#     return portals

# def connect_clusters_via_portals(portals: Dict[Tuple[int, int], List[Tuple[int, int]]], cluster_size: int) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
#     connections = {}
#     for portal in portals:
#         x, y = portal
#         cluster_x, cluster_y = x // cluster_size, y // cluster_size
#         for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
#             neighbor_x, neighbor_y = x + dx, y + dy
#             neighbor_cluster_x, neighbor_cluster_y = neighbor_x // cluster_size, neighbor_y // cluster_size
#             if (neighbor_cluster_x, neighbor_cluster_y) != (cluster_x, cluster_y):
#                 if (neighbor_x, neighbor_y) in portals:
#                     if portal not in connections:
#                         connections[portal] = []
#                     connections[portal].append((neighbor_x, neighbor_y))
#     return connections

# # Algoritmo HPA*
# def hpa_star(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], portals: Dict[Tuple[int, int], List[Tuple[int, int]]]) -> List[Tuple[int, int]]:
#     open_list = [(0, start)]
#     came_from = {}
#     g_score = {start: 0}

#     while open_list:
#         _, current = heapq.heappop(open_list)

#         if current == goal:
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.append(start)
#             return path[::-1]

#         for neighbor in portals.get(current, []):
#             tentative_g = g_score[current] + 1
#             if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                 g_score[neighbor] = tentative_g
#                 heapq.heappush(open_list, (tentative_g, neighbor))
#                 came_from[neighbor] = current

#     return []  # No se encontró camino

# def refine_path(grid: np.ndarray, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
#     refined_path = []
#     for i in range(len(path) - 1):
#         start = path[i]
#         goal = path[i + 1]
#         segment, _ = find_path(grid, start, goal)
#         if segment:
#             refined_path.extend(segment[:-1])  # Exclude the last node to avoid duplication
#     refined_path.append(path[-1])  # Add the final goal node
#     return refined_path

# def find_path(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[List[Tuple[int, int]], Dict[Tuple[int, int], float]]:
#     open_list = [(0, start)]
#     came_from = {}
#     g_score = {start: 0}
#     f_score = {start: calculate_heuristic(start, goal)}

#     while open_list:
#         _, current = heapq.heappop(open_list)

#         if current == goal:
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.append(start)
#             return path[::-1], f_score

#         for neighbor in get_neighbors(grid, current):
#             tentative_g = g_score[current] + 1
#             if neighbor not in g_score or tentative_g < g_score[neighbor]:
#                 g_score[neighbor] = tentative_g
#                 f_score[neighbor] = tentative_g + calculate_heuristic(neighbor, goal)
#                 heapq.heappush(open_list, (f_score[neighbor], neighbor))
#                 came_from[neighbor] = current

#     return [], f_score  # No path found

# def calculate_heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
#     return abs(a[0] - b[0]) + abs(a[1] - b[1])

# def get_neighbors(grid: np.ndarray, node: Tuple[int, int]) -> List[Tuple[int, int]]:
#     neighbors = []
#     x, y = node
#     for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
#         nx, ny = x + dx, y + dy
#         if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx, ny] == 0:
#             neighbors.append((nx, ny))
#     return neighbors

# # Generar el mapa y clusters
# grid = generate_map(MAP_SIZE)
# clusters = get_clusters(grid, CLUSTER_SIZE)
# portals = get_portals(clusters, grid)
# connections = connect_clusters_via_portals(portals, CLUSTER_SIZE)

# print("Connections:", connections)

# # Definir inicio y destino
# start, goal = (0, 0), (19, 19)

# # Ejecutar HPA*
# abstract_path = hpa_star(grid, start, goal, connections)
# if abstract_path:
#     path = refine_path(grid, abstract_path)
# else:
#     path = []

# print("Path:", path)

# # Mostrar resultado (2 imagenes en una misma ventana, una con los clusters y otra con los portales)
# fig, (ax1, ax2) = plt.subplots(1, 2)
# ax1.imshow(grid, cmap='gray_r')
# ax1.set_title('Clusters')
# for cluster in clusters:
#     x, y = zip(*cluster)
#     # Vary color of clusters for better visualization
#     ax1.scatter(y, x, c=np.random.rand(3,))

# ax2.imshow(grid, cmap='gray_r')
# for portal, neighbors in portals.items():
#     x, y = zip(*neighbors)
#     ax2.scatter(y, x, c='blue', marker='x')
#     ax2.scatter(portal[1], portal[0], c='red', marker='o')
# ax2.set_title('Vecinos & Portales')

# plt.show()



















import numpy as np
import heapq
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict

# Tamaño del mapa y cluster
MAP_SIZE = (20, 20)
CLUSTER_SIZE = 5


# Crear un mapa de prueba con obstáculos
def generate_map(size: Tuple[int, int]) -> np.ndarray:
    grid = np.zeros(size, dtype=int)
    grid[5:15, 10] = 1  # Obstáculo vertical
    return grid


# Dividir el mapa en clusters
def get_clusters(grid: np.ndarray, cluster_size: int) -> List[List[Tuple[int, int]]]:
    clusters = []
    rows, cols = grid.shape
    for i in range(0, rows, cluster_size):
        for j in range(0, cols, cluster_size):
            cluster = [(x, y) for x in range(i, min(i+cluster_size, rows))
                                for y in range(j, min(j+cluster_size, cols))]
            clusters.append(cluster)
    return clusters


# Identificar portales entre clusters
def get_portals(clusters: List[List[Tuple[int, int]]], grid: np.ndarray) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
    portals = {}
    for cluster in clusters:
        for x, y in cluster:
            if grid[x, y] == 0:  # No es obstáculo
                neighbors = [(x+dx, y+dy) for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]
                             if (x+dx, y+dy) in cluster and grid[x+dx, y+dy] == 0]
                if len(neighbors) < 4:  # Borde del cluster
                    portals[(x, y)] = neighbors
    return portals


def connect_clusters_via_portals(portals: Dict[Tuple[int, int], List[Tuple[int, int]]], cluster_size: int) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
    connections = {}
    for portal in portals:
        x, y = portal
        cluster_x, cluster_y = x // cluster_size, y // cluster_size
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            neighbor_x, neighbor_y = x + dx, y + dy
            neighbor_cluster_x, neighbor_cluster_y = neighbor_x // cluster_size, neighbor_y // cluster_size
            if (neighbor_cluster_x, neighbor_cluster_y) != (cluster_x, cluster_y):
                if (neighbor_x, neighbor_y) in portals:
                    if portal not in connections:
                        connections[portal] = []
                    connections[portal].append((neighbor_x, neighbor_y))
    return connections


# Algoritmo HPA*
def hpa_star(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int], portals: Dict[Tuple[int, int], List[Tuple[int, int]]]) -> List[Tuple[int, int]]:
    open_list = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    while open_list:
        _, current = heapq.heappop(open_list)
        print("Current:", current)
        print("Portals:", portals.get(current, []))
        print("Open list:", open_list)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]

        for neighbor in portals.get(current, []):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                heapq.heappush(open_list, (tentative_g, neighbor))
                came_from[neighbor] = current

    return []  # No se encontró camino


# Generar el mapa y clusters
grid = generate_map(MAP_SIZE)
clusters = get_clusters(grid, CLUSTER_SIZE)
portals = get_portals(clusters, grid)

# Definir inicio y destino
start, goal = (0, 0), (19, 19)

# Ejecutar HPA*
path = hpa_star(grid, start, goal, portals)

print(path)

# Mostrar resultado (2 imagenes en una misma ventana, una con los clusters y otra con los portales)
fig, (ax1, ax2) = plt.subplots(1, 2)
ax1.imshow(grid, cmap='gray_r')
ax1.set_title('Clusters')
for cluster in clusters:
    x, y = zip(*cluster)
    # Vary color of clusters for better visualization
    ax1.scatter(y, x, c=np.random.rand(3,))


ax2.imshow(grid, cmap='gray_r')
for portal, neighbors in portals.items():
    x, y = zip(*neighbors)
    ax2.scatter(y, x, c='blue', marker='x')
    ax2.scatter(portal[1], portal[0], c='red', marker='o')
ax2.set_title('Vecinos & Portales')

plt.show()
