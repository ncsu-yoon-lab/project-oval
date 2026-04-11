import heapq
from .graph import haversine


# Heuristic for A* straight line haversine distance to goal.
def heuristic(a, b, coords):
    return haversine(coords[a], coords[b])


# A* pathfinding on the road graph.
# Returns an ordered list of vertex ids from start to goal, or None if no path exists.
def astar(start, goal, coords, adj):
    if start not in adj or goal not in adj:
        return None

    open_set  = [(heuristic(start, goal, coords), 0.0, start)]
    g_score   = {start: 0.0}
    came_from = {}
    visited   = set()

    while open_set:
        _, g, u = heapq.heappop(open_set)

        if u in visited:
            continue
        visited.add(u)

        if u == goal:
            path = [u]
            while u in came_from:
                u = came_from[u]
                path.append(u)
            path.reverse()
            return path

        for v, w in adj[u].items():
            if v in visited:
                continue
            tentative = g + w
            if tentative < g_score.get(v, float("inf")):
                g_score[v]   = tentative
                came_from[v] = u
                f = tentative + heuristic(v, goal, coords)
                heapq.heappush(open_set, (f, tentative, v))

    return None


# Total haversine distance in meters along a path.
def path_distance(path_ids, coords):
    total = 0.0
    for i in range(len(path_ids) - 1):
        total += haversine(coords[path_ids[i]], coords[path_ids[i + 1]])
    return total