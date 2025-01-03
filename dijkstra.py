import cv2
import numpy as np
import heapq

def dijkstra(image, start, goal):
    img = np.where(image == 255, 0, 1)  # White is 0, Black is 1
    rows, cols = img.shape
    dist = {start: 0}
    prev = {}
    unvisited = [(0, start)]
    visited = set()

    while unvisited:
        _, current = heapq.heappop(unvisited)

        if current == goal:
            path = []
            while current in prev:
                path.append(current)
                current = prev[current]
            path.append(start)
            return path[::-1], visited

        visited.add(current)

        neighbors = [
            (current[0] + dx, current[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]

        for neighbor in neighbors:
            if (
                0 <= neighbor[0] < cols
                and 0 <= neighbor[1] < rows
                and img[neighbor[1], neighbor[0]] == 0
            ):
                new_dist = dist[current] + 1
                if new_dist < dist.get(neighbor, float("inf")):
                    dist[neighbor] = new_dist
                    prev[neighbor] = current
                    heapq.heappush(unvisited, (new_dist, neighbor))

    return None, visited

if __name__ == "__main__":
    img = cv2.imread("maps/map2.png", cv2.IMREAD_GRAYSCALE)
    start = (50, 50)
    goal = (600, 400)

    path, visited = dijkstra(img, start, goal)

    colored_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    for v in visited:
        colored_img[v[1], v[0]] = [0, 255, 0]  # Green for visited regions

    if path:
        for p in path:
            cv2.circle(colored_img, p, 2, (255, 0, 0), -1)  # Blue with larger radius

    cv2.circle(colored_img, start, 5, (0, 0, 255), -1)  # Red start point
    cv2.circle(colored_img, goal, 5, (0, 0, 255), -1)   # Red goal point

    cv2.imshow("Dijkstra Pathfinding", colored_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
