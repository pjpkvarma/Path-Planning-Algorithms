import cv2
import numpy as np
import heapq

def a_star(image, start, goal):
    img = np.where(image == 255, 0, 1)  # White is 0, Black is 1
    rows, cols = img.shape
    open_set = [(0, start)]
    g_score = {start: 0}
    f_score = {start: np.linalg.norm(np.array(start) - np.array(goal))}
    came_from = {}
    visited = set()  # Track searched regions

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], visited

        visited.add(current)  # Mark current as visited

        neighbors = [
            (current[0] + dx, current[1] + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        ]

        for neighbor in neighbors:
            if (
                0 <= neighbor[0] < cols
                and 0 <= neighbor[1] < rows
                and img[neighbor[1], neighbor[0]] == 0  # Only travel in white space
            ):
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + np.linalg.norm(
                        np.array(neighbor) - np.array(goal)
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, visited

if __name__ == "__main__":
    img = cv2.imread("maps/map2.png", cv2.IMREAD_GRAYSCALE)
    start = (50, 50)
    goal = (600, 400)

    path, visited = a_star(img, start, goal)

    colored_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    for v in visited:
        colored_img[v[1], v[0]] = [0, 255, 0]  # Green for visited regions

    if path:
        for p in path:
            cv2.circle(colored_img, p, 2, (255, 0, 0), -1)  # Blue with larger radius

    cv2.circle(colored_img, start, 5, (0, 0, 255), -1)  # Red start point
    cv2.circle(colored_img, goal, 5, (0, 0, 255), -1)   # Red goal point

    cv2.imshow("A* Pathfinding", colored_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
