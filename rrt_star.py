import cv2
import numpy as np

def rrt_star(image, start, goal, max_iter=5000):
    img = np.where(image == 255, 0, 1)  # White is 0, Black is 1
    nodes = [start]
    parents = [-1]
    rows, cols = img.shape

    for _ in range(max_iter):
        rand_point = (np.random.randint(cols), np.random.randint(rows))
        closest_idx = np.argmin([np.linalg.norm(np.array(rand_point) - np.array(n)) for n in nodes])
        closest = nodes[closest_idx]
        direction = np.array(rand_point) - np.array(closest)
        dist = np.linalg.norm(direction)

        if dist > 15:
            new_point = tuple((np.array(closest) + direction / dist * 15).astype(int))
        else:
            new_point = rand_point

        if 0 <= new_point[0] < cols and 0 <= new_point[1] < rows and img[new_point[1], new_point[0]] == 0:
            nodes.append(new_point)
            parents.append(closest_idx)

            # Draw the graph (edges in green)
            yield closest, new_point  # Return edge points for drawing

            if np.linalg.norm(np.array(new_point) - np.array(goal)) < 15:
                path = [goal]
                curr_idx = len(nodes) - 1
                while curr_idx != -1:
                    path.append(nodes[curr_idx])
                    curr_idx = parents[curr_idx]
                yield path, None  # Return final path
                break

    yield None, None  # Return when no path is found

if __name__ == "__main__":
    img = cv2.imread("maps/map2.png", cv2.IMREAD_GRAYSCALE)
    start = (50, 50)
    goal = (600, 400)

    colored_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    graph_edges = []

    for edge_start, edge_end in rrt_star(img, start, goal):
        if edge_start and edge_end:
            cv2.line(colored_img, edge_start, edge_end, (0, 255, 0), 1)  # Draw graph edge in green
            graph_edges.append((edge_start, edge_end))
        elif edge_start is None and edge_end is None:
            break
        else:  # Final path received
            path = edge_start
            for p in path:
                cv2.circle(colored_img, p, 2, (255, 0, 0), -1)  # Blue path
            break

    cv2.circle(colored_img, start, 5, (0, 0, 255), -1)  # Red start point
    cv2.circle(colored_img, goal, 5, (0, 0, 255), -1)   # Red goal point

    cv2.imshow("RRT* Pathfinding with Graph", colored_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
