#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
import math
import heapq


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__("path_planner_node")
        self.get_logger().info("PathPlannerNode started")

        # Latest occupancy grid
        self.map = None

        # QoS for map subscription
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, qos
        )

        # Service to create path plans
        self.srv = self.create_service(CreatePlan, "create_plan", self.create_plan_cb)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
        self.get_logger().info(
            f"Map received: frame={msg.header.frame_id}, width={msg.info.width}, height={msg.info.height}"
        )

    def create_plan_cb(self, request, response):
        if self.map is None:
            self.get_logger().warn("No map received yet!")
            return response

        start_pose = request.start
        goal_pose = request.goal

        # Generate A* path with interpolation
        response.path = self.create_astar_plan(start_pose, goal_pose)
        self.get_logger().info(f"Returning path with {len(response.path.poses)} waypoints")
        return response

    def create_astar_plan(self, start, goal):
        """
        Generate a smooth path using A* with obstacle avoidance and safety buffer.
        """
        grid = self.map
        resolution = grid.info.resolution
        width = grid.info.width
        height = grid.info.height
        data = grid.data
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        # --- Helpers ---
        def pose_to_idx(pose):
            x = int((pose.pose.position.x - origin_x) / resolution)
            y = int((pose.pose.position.y - origin_y) / resolution)
            return (x, y)

        def idx_to_pose(idx):
            x = idx[0] * resolution + origin_x + resolution / 2
            y = idx[1] * resolution + origin_y + resolution / 2
            pose = PoseStamped()
            pose.header.frame_id = grid.header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            return pose

        # Check if a cell and its neighbors are free (safety buffer)
        def is_free(x, y):
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        occ = data[ny * width + nx]
                        if occ != 0:  # treat unknown (-1) and occupied (100) as obstacles
                            return False
                    else:
                        return False  # outside map is obstacle
            return True

        def neighbors(idx):
            x, y = idx
            nbrs = [
                (x+1, y), (x-1, y), (x, y+1), (x, y-1),
                (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)
            ]
            return [n for n in nbrs if is_free(*n)]

        def heuristic(a, b):
            return math.hypot(a[0]-b[0], a[1]-b[1])

        start_idx = pose_to_idx(start)
        goal_idx = pose_to_idx(goal)

        # --- A* search ---
        open_set = []
        heapq.heappush(open_set, (0, start_idx))
        came_from = {}
        g_score = {start_idx: 0}
        closed = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            if current == goal_idx:
                break
            closed.add(current)
            for n in neighbors(current):
                tentative_g = g_score[current] + heuristic(current, n)
                # small penalty for diagonal moves (optional)
                if abs(n[0]-current[0]) + abs(n[1]-current[1]) == 2:
                    tentative_g += 0.1
                if n not in g_score or tentative_g < g_score[n]:
                    g_score[n] = tentative_g
                    f_score = tentative_g + heuristic(n, goal_idx)
                    heapq.heappush(open_set, (f_score, n))
                    came_from[n] = current

        # --- Reconstruct path ---
        path_idx = []
        curr = goal_idx
        while curr != start_idx:
            path_idx.append(curr)
            if curr in came_from:
                curr = came_from[curr]
            else:
                self.get_logger().warn("No path found!")
                break
        path_idx.append(start_idx)
        path_idx.reverse()

        # --- Interpolate for smooth motion ---
        def idx_to_world(ix, iy):
            x = ix * resolution + origin_x + resolution / 2
            y = iy * resolution + origin_y + resolution / 2
            return (x, y)

        interpolated_path = []
        for i in range(len(path_idx)-1):
            x0, y0 = idx_to_world(*path_idx[i])
            x1, y1 = idx_to_world(*path_idx[i+1])
            dist = math.hypot(x1-x0, y1-y0)
            steps = max(1, int(dist / 0.2))  # 0.2 m spacing
            for s in range(steps):
                t = s / steps
                interpolated_path.append((x0 + t*(x1-x0), y0 + t*(y1-y0)))
        interpolated_path.append(idx_to_world(*path_idx[-1]))

        # --- Convert to PoseStamped ---
        path_msg = Path()
        path_msg.header.frame_id = grid.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in interpolated_path:
            pose = PoseStamped()
            pose.header.frame_id = grid.header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
