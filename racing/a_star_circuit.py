import os
import sys
import math

import numpy as np

# import matplotlib.pyplot as plt


print_map = True
show_animation = False


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


class AStarPlanner:
    def __init__(self, obstacle_map, resolution, min_x, min_y, max_x, max_y, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """
        self.obstacle_map = np.where(obstacle_map == 100, True, False)
        self.resolution = resolution
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.rr = rr
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return (
                str(self.x)
                + ","
                + str(self.y)
                + ","
                + str(self.cost)
                + ","
                + str(self.parent_index)
            )

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(
            self.calc_xy_index(sx, self.min_x),
            self.calc_xy_index(sy, self.min_y),
            0.0,
            -1,
        )
        goal_node = self.Node(
            self.calc_xy_index(gx, self.min_x),
            self.calc_xy_index(gy, self.min_y),
            0.0,
            -1,
        )

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost
                + self.calc_heuristic(goal_node, open_set[o]),
            )
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print(f"{__file__} Found goal!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(
                    current.x + self.motion[i][0],
                    current.y + self.motion[i][1],
                    current.cost + self.motion[i][2],
                    c_id,
                )
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)
        ]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        squares = math.ceil(self.rr / (self.resolution * 2))
        for i in range(-squares, squares + 1):
            for j in range(-squares, squares + 1):
                if self.obstacle_map[node.x + i][node.y + j]:
                    return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [
            [False for _ in range(self.y_width)] for _ in range(self.x_width)
        ]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)],
        ]

        return motion


def downsample_grid(map, factor=1):
    """
    Downsample grid by some factor.

    Key:
        - any occupied -> occupied
        - all free -> free
        - else (same as any unknown) -> unknown
    """
    src_x_dim, src_y_dim = len(map), len(map[0])
    dst_x_dim, dst_y_dim = src_x_dim // factor, src_y_dim // factor
    ret = np.ones((dst_x_dim, dst_y_dim))
    for dst_i, src_i in enumerate(range(0, src_x_dim, factor)):
        for dst_j, src_j in enumerate(range(0, src_y_dim, factor)):
            patch = map[src_i : src_i + factor, src_j : src_j + factor]
            if np.any(patch == 100):
                ret[dst_i, dst_j] = 100
            elif np.all(patch == 1):
                ret[dst_i, dst_j] = 1
            else:
                ret[dst_i, dst_j] = -1
    return ret


def print_terminal_map(ox, oy, sx, sy, gx, gy, rx, ry, resolution=24):
    min_x = min(rx + [sx, gx]) - 1
    max_x = max(rx + [sx, gx]) + 1
    min_y = min(ry + [sy, gy]) - 1
    max_y = max(ry + [sy, gy]) + 1

    size = max([max_x - min_x, max_y - min_y])

    def coord_to_pixel(x, y):
        return (
            round((x - min_x) * resolution / size) - 1,
            round((y - min_y) * resolution / size) - 1,
        )

    map = np.zeros((resolution, resolution))

    # Obstacles
    for x, y in zip(ox, oy):
        px, py = coord_to_pixel(x, y)
        if px >= 0 and px < resolution and py >= 0 and py < resolution:
            map[px, py] = 1

    # Waypoints
    for x, y in zip(rx, ry):
        px, py = coord_to_pixel(x, y)
        map[px, py] = 2

    # Start and goal
    px, py = coord_to_pixel(sx, sy)
    map[px, py] = 3
    px, py = coord_to_pixel(gx, gy)
    map[px, py] = 4

    for y in range(resolution - 1, 0, -1):
        row = ""
        for x in range(resolution):
            if map[x, y] == 1:
                row += "*"
            elif map[x, y] == 2:
                row += "."
            elif map[x, y] == 3:
                row += "S"
            elif map[x, y] == 4:
                row += "G"
            else:
                row += " "
        print(row)


def main():
    print(f"{__file__} Start!")

    args = sys.argv[1:]
    if len(args) != 1:
        ValueError("Please provide only the path to the grid and pose files.")
    path = args[0]

    with open(os.path.join(path, "occupancy_grid.npy"), "rb") as f:
        grid = np.load(f)
    with open(os.path.join(path, "map_metadata.npy"), "rb") as f:
        # resolution, width, height
        map_metadata = np.load(f)
    with open(os.path.join(path, "map_origin.npy"), "rb") as f:
        # position.x, position.y, position.z, quaternion.x, quaternion.y, quaternion.z, quaternion.w
        map_origin = np.load(f)
    with open(os.path.join(path, "pose.npy"), "rb") as f:
        # position.x, position.y, position.z, quaternion.x, quaternion.y, quaternion.z, quaternion.w
        pose = np.load(f)

    # Unpack
    map_origin_x, map_origin_y, *_ = map_origin
    map_resolution, map_width_voxels, map_height_voxels = map_metadata
    map_resolution = round(map_resolution, 5)
    map_width_meters = map_width_voxels * map_resolution
    map_height_meters = map_height_voxels * map_resolution

    # Flip grid
    grid = grid[::-1, ::]

    # start position
    sx = -pose[1]
    sy = pose[0]
    _, _, yaw = euler_from_quaternion(*pose[3:])

    # set goal position
    robot_radius = 0.3
    scaling_factor = robot_radius * 1.8
    gx = sx - (math.sin(yaw)) * scaling_factor
    gy = sy - (math.cos(yaw)) * scaling_factor

    # Get obstacles before drawing the midpoint line
    ob = np.argwhere(grid == 100)
    downsample_factor = 1
    ox, oy = (
        list(ob[::downsample_factor, 0] * map_resolution + map_origin_x),
        list(ob[::downsample_factor, 1] * map_resolution + map_origin_y),
    )

    midpoint_x = (sx + gx) / 2
    midpoint_y = (sy + gy) / 2

    run = sx - gx  # horizontal distance from start to goal
    rise = sy - gy  # vertical distance from start to goal

    def point_to_index_x(point):
        return int((point - map_origin_x) / map_resolution)

    def point_to_index_y(point):
        return int((point - map_origin_y) / map_resolution)

    midpoint_x_idx = point_to_index_x(midpoint_x)
    midpoint_y_idx = point_to_index_y(midpoint_y)

    def draw_vertical():
        i = 1  # start at 1 so that second part of line can start at 0
        while True:
            if (
                midpoint_y_idx + i >= len(grid[0])
                or grid[midpoint_x_idx][midpoint_y_idx + i] == 100
            ):
                break
            grid[midpoint_x_idx][midpoint_y_idx + i] = 100
            i += 1
        i = 0
        while True:
            if (
                midpoint_y_idx - i == 0
                or grid[midpoint_x_idx][midpoint_y_idx - i] == 100
            ):
                break
            grid[midpoint_x_idx][midpoint_y_idx - i] = 100
            i += 1

    def draw_horizontal():
        i = 1  # start at 1 so that second part of line can start at 0
        while True:
            if (
                midpoint_x_idx + i >= len(grid)
                or grid[midpoint_x_idx + i][midpoint_y_idx] == 100
            ):
                break
            grid[midpoint_x_idx + i][midpoint_y_idx] = 100
            i += 1
        i = 0
        while True:
            if (
                midpoint_x_idx - i == 0
                or grid[midpoint_x_idx - i][midpoint_y_idx] == 100
            ):
                break
            grid[midpoint_x_idx - i][midpoint_y_idx] = 100
            i += 1

    # TODO: revisit this, might need to flip inequality
    if run > rise:
        draw_vertical()
    else:
        draw_horizontal()

    # # Get obstacles before drawing the midpoint line
    # ob = np.argwhere(grid == 100)
    # downsample_factor = 1
    # ox, oy = (
    #     list(ob[::downsample_factor, 0] * map_resolution + map_origin_x),
    #     list(ob[::downsample_factor, 1] * map_resolution + map_origin_y),
    # )

    # plt.plot(ox, oy, ".k")
    # plt.plot(sx, sy, "og")
    # plt.plot(gx, gy, "xb")
    # plt.plot(midpoint_x, midpoint_y, "xr")
    # plt.grid(True)
    # plt.axis("equal")
    # # plt.plot(rx, ry, "-r")
    # plt.pause(0.001)
    # plt.show()

    # Downsample map
    downsample_factor = 2**1  # must be a factor of 2
    grid = downsample_grid(grid, downsample_factor)
    map_resolution *= downsample_factor
    map_width_voxels, map_height_voxels = len(grid), len(grid[0])

    print(f"{__file__} Planning path...")
    a_star = AStarPlanner(
        grid,
        map_resolution,
        map_origin_x,
        map_origin_y,
        map_origin_x + map_width_meters,
        map_origin_y + map_height_meters,
        robot_radius,
    )
    rx, ry = a_star.planning(sx, sy, gx, gy)

    # save waypoints to a file
    np.savetxt(os.path.join(path, "rx.npy"), np.array(rx))
    np.savetxt(os.path.join(path, "ry.npy"), np.array(ry))

    if print_map:  # pragma: no cover
        print_terminal_map(ox, oy, sx, sy, gx, gy, rx, ry)

    # if show_animation:
    #     plt.plot(ox, oy, ".k")
    #     plt.plot(sx, sy, "og")
    #     plt.plot(gx, gy, "xb")
    #     plt.plot(midpoint_x, midpoint_y, "xr")
    #     plt.grid(True)
    #     plt.axis("equal")
    #     plt.plot(rx, ry, "-r")
    #     plt.pause(0.001)
    #     plt.show()


if __name__ == "__main__":
    main()
