"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""
import os
import sys

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


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


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1.0  # [s]
        self.to_goal_cost_gain = 0.25
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.25  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array(
            [
                [-1, -1],
                [0, 2],
                [4.0, 2.0],
                [5.0, 4.0],
                [5.0, 5.0],
                [5.0, 6.0],
                [5.0, 9.0],
                [8.0, 9.0],
                [7.0, 9.0],
                [8.0, 10.0],
                [9.0, 11.0],
                [12.0, 13.0],
                [12.0, 12.0],
                [15.0, 15.0],
                [13.0, 13.0],
            ]
        )

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed, -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [
        x[3] - config.max_accel * config.dt,
        x[3] + config.max_accel * config.dt,
        x[4] - config.max_delta_yaw_rate * config.dt,
        x[4] + config.max_delta_yaw_rate * config.dt,
    ]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, og):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(
                trajectory, goal
            )
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost_og(
                trajectory, og, config
            )

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if (
                    abs(best_u[0]) < config.robot_stuck_flag_cons
                    and abs(x[3]) < config.robot_stuck_flag_cons
                ):
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    # TODO: convert the point cloud check to a occupancy grid check
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (
            np.logical_and(
                np.logical_and(upper_check, right_check),
                np.logical_and(bottom_check, left_check),
            )
        ).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_obstacle_cost_og(trajectory, og, config):
    """
    calc obstacle cost inf: collision
    """
    # Unpack occupancy grid
    og, (og_resolution, _, _), (og_origin_x, og_origin_y, *_) = og
    og_resolution = round(og_resolution, 5)

    # Define radius to check for collision
    # (check this many cells in each direction)
    r = int((max(config.robot_length, config.robot_width) / 1.5) / og_resolution)

    # Check for collisions
    for ix, iy in zip(trajectory[:, 0], trajectory[:, 1]):
        x = int((ix - og_origin_x) / og_resolution)
        y = int((iy - og_origin_y) / og_resolution)

        collision = any(
            [
                og[x + xf, y + yf]
                for xf in range(-r - 1, r + 2)
                for yf in range(-r - 1, r + 2)
            ]
        )

        # If there is any collision, trajectory is strapped
        if collision:
            return float("Inf")

    return 0  # OK


def calc_to_goal_cost(trajectory, goal):
    """
    calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(
        x,
        y,
        length * math.cos(yaw),
        length * math.sin(yaw),
        head_length=width,
        head_width=width,
    )
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array(
            [
                [
                    -config.robot_length / 2,
                    config.robot_length / 2,
                    (config.robot_length / 2),
                    -config.robot_length / 2,
                    -config.robot_length / 2,
                ],
                [
                    config.robot_width / 2,
                    config.robot_width / 2,
                    -config.robot_width / 2,
                    -config.robot_width / 2,
                    config.robot_width / 2,
                ],
            ]
        )
        Rot1 = np.array(
            [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]]
        )
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(
            np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), "-k"
        )
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (
            np.array([x, y])
            + np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius
        )
        plt.plot([x, out_x], [y, out_y], "-k")


def rotate_waypoints(x, y, wx, wy):
    # Find closest waypoint
    idx = 0
    min_diff = np.inf
    for i, (_wx, _wy) in enumerate(zip(wx, wy)):
        diff = np.hypot(_wx - x, _wy - y)
        if diff < min_diff:
            idx = i
            min_diff = diff

    # Move through waypoints
    wx = np.concatenate((wx[idx:], wx[:idx]), axis=0)
    wy = np.concatenate((wy[idx:], wy[:idx]), axis=0)

    return wx, wy


def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
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

    # TODO: Properly load the waypoints and sample them

    # load way points from a file (flip and filter based on WP_D_F)
    WP_D_F = 18
    wx = np.loadtxt(os.path.join(path, "rx.npy"))[::-WP_D_F]
    wy = np.loadtxt(os.path.join(path, "ry.npy"))[::-WP_D_F]

    # Flip grid
    grid = grid[::-1, ::]

    # start position
    sx = -pose[1]
    sy = pose[0]
    _, _, yaw = euler_from_quaternion(*pose[3:])

    # temporary hardcode
    sx = wx[-1]
    sy = wy[-1]
    yaw = 0

    # Set config for our robot
    config.robot_type = RobotType.rectangle
    config.robot_length = 0.2
    config.robot_width = 0.1
    config.max_speed = 2.0
    config.max_accel = 0.5
    config.v_resolution = 0.05
    config.dt = 0.5

    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([sx, sy, yaw, 0.0, 0.0])

    wx, wy = rotate_waypoints(sx, sy, wx, wy)

    # goal position [x(m), y(m)]
    goal = np.array([wx[0], wy[0]])
    # input [forward speed, yaw_rate]

    trajectory = np.array(x)
    # ob = config.ob
    ob = np.argwhere(grid == 100)
    downsample_factor = 1
    ox, oy = (
        list(ob[::downsample_factor, 0] * map_resolution + map_origin_x),
        list(ob[::downsample_factor, 1] * map_resolution + map_origin_y),
    )
    ob = np.hstack([np.reshape(ox, (len(ox), 1)), np.reshape(oy, (len(oy), 1))])
    while True:
        u, predicted_trajectory = dwa_control(
            x, config, goal, (grid, map_metadata, map_origin)
        )
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            # plt.plot(goal[0], goal[1], "xb")
            plt.plot(wx, wy, "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok", markersize=2)
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            # Move through waypoints
            wx = np.concatenate((wx[1:], wx[:1]), axis=0)
            wy = np.concatenate((wy[1:], wy[:1]), axis=0)

            # Set new goal
            goal = np.array([wx[0], wy[0]])
            print(f"Reached waypoint, next goal is {goal}")

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)
        plt.show()


if __name__ == "__main__":
    main(robot_type=RobotType.rectangle)
