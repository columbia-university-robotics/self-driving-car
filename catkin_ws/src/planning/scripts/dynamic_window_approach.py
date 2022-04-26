
import math
from enum import Enum
import datetime


import numpy as np

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist

import numpy as np

def quaternion_to_euler_angle_vectorized(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

class RobotType(Enum):
    circle = 0
    rectangle = 1


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


class DWAControl:
    """Dynamic Window Approach for motion planning.

    :param config: configuration for agent
    :param goal: (x, y) coordinate for goal.
    """

    def __init__(self, config=None, demo=False):
        self.config = config
        """initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]"""
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.trajectory = np.array([self.state])
        if demo:
            self.goal = [10.0, 10.0]
            self.ob = np.array(
                [
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [8.0, 10.0],
                    [9.0, 11.0],
                    [12.0, 12.0],
                    [15.0, 15.0],
                    [13.0, 13.0],
                ]
            )
        else:
            rospy.init_node("dwa_planner")
            self.goal = [np.nan, np.nan]
            self.ob = np.array([])
            self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
            self.ob_sub = rospy.Subscriber("/map", OccupancyGrid, self.ob_cb)
            self.state_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.state_cb)
            self.action_pub = rospy.Publisher("/cmd", Twist)

    def state_cb(self, data):
        """Callback for state subscriber.
        """
	X, Y, yaw = quaternion_to_euler_angle_vectorized(
		data.pose.pose.orientation.w, 
		data.pose.pose.orientation.x, 
		data.pose.pose.orientation.y, 
		data.pose.pose.orientation.z)

        self.state = np.array(
            [
                data.pose.pose.position.x,
                data.pose.pose.position.y,
                yaw,
		data.twist.twist.linear.x,
		data.twist.twist.angular.z,
            ]
        )

    def ob_cb(self, data):
        """Callback for obstacle subscriber.
        """
        hw_arr = np.array(data.data).reshape((data.info.height, data.info.width))
        idx = np.argwhere(hw_arr == 100)
        idx = idx.astype(float) * data.info.resolution
        idx[:,0] += data.info.origin.position.x
        idx[:,1] += data.info.origin.position.y
        self.ob = idx

    def goal_cb(self, data):
        """Callback for goal subscriber.
        """
        self.goal = [data.pose.position.x, data.pose.position.y]

    def move(self, action):
        """Update the state of the agent with an action.

        :param action: action in form (x_v_robot_frame, yaw_rate_robot_frame)
        """
        self.state[2] += action[1] * self.config.dt
        self.state[0] += action[0] * math.cos(self.state[2]) * self.config.dt
        self.state[1] += action[0] * math.sin(self.state[2]) * self.config.dt
        self.state[3] = action[0]
        self.state[4] = action[1]

    @staticmethod
    def simulate_motion(state, action, dt):
        """Simulate a new state with an action.

        :param state: Agent state in form (x, y, yaw, x_v_robot_frame, yaw_rate_robot_frame)
        :param action: action in form (x_v_robot_frame, yaw_rate_robot_frame)
        :param dt: Timedelta for action (how long to run action)
        """
        state[2] += action[1] * dt
        state[0] += action[0] * math.cos(state[2]) * dt
        state[1] += action[0] * math.sin(state[2]) * dt
        state[3] = action[0]
        state[4] = action[1]

        return state

    def calculate_dynamic_window_search_space(self):
        """Calculate the search space for Dynamic Window Approach."""
        min_v = max(
            -self.config.max_speed,
            self.state[3] - self.config.max_accel * self.config.dt,
        )
        max_v = min(
            self.config.max_speed,
            self.state[3] + self.config.max_accel * self.config.dt,
        )
        min_omega = max(
            -self.config.max_yaw_rate,
            self.state[4] - self.config.max_yaw_rate * self.config.dt,
        )
        max_omega = min(
            self.config.max_yaw_rate,
            self.state[4] + self.config.max_yaw_rate * self.config.dt,
        )
        return min_v, max_v, min_omega, max_omega

    def predict_trajectory(self, v, omega):
        """Predict trajectory based on a linear velocity and angular velocity

        :param v: Linear velocity of agent (x_v_robot_frame)
        :param omega: Angular velocity of agent. (yaw_rate_robot_frame)
        """
        temp_state = np.array(self.state)
        trajectory = np.array(temp_state)
        time = 0
        while time <= self.config.predict_time:
            temp_state = self.simulate_motion(temp_state, [v, omega], self.config.dt)
            trajectory = np.vstack((trajectory, temp_state))
            time += self.config.dt

        return trajectory

    def calc_obstacle_cost(self, trajectory, ob):
        """Calculate cost of collision with obstacles."""
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if self.config.robot_type == RobotType.rectangle:
            yaw = trajectory[:, 2]
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            rot = np.transpose(rot, [2, 0, 1])
            local_ob = ob[:, None] - trajectory[:, 0:2]
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            # local_ob = np.array([local_ob @ x for x in rot]) TODO python2 is sh*t
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            upper_check = local_ob[:, 0] <= self.self.config.robot_length / 2
            right_check = local_ob[:, 1] <= self.config.robot_width / 2
            bottom_check = local_ob[:, 0] >= -self.config.robot_length / 2
            left_check = local_ob[:, 1] >= -self.config.robot_width / 2
            if (
                np.logical_and(
                    np.logical_and(upper_check, right_check),
                    np.logical_and(bottom_check, left_check),
                )
            ).any():
                return float("Inf")
        elif self.config.robot_type == RobotType.circle:
            if np.array(r <= self.config.robot_radius).any():
                return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK

    def calc_to_goal_cost(self, trajectory):
        """

        :param trajectory:
        :return:
        """
        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def calc_control_and_trajectory(self, dw, ob):
        """

        :param dw:
        :param ob:
        :return:
        """
        # x_init = self.state[:]
        min_cost = float("inf")
        best_action = [0.0, 0.0]
        best_trajectory = np.array([self.state])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                trajectory = self.predict_trajectory(v, omega)
                # calc cost
                to_goal_cost = self.config.to_goal_cost_gain * self.calc_to_goal_cost(
                    trajectory
                )
                speed_cost = self.config.speed_cost_gain * (
                    self.config.max_speed - trajectory[-1, 3]
                )
                ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(
                    trajectory, ob
                )

                final_cost = to_goal_cost + speed_cost + ob_cost
                # search minimum trajectory
                if min_cost >= final_cost:
                    # print(final_cost, (v, omega))
                    min_cost = final_cost
                    best_action = [v, omega]
                    best_trajectory = trajectory
                    if (
                        abs(best_action[0]) < self.config.robot_stuck_flag_cons
                        and abs(self.state[3]) < self.config.robot_stuck_flag_cons
                    ):
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_action[1] = -self.config.max_delta_yaw_rate

        return best_action, best_trajectory

    def __call__(self, *args, **kwargs):

        ob = np.array(
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

        self.rate = rospy.Rate(50)	

        while not rospy.is_shutdown():  # TODO: change to some ROS thing

            dw = self.calculate_dynamic_window_search_space()
            best_action, predicted_trajectory = self.calc_control_and_trajectory(dw, ob)
            self.move(best_action)  # move robot
            self.trajectory = np.vstack((self.trajectory, self.state))
            
            if self.goal != [np.nan, np.nan]:
                twist = Twist()
                twist.linear.x = best_action[0]
                twist.angular.z = best_aciton[1]
                self.action_pub.publish(Twist)

            if kwargs.get("animate", False):
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: [exit(0) if event.key == "escape" else None],
                )
                plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
                plt.plot(self.state[0], self.state[1], "xr")
                plt.plot(self.goal[0], self.goal[1], "xb")
                plt.plot(ob[:, 0], ob[:, 1], "ok")
                plot_robot(self.state[0], self.state[1], self.state[2], self.config)
                plot_arrow(self.state[0], self.state[1], self.state[2])
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.0001)

            # check reaching goal
            dist_to_goal = math.hypot(
                self.state[0] - self.goal[0], self.state[1] - self.goal[1]
            )
            if kwargs.get("print_state", False):
		pass
                # print(f"dist to goal: {dist_to_goal}; state: {self.state}")
            if dist_to_goal <= self.config.robot_radius:
                print("Goal!!")
                # break
	self.rate.sleep()


class Config:
    """
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]
        self.min_speed = 0  # [m/s]
        self.max_accel = 1.5  # [m/ss]
        self.max_yaw_rate = 60.0 * math.pi / 180.0  # [rad/s]

        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1.5  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.162  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


if __name__ == "__main__":
    dwa = DWAControl(config=Config())
    dwa()
