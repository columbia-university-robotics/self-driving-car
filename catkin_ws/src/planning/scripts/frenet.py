import numpy as np
import copy
import math
import sys
import pathlib
import bisect

import rospy

from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

# Sim Params
SIM_LOOP = 500
SIM_WINDOW_OFFSET = 3
OB_D_F = 4  # obstacles downsample factor

# Parameter
MAX_SPEED = 2.0  # maximum speed [m/s]
MAX_ACCEL = 10.0  # maximum acceleration [m/ss] # Increase this value for standing start
MAX_CURVATURE = 22.0  # maximum curvature [1/m]  # Increase this value to run the demo
MAX_ROAD_WIDTH = 7.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 0.25  # time tick [s]
MAX_T = 2.0  # max prediction time [m]
MIN_T = 1.0  # min prediction time [m]
TARGET_SPEED = 1.8  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 0.2  # robot radius [m]
DIST_TO_GOAL = 0.1

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

WP_D_F = 5  # waypoint downsample factor


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


class QuarticPolynomial:
    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time**2, 4 * time**3], [6 * time, 12 * time**2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time, axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = (
            self.a0
            + self.a1 * t
            + self.a2 * t**2
            + self.a3 * t**3
            + self.a4 * t**4
        )

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class QuinticPolynomial:
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # calc coefficient of quintic polynomial
        # See jupyter notebook document for derivation of this equation.
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array(
            [
                [time**3, time**4, time**5],
                [3 * time**2, 4 * time**3, 5 * time**4],
                [6 * time, 12 * time**2, 20 * time**3],
            ]
        )
        b = np.array(
            [
                xe - self.a0 - self.a1 * time - self.a2 * time**2,
                vxe - self.a1 - 2 * self.a2 * time,
                axe - 2 * self.a2,
            ]
        )
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = (
            self.a0
            + self.a1 * t
            + self.a2 * t**2
            + self.a3 * t**3
            + self.a4 * t**4
            + self.a5 * t**5
        )

        return xt

    def calc_first_derivative(self, t):
        xt = (
            self.a1
            + 2 * self.a2 * t
            + 3 * self.a3 * t**2
            + 4 * self.a4 * t**3
            + 5 * self.a5 * t**4
        )

        return xt

    def calc_second_derivative(self, t):
        xt = (
            2 * self.a2
            + 6 * self.a3 * t
            + 12 * self.a4 * t**2
            + 20 * self.a5 * t**3
        )

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt


class CubicSpline1D:
    """
    1D Cubic Spline class

    Parameters
    ----------
    x : list
        x coordinates for data points. This x coordinates must be
        sorted
        in ascending order.
    y : list
        y coordinates for data points

    Examples
    --------
    You can interpolate 1D data points.

    >>> import numpy as np
    >>> import matplotlib.pyplot as plt
    >>> x = np.arange(5)
    >>> y = [1.7, -6, 5, 6.5, 0.0]
    >>> sp = CubicSpline1D(x, y)
    >>> xi = np.linspace(0.0, 5.0)
    >>> yi = [sp.calc_position(x) for x in xi]
    >>> plt.plot(x, y, "xb", label="Data points")
    >>> plt.plot(xi, yi , "r", label="Cubic spline interpolation")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.show()

    .. image:: cubic_spline_1d.png

    """

    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) - h[i] / 3.0 * (
                2.0 * self.c[i] + self.c[i + 1]
            )
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        """
        Calc `y` position for given `x`.

        if `x` is outside the data point's `x` range, return None.

        Returns
        -------
        y : float
            y position for given x.
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = (
            self.a[i] + self.b[i] * dx + self.c[i] * dx**2.0 + self.d[i] * dx**3.0
        )

        return position

    def calc_first_derivative(self, x):
        """
        Calc first derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        dy : float
            first derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2.0
        return dy

    def calc_second_derivative(self, x):
        """
        Calc second derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        ddy : float
            second derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = (
                3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i]
            )
        return B


class CubicSpline2D:
    """
    Cubic CubicSpline2D class

    Parameters
    ----------
    x : list
        x coordinates for data points.
    y : list
        y coordinates for data points.

    Examples
    --------
    You can interpolate a 2D data points.

    >>> import matplotlib.pyplot as plt
    >>> x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    >>> y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    >>> ds = 0.1  # [m] distance of each interpolated points
    >>> sp = CubicSpline2D(x, y)
    >>> s = np.arange(0, sp.s[-1], ds)
    >>> rx, ry, ryaw, rk = [], [], [], []
    >>> for i_s in s:
    ...     ix, iy = sp.calc_position(i_s)
    ...     rx.append(ix)
    ...     ry.append(iy)
    ...     ryaw.append(sp.calc_yaw(i_s))
    ...     rk.append(sp.calc_curvature(i_s))
    >>> plt.subplots(1)
    >>> plt.plot(x, y, "xb", label="Data points")
    >>> plt.plot(rx, ry, "-r", label="Cubic spline path")
    >>> plt.grid(True)
    >>> plt.axis("equal")
    >>> plt.xlabel("x[m]")
    >>> plt.ylabel("y[m]")
    >>> plt.legend()
    >>> plt.show()

    .. image:: cubic_spline_2d_path.png

    >>> plt.subplots(1)
    >>> plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.xlabel("line length[m]")
    >>> plt.ylabel("yaw angle[deg]")

    .. image:: cubic_spline_2d_yaw.png

    >>> plt.subplots(1)
    >>> plt.plot(s, rk, "-r", label="curvature")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.xlabel("line length[m]")
    >>> plt.ylabel("curvature [1/m]")

    .. image:: cubic_spline_2d_curvature.png
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        k : float
            curvature for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx**2 + dy**2) ** (3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw


class FrenetPath:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []


def calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):
        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(
                TARGET_SPEED - D_T_S * N_S_SAMPLE,
                TARGET_SPEED + D_T_S * N_S_SAMPLE,
                D_T_S,
            ):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):
    for fp in fplist:
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision_ob(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [
            ((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
            for (ix, iy) in zip(fp.x, fp.y)
        ]

        collision = any([di <= ROBOT_RADIUS**2 for di in d])

        if collision:
            return False

    return True


def check_collision_og(fp, og):
    og, (og_resolution, _, _), (og_origin_x, og_origin_y, *_) = og
    og_resolution = round(og_resolution, 5)
    for ix, iy in zip(fp.x, fp.y):
        x = int((ix - og_origin_x) / og_resolution)
        y = int((iy - og_origin_y) / og_resolution)

        collision = any(
            [
                og[x + xf, y + yf]
                for xf in range(-int(ROBOT_RADIUS) - 1, int(ROBOT_RADIUS) + 2)
                for yf in range(-int(ROBOT_RADIUS) - 1, int(ROBOT_RADIUS) + 2)
            ]
        )

        if collision:
            return False

    return True


def check_paths(fplist, og):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        if any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
            continue
        if any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
            continue
        if not check_collision_og(fplist[i], og):
            continue
        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, og):
    """
    Parameters
    ----------
    csp : CubicSpline2D
        cubic spline planner
    s0 : float
        current course position
    c_speed : float
        current speed [m/s]
    c_accel : float
        current acceleration [m/ss]
    c_d : float
        current lateral position [m]
    c_d_d : float
        current lateral speed [m/s]
    c_d_dd : float
        corrent lateral acceleration [m/s]
    """
    fplist = calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
    fplist = calc_global_paths(fplist, csp)
    fplist = check_paths(fplist, og)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    """
    Parameters
    ----------
    x : list
        x coordinates of waypoints
    y : list
        y coordinates of waypoints
    """
    csp = CubicSpline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


class FrenetPlanner:
    def __init__(
        self, initial_pose, wx, wy, occ_grid, occ_grid_metadata, occ_grid_origin
    ) -> None:
        self.c_speed = 0.0  # current speed [m/s]
        self.c_accel = 0.0  # current acceleration [m/ss]
        self.c_d = 0.0  # current lateral position [m]
        self.c_d_d = 0.0  # current lateral speed [m/s]
        self.c_d_dd = 0.0  # current lateral acceleration [m/s]
        self.s0 = 0.0  # current course position
        self.yaw = math.pi / 2  # current yaw [rad]

        x, y, yaw = initial_pose
        self.x = x
        self.y = y
        self.yaw = yaw

        self.wx = wx  # x-coord waypoints
        self.wy = wy  # y-coord waypoints
        self._rotate_waypoints()

        self.occ_grid = occ_grid  # occupancy grid
        self.occ_grid_metadata = occ_grid_metadata
        self.occ_grid_origin = occ_grid_origin

        self.show_animation = False

        rospy.init_node("frenet_planner")

        self.map_subscriber = rospy.Subscriber(
            "/map", OccupancyGrid, self._map_callback
        )
        self.pose_subscriber = rospy.Subscriber(
            "/slam_out_pose", PoseStamped, self._pose_callback
        )
        self.speed_topic = rospy.Publisher("systems/output/speed", Float64)
        self.steer_angle_topic = rospy.Publisher("systems/output/steer_angle", Float64)

    def _rotate_waypoints(self):
        # Find closest waypoint
        idx = 0
        min_diff = np.inf
        for i, (wx, wy) in enumerate(zip(self.wx, self.wy)):
            diff = np.hypot(wx - self.x, wy - self.y)
            if diff < min_diff:
                idx = i
                min_diff = diff

        # Move through waypoints
        self.wx = np.concatenate((self.wx[idx:], self.wx[:idx]), axis=0)
        self.wy = np.concatenate((self.wy[idx:], self.wy[:idx]), axis=0)

    def _map_callback(self, data):
        self.occ_grid = np.array(data.data).reshape(
            (data.info.width, data.info.height)
        )[::-1, ::-1]
        self.occ_grid_metadata = np.array(
            [
                data.info.resolution,
                data.info.width,
                data.info.height,
            ]
        )
        self.occ_grid_origin = np.array(
            [
                data.info.origin.position.x,
                data.info.origin.position.y,
                data.info.origin.position.z,
                data.info.origin.orientation.x,
                data.info.origin.orientation.y,
                data.info.origin.orientation.z,
                data.info.origin.orientation.w,
            ]
        )

    def _pose_callback(self, data):
        pose = np.array(
            [
                data.pose.position.x,
                data.pose.position.y,
                data.pose.position.z,
                data.pose.orientation.x,
                data.pose.orientation.y,
                data.pose.orientation.z,
                data.pose.orientation.w,
            ]
        )
        self.x = pose[0]
        self.y = pose[1]
        _, _, self.yaw = euler_from_quaternion(*pose[3:])

        self._rotate_waypoints()

    def __call__(self, *args, **kwargs):
        self.rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            csp = CubicSpline2D(self.wx, self.wy)

            # TODO: update current course pos, current lateral pos
            path = frenet_optimal_planning(
                csp,
                0,  # current course position (in reference to spline path)
                self.c_speed,
                self.c_accel,
                self.c_d,  # current lateral position (orthogonal to spline path)
                self.c_d_d,
                self.c_d_dd,
                (self.occ_grid, self.occ_grid_metadata, self.occ_grid_origin),
            )

            # Move to next position
            self.s0 = path.s[1]
            self.c_speed = path.s_d[1]
            self.c_accel = path.s_dd[1]
            self.c_d = path.d[1]
            self.c_d_d = path.d_d[1]
            self.c_d_dd = path.d_dd[1]

            # TODO Deal with speed and steering angle
            speed, yaw = path.s_d[1], path.yaw[1]
            print(speed, yaw)

        print("Finish")


if __name__ == "__main__":
    # load way points from a file (flip and filter based on WP_D_F)
    wx = np.loadtxt("/rx.npy")[::-WP_D_F]
    wy = np.loadtxt("/ry.npy")[::-WP_D_F]

    # Load map and metadata
    og = np.load("/occupancy_grid.npy")
    map_metadata = np.load("/map_metadata.npy")
    map_origin = np.load("/map_origin.npy")
    initial_pose = np.load("/pose.npy")

    # TODO: maybe deal with flipping grid, pose, maybe waypoints

    # Flip grid
    og = og.T[::-1, ::-1]

    # Unpack
    og = np.where(og == 100, 1.0, 0.0)
    map_origin_x, map_origin_y, *_ = map_origin
    map_resolution, map_width, map_height = map_metadata

    # Start position from initial pose
    sx = -initial_pose[0]
    sy = -initial_pose[1]
    _, _, yaw = euler_from_quaternion(*initial_pose[3:])
    yaw += np.pi / 2
    initial_pose = (sx, sy, yaw)

    # Create and run planner
    planner = FrenetPlanner(initial_pose, wx, wy, og, map_metadata, map_origin)
    planner()
