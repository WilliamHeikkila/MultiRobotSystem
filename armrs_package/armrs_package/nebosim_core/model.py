import numpy as np


class Dynamic:
    """
    Robot's dynamical model.
    It can consider a simpler kinematic model or a more complex one.
    This class is a template, to implement continuous model in digital via Forward Euler approach.
    A new class can inherit this class and adjust the compute_dot_state and/or add new methods.
    """

    def __init__(self, dt):
        """
        Initialize variables to contain the state and input.
        Each state and input are defined inside a dictionary,
        to allow different naming convention between each model.

        :param dt: (default) time sampling for update
        """
        self.model_name = self.__class__.__name__
        self.Ts = dt
        self.state = {}
        self.dot_state = {}
        self.input = {}
        self.model_name = None

    def compute_dot_state(self):
        """
        Define the robot's model as a relation of input and state to dot_state
        for example,
            self.dot_state["q"] = self.state["q"] + self.input["u"]
        """
        pass

    def update(self, Ts=None):
        """
        Update the next state based on Forward Euler computation.
        Allow computation with varying time sampling, but only this instance.
        If not defined again in the next step it default back to existing self.dt,
        for permanent changes of dt, directly set self.dt.

        :return:
        """
        dt = self.Ts if Ts is None else Ts

        # Compute next state (based on nominal model)
        self.compute_dot_state()

        # Increment from past to present state
        for k, v in self.state.items():
            self.state[k] = v + dt * self.dot_state[k]

        return self.state


class SingleIntegrator(Dynamic):
    """
    A single integrator (kinematic) model for a robot in 3 dimension.
    Also ofter refered as point dynamics
    State: Position as pos = [q_x q_y q_z]
    Input: Velocity Input as vel = [u_x u_y u_z]
    Dynamics: dot(pos) = vel

    Note:
    in mathematical formulation, the q and u is usually presented as column vector (3x1)
    But for this implementation we opted for row vector with 1D numpy array.
    """

    def __init__(self, dt, *,
                 init_pos=np.array([0., 0., 0.]), init_vel=np.array([0., 0., 0.]), robot_ID=None):
        """
        :param dt: (default) time sampling for update
        :param init_pos: robot's initial position in numpy array
        :param init_vel: robot's initial velocity input in numpy array
        :param robot_ID: to identify different robot for multi-robot scenarios
        """

        super().__init__(dt)
        self.robot_ID = robot_ID

        self.state["pos"] = init_pos
        self.input["vel"] = init_vel
        self.dot_state["pos"] = np.array([0., 0., 0.])

    def compute_dot_state(self):
        """
        Dynamics: dot(q) = vel
        """
        self.dot_state["pos"] = self.input["vel"]

    def set_input(self, vel):
        """
        :param vel: velocity input in numpy array
        """
        self.input["vel"] = vel


class Unicycle(Dynamic):
    """
    A unicycle dynamic for a robot in planar plane (3 dimension with z=0).
    State: Position as pos = [q_x q_y q_z], orientation as theta = th
    Input: Linear velocity as linV = V, Angular Velocity as angV = omg
    Dynamics:
        dot(q) = [linV*cos(theta), linV*sin(theta), 0.]
        dot(theta) = angV
    """

    def __init__(self, dt, *,
                 init_pos=np.array([0., 0., 0.]), init_theta=0.,
                 init_linV=0., init_angV=0.,
                 robot_ID=None, look_ahead_dist=-0.1,
                 max_linV=-0.1, max_angV=-0.1):
        """
        :param dt: (default) time sampling for update
        :param init_pos: robot's initial position in numpy array
        :param init_vel: robot's initial velocity input in numpy array
        :param robot_ID: to identify different robot for multi-robot scenarios

        :param look_ahead_dist: set the point with a distance [m] ahead of robot,
        to transformation from world velocity input into linV and angV
        (look_ahead_dist > 0, enabling control of the look ahead point as a single integrator)

        :param dt:
        """

        super().__init__(dt)
        self.robot_ID = robot_ID

        self.state["pos"], self.state["theta"] = init_pos, init_theta
        self.input["linV"], self.input["angV"] = init_linV, init_angV
        self.dot_state["pos"] = np.array([0., 0., 0.])
        self.dot_state["theta"] = 0.

        self.look_ahead_dist = look_ahead_dist
        self.max_linV, self.max_angV = max_linV, max_angV

    def compute_dot_state(self):
        """
        Dynamics:
            dot(q) = [linV*cos(theta), linV*sin(theta), 0.]
            dot(theta) = angV
        """
        self.dot_state["pos"] = np.array([
            self.input["linV"] * np.cos(self.state["theta"]),
            self.input["linV"] * np.sin(self.state["theta"]),
            0.
        ])
        self.dot_state["theta"] = self.input["angV"]

    @staticmethod
    def impose_unicycle_saturation(linV, angV, max_linV, max_angV):
        """
        :param linV: float, linear velocity input
        :param angV: float, angular velocity input
        :param max_linV: float, maximum linear velocity input
        :param max_angV: float, maximum angular velocity input
        """
        if max_linV > 0. and max_angV > 0.:
            sat_linV, sat_angV = linV, angV
            if (max_linV > 0.) and (abs(linV) >= max_linV):
                sat_linV = max_linV * linV / abs(linV)
            if (max_angV > 0.) and (abs(angV) >= max_angV):
                sat_angV = max_angV * angV / abs(angV)
            return sat_linV, sat_angV
        else:
            return linV, angV
        
    def set_input_unicycle(self, linV, angV):
        """
        :param linV: float, linear velocity input
        :param angV: float, angular velocity input
        """
        linV, angV = self.impose_unicycle_saturation(linV, angV, self.max_linV, self.max_angV)
        self.input["linV"], self.input["angV"] = linV, angV

    def set_input_lookahead(self, vel):
        """
        Inverse Look up ahead Mapping (u_z remain 0.)
            linV = u_x cos(theta) + u_y sin(theta)
            angV = (- u_x sin(theta) + u_y cos(theta)) / l

        :param vel: velocity input in numpy array
        """
        assert (self.look_ahead_dist > 0.), \
            f"Input with lookahead is disabled. look_ahead_dist={self.look_ahead_dist} should be > 0."

        th = self.state["theta"]

        # do SI to unicycle conversion
        Ml = np.array([[1, 0], [0, 1 / self.look_ahead_dist]])
        Mth = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])
        current_input = Ml @ Mth @ vel[:2]

        linV, angV = current_input[:2]
        linV, angV = self.impose_unicycle_saturation(linV, angV, self.max_linV, self.max_angV)

        self.input["linV"] = linV
        self.input["angV"] = angV
