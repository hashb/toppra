import time
from toppra import constants
from toppra.interpolator import AbstractGeometricPath
from .time_optimal_algorithm import TOPPRA
import logging
import numpy as np
import rerun as rr
from dataclasses import dataclass, Field

logger = logging.getLogger(__name__)

rr.init("jerk_toppra", spawn=False)


@dataclass
class Node:
    i: int
    x: float
    s: float

    cost: float = 0.0

    parent: "Node | None" = None
    children: "list[Node] | None" = None

    @property
    def u_prev(self):
        if self.parent is not None:
            return 0.5 * (self.x - self.parent.x) / (self.s - self.parent.s)
        return 0

    @property
    def s_jerk_prev(self):
        if self.parent is not None and self.parent.parent is not None:
            return (
                (self.u_prev - self.parent.u_prev)
                * np.sqrt(self.parent.x)
                / (self.parent.s - self.parent.parent.s)
            )
        return 0

    def __str__(self) -> str:
        return f"Node(i={self.i}, x={self.x}, s={self.s}, u-1={self.u_prev}, cost={self.cost})\n"

    def __repr__(self) -> str:
        return self.__str__()


def cost_fn(parent: Node, child: Node):
    return parent.cost + (2 * (child.s - parent.s) / (child.x + parent.x + 1e-8))


class JerkLimitedTOPPRA(TOPPRA):

    def __init__(
        self,
        constraint_list,
        path,
        gridpoints=None,
        solver_wrapper="seidel",
        # solver_wrapper="cvxpy",
        parametrizer="ParametrizeSpline",
        **kwargs,
    ):
        super().__init__(
            constraint_list,
            path,
            gridpoints,
            solver_wrapper,
            parametrizer,
            **kwargs,
        )
        self._jerk_limit = kwargs.get("jerk_limit", 200)
        self.params = [
            c.compute_constraint_params(self.path, self.gridpoints)
            for c in self.constraints
        ]

    def _uniform_sample(self, idx, s, sd_min, sd_max, n_samples=6) -> list[Node]:
        x_samples = np.random.uniform(sd_min, sd_max, n_samples)
        children = []
        for sample in x_samples:
            children.append(
                Node(
                    i=idx,
                    x=sample,
                    s=s,
                )
            )
        # import matplotlib.pyplot as plt

        # count, bins, ignored = plt.hist(x_samples, 15, density=True)
        # plt.plot(bins, np.ones_like(bins), linewidth=2, color="r")
        # plt.scatter(np.ones_like(x_samples) * s, x_samples, marker="x", color="r")
        # plt.show()
        # breakpoint()
        return children

    def _sample_x(self, idx, s, parent_nodes: list[Node], sd_min, sd_max) -> list[Node]:
        """for every x_{i-1} calculate xi using
        xi = x_{i-1} + 2 * delta_i * u_{i-2}

        add uniform samples of len(xi) * NUM_UNIFORM_SAMPLES_X
        """
        if len(parent_nodes) == 0 or parent_nodes[0].parent is None:
            return self._uniform_sample(idx, s, sd_min, sd_max, n_samples=6)

        samples = []
        for parent in parent_nodes:
            new_x = parent.x + 2 * (s - parent.s) * parent.u_prev
            if new_x < sd_min or new_x > sd_max:
                continue
            samples.append(Node(i=idx, x=new_x, s=s))
        return samples + self._uniform_sample(idx, s, sd_min, sd_max, n_samples=6)

    def _near_parents(self, x: float, v_open: list[Node], r: int) -> list[Node]:
        """return all nodes in v_open that are within r distance from x"""
        # sort v_open by x
        v_open.sort(key=lambda node: node.x - x)
        return v_open  # [: min(len(v_open), r)]

    def _find_parent(self, curr_node: Node, v_near_prev) -> Node | None:
        """
        calculate cost for every node in v_near
        sort the cost
        return the node with the lowest cost and satisfies the constraints
        """
        v_near_prev.sort(key=lambda node: cost_fn(node, curr_node))
        for parent in v_near_prev:
            if self._check_constraints(parent, curr_node):
                return parent
        return None

    def _check_constraints(self, parent: Node, child: Node):
        """check if the constraints are satisfied"""
        i = child.i  # I think we check the constraints for the parent node
        s = child.s
        x = child.x
        u = 0.5 * (x - parent.x) / (s - parent.s)
        s_jerk = (u - parent.u_prev) * np.sqrt(parent.x) / (s - parent.s + 1e-8)

        # check standard constraints
        for k, constraint in enumerate(self.constraints):
            a, b, c, F, h, ubound, xbound = self.params[k]

            if a is not None:
                v = a[i] * u + b[i] * x + c[i]
                if constraint.identical:
                    if not np.all(F.dot(v) <= h):
                        return False
                elif not np.all(F[i].dot(v) <= h[i]):
                    return False

            if ubound is not None:
                if not (max(-constants.MAXU, ubound[i, 0]) <= u):
                    return False
                if not (u <= min(constants.MAXU, ubound[i, 1])):
                    return False

            if xbound is not None:
                if not (xbound[i, 0] <= x):
                    return False
                if not (x <= min(constants.MAXX, xbound[i, 1])):
                    return False

        # check jerk constraints
        q_dash = self.path(parent.s, 1)
        q_2dash = self.path(parent.s, 2)
        q_3dash = self.path(parent.s, 3)
        # print((q_dash * s_jerk) + (3 * q_2dash * u * x) + (q_3dash * x * np.sqrt(x)))
        if not np.all(
            (q_dash * s_jerk) + (3 * q_2dash * u * x) + (q_3dash * x * np.sqrt(x))
            <= self._jerk_limit
        ):
            return False

        return True

    def compute_parameterization(self, sd_start, sd_end, return_data=False):
        """_summary_

        Args:
            sd_start (_type_): _description_
            sd_end (_type_): _description_
            return_data (bool, optional): _description_. Defaults to False.

        Returns:
            _type_: _description_
        """

        # sample range must start with 0
        # use reachable sets instead of controllable set

        # we only need min, max of the x value.
        # for sampling we dont even use u, we just use x

        t1 = time.monotonic()
        super().compute_parameterization(sd_start, sd_end, return_data)
        t2 = time.monotonic()
        print(f"Time taken for TOPPRA: {t2 - t1}")

        constraint_params = self.solver_wrapper.params
        # breakpoint()

        # check if path is reachable

        # samapling to find x and u that satisfy the constraints

        # input x_0 = 0, q', q'', q'''
        # output [x_0, x_1,..... x_n]

        # tree T = {x_0}

        # iter 0 = 0
        # iter 1 = uniform sample
        # u_i = x_i+1 - x_i / 2 * delta_i
        # delta_i = s_i+1 - s_i

        # s'''_i-1 = u_i-1 - u_i * sqrt(x_i-1) / delta_i-2
        assert self.problem_data is not None
        assert self.problem_data.K is not None
        assert self.problem_data.L is not None
        assert self.problem_data.gridpoints is not None

        M = np.zeros(self.problem_data.K.shape)
        M[:, 0] = np.maximum(self.problem_data.K[:, 0], self.problem_data.L[:, 0])
        M[:, 1] = np.minimum(self.problem_data.K[:, 1], self.problem_data.L[:, 1])

        graph: list[list[Node]] = [[] for _ in range(len(self.problem_data.gridpoints))]

        x_0 = Node(0, 0, 0)
        graph[0].append(x_0)

        v_open: list[list[Node]] = [
            [] for _ in range(len(self.problem_data.gridpoints))
        ]
        v_open[0].append(x_0)

        # plot graph
        rr.set_time_seconds("stable_time", 0)
        rr.log("K_max", rr.SeriesLine(color=[0, 165, 255], name="K_max"), timeless=True)
        rr.log("K_min", rr.SeriesLine(color=[0, 255, 255], name="K_min"), timeless=True)

        rr.log(
            "L_max", rr.SeriesLine(color=[165, 165, 25], name="L_max"), timeless=True
        )
        rr.log(
            "L_min", rr.SeriesLine(color=[165, 255, 25], name="L_min"), timeless=True
        )

        rr.log("M_max", rr.SeriesLine(color=[255, 165, 0], name="M_max"), timeless=True)
        rr.log("M_min", rr.SeriesLine(color=[165, 255, 0], name="M_min"), timeless=True)
        rr.log(
            "x",
            rr.SeriesPoint(color=[0, 0, 255], name="x", marker_size=1.5),
            timeless=True,
        )

        for idx, s in enumerate(self.problem_data.gridpoints):
            if idx == 0:
                continue

            # calculate V_unvisited for every parent node
            # x_i = x_{i-1} + 2 * delta_i * u_{i-2}
            parent_nodes = graph[idx - 1]
            v_unvisited = self._sample_x(idx, s, parent_nodes, M[idx, 0], M[idx, 1])

            rr.set_time_seconds("stable_time", s)
            rr.log("M_max", rr.Scalar(M[idx, 1]))
            rr.log("M_min", rr.Scalar(M[idx, 0]))

            rr.log("K_max", rr.Scalar(self.problem_data.K[idx, 1]))
            rr.log("K_min", rr.Scalar(self.problem_data.K[idx, 0]))

            rr.log("L_max", rr.Scalar(self.problem_data.L[idx, 1]))
            rr.log("L_min", rr.Scalar(self.problem_data.L[idx, 0]))

            for x in v_unvisited:
                rr.log("x", rr.Scalar(x.x))

            time.sleep(0.001)

            epsilon = 10
            r = int(np.ceil(epsilon * (M[idx, 1] - M[idx, 0]) / len(v_unvisited)))
            r = max(r, 15)
            # print(f"{idx=}, {r=}\n\n")
            for u_node in v_unvisited:
                # find parent nodes that are near x
                v_near_prev = self._near_parents(u_node.x, v_open[idx - 1], r)
                # print(v_near_prev)
                # find the parent node that has the lowest cost
                y = self._find_parent(u_node, v_near_prev)
                # add x to the graph
                if y is not None:
                    v_open[idx].append(u_node)

                    u_node.parent = y
                    u_node.cost = cost_fn(y, u_node)

                    # print(u_node, y)

                    graph[idx].append(u_node)

            # print("\n\n")
        # find the node with the lowest cost
        min_cost_node = min(graph[-1], key=lambda node: node.cost)
        path = [min_cost_node]
        while path[-1].parent is not None:
            path.append(path[-1].parent)

        path.reverse()
        print(path)
        print(len(path))
        sdd_vec = np.array([node.u_prev for node in path])
        sd_vec = np.sqrt(np.array([node.x for node in path]))

        self.problem_data.sdd_vec = sdd_vec
        self.problem_data.sd_vec = sd_vec

        return sdd_vec, sd_vec, None
        # breakpoint()
