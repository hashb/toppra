from toppra.interpolator import AbstractGeometricPath
from .time_optimal_algorithm import TOPPRA
import logging
import numpy as np

from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass()
class Node:
    x: float
    s: float

    parent: "Node | None" = None
    children: "list[Node]" = []

    j: float = 0
    u: float = 0

    c: float = 0


class JerkLimitedTOPPRA(TOPPRA):

    def _uniform_sample(self, s, sd_min, sd_max, n_samples=50) -> list[Node]:
        x_samples = np.random.uniform(sd_min, sd_max, n_samples)
        children = []
        for sample in x_samples:
            children.append(
                Node(
                    x=sample,
                    s=s,
                )
            )
        return children

    def _sample_x(self, s, sd_min, sd_max) -> list[Node]:
        return []

    def _compute_cost(self, prev_node, new_node):
        pass

    def _near_parents(self, x, v_open, r) -> list[Node]:
        return []

    def compute_trajectory(
        self, sd_start: float = 0, sd_end: float = 0
    ) -> AbstractGeometricPath | None:
        """"""
        # sample range must start with 0
        # use reachable sets instead of controllable set

        # we only need min, max of the x value.
        # for sampling we dont even use u, we just use x

        # return super().compute_trajectory(sd_start, sd_end)
        self.compute_parameterization(sd_start, sd_end)

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

        graph: list[list[Node]] = []

        x_0 = Node(0, 0)
        graph.append([x_0])

        v_open: list[list[Node]] = []
        v_open.append([x_0])

        for idx, s in enumerate(self.problem_data.gridpoints):
            if idx == 0:
                continue

            v_unvisited = []
            if idx == 1:
                v_unvisited = self._uniform_sample(s, M[idx, 0], M[idx, 1])
            else:
                v_unvisited = self._sample_x(s, M[idx, 0], M[idx, 1])

            epsilon = 10
            r = epsilon * (M[idx, 1] - M[idx, 0]) / len(v_unvisited)
            for x in v_unvisited:
                # find parent nodes that are near x
                v_near_i_1 = self._near_parents(x, v_open[idx - 1])
                # find the parent node that has the lowest cost
