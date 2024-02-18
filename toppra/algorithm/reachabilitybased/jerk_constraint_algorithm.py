from toppra.interpolator import AbstractGeometricPath
from .time_optimal_algorithm import TOPPRA
import logging
import numpy as np

from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass()
class Node:
    children: "list[Node]"


class JerkLimitedTOPPRA(TOPPRA):

    def compute_trajectory(
        self, sd_start: float = 0, sd_end: float = 0
    ) -> AbstractGeometricPath | None:
        """"""
        # sample range must start with 0
        # use reachable sets instead of controllable set

        # we only need min, max of the x value.
        # for sampling we dont even use u, we just use x

        # return super().compute_trajectory(sd_start, sd_end)
        # sdd, sd, v, K = self.compute_parameterization(
        #     sd_start, sd_end, return_data=True
        # )

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
