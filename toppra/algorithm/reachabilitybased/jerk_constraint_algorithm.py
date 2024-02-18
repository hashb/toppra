from toppra.interpolator import AbstractGeometricPath
from .time_optimal_algorithm import TOPPRA
import logging
import numpy as np

logger = logging.getLogger(__name__)


class JerkLimitedTOPPRA(TOPPRA):

    def compute_trajectory(
        self, sd_start: float = 0, sd_end: float = 0
    ) -> AbstractGeometricPath | None:
        # return super().compute_trajectory(sd_start, sd_end)
        L = self.compute_reachable_sets(sd_start, sd_start)

        # check if path is reachable

        # samapling to find x and u that satisfy the constraints
