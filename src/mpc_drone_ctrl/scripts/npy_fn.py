import numpy as np

class Mppi_npy():
    def __init__(self,ref_path,T):
        self.ref_path = ref_path
        # ref_path info
        self.prev_waypoints_idx = 0
        self.T = T

    def get_nearest_waypoint(self, x, y):
        """search the closest waypoint to the vehicle on the reference path"""

        SEARCH_IDX_LEN = 200 # [points] forward search range
        prev_idx = self.prev_waypoints_idx
        dx = x - self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 0]
        dy = y - self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 1]
        d = dx**2 + dy**2
        nearest_idx = np.argmin(d) + prev_idx

        # get reference values of the nearest waypoint
        ref_x = self.ref_path[nearest_idx:(nearest_idx + self.T),0]
        ref_y = self.ref_path[nearest_idx:(nearest_idx + self.T),1]
        ref_yaw = self.ref_path[nearest_idx:(nearest_idx + self.T),2]
        ref_v = self.ref_path[nearest_idx:(nearest_idx + self.T),3]

        # update nearest waypoint index
        self.prev_waypoints_idx = nearest_idx
 
        return ref_x, ref_y, ref_yaw, ref_v

    