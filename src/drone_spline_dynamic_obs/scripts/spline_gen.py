import numpy as np
from scipy.interpolate import CubicSpline

def path_spline(x_path: np.ndarray, y_path: np.ndarray) -> tuple: # x(s), y(s), x'(s), y'(s), 
    x_diff = np.diff(x_path)
    y_diff = np.diff(y_path)
    phi = np.unwrap(np.arctan2(y_diff, x_diff))
    phi_init = phi[0]
    phi = np.hstack(( phi_init, phi  ))
    arc = np.cumsum( np.sqrt( x_diff**2+y_diff**2 )   )
    arc_length = arc[-1]
    arc_vec = np.linspace(0, arc_length, np.shape(x_path)[0])
    cs_x_path = CubicSpline(arc_vec, x_path)
    cs_y_path = CubicSpline(arc_vec, y_path)
    cs_phi_path = CubicSpline(arc_vec, phi)
    x_d_dot_path = cs_x_path.derivative()
    y_d_dot_path = cs_y_path.derivative()
    return cs_x_path, cs_y_path, x_d_dot_path, y_d_dot_path, cs_phi_path, arc_length, arc_vec

def waypoint_generator(x_global_init, y_global_init, x_path_data, y_path_data, arc_vec, cs_x_path, cs_y_path, cs_phi_path, arc_length, cs_xdot_path, cs_ydot_path, v, dt) -> tuple:
    idx = np.argmin( np.sqrt((x_global_init-x_path_data)**2+(y_global_init-y_path_data)**2))
    arc_curr = arc_vec[idx] # s_0
    arc_pred = arc_curr + v*dt #s_0+sdot*delta_t from slides
    x_waypoints = cs_x_path(arc_pred) # x_d(s_0+sdot*delta*t)
    y_waypoints =  cs_y_path(arc_pred) # y_d(s_0+sdot*delta*t)
    phi_Waypoints = cs_phi_path(arc_pred)
    x_d = cs_x_path(arc_pred)
    y_d = cs_y_path(arc_pred)
    xd_dot = cs_xdot_path(arc_pred)*v
    yd_dot = cs_ydot_path(arc_pred)*v
    return x_waypoints, y_waypoints, phi_Waypoints, xd_dot, yd_dot, x_d, y_d