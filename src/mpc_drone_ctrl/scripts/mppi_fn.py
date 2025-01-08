import numpy as np
import jax.numpy as jnp
from jax import jit,vmap,lax
from functools import partial
from jax import random
import jax 


class MPPIControllerForPathTracking():
    def __init__(
            self,
            delta_t: float = 0.05,
            max_omega: float = jnp.pi/4, # [rad]
            max_vel: float = 5.000, # [m/s^2]
            horizon_step_T: int = 30,
            number_of_samples_K: int = 1000,
            param_exploration: float = 0.0,
            param_lambda: float = 50.0,
            param_alpha: float = 1.0,
            sigma = jnp.diag(jnp.array([0.5,0.1])), 
            stage_cost_weight = jnp.array([50.0, 50.0, 50.0, 1.0]), # weight for [x, y, yaw, v]
            terminal_cost_weight = jnp.array([50.0, 50.0, 50.0, 1.0]), # weight for [x, y, yaw, v]
            window_size = 10,
            obstacle_circles: jnp.ndarray = jnp.array([1]),
            collision_safety_margin_rate: float = 0.4
    ) -> None:
        """initialize mppi controller for path-tracking"""
        # mppi parameters
        self.dim_x = 3 # dimension of system state vector
        self.dim_u = 2 # dimension of control input vector
        self.T = horizon_step_T # prediction horizon
        self.K = number_of_samples_K # number of sample trajectories
        self.param_exploration = param_exploration  # constant parameter of mppi
        self.param_lambda = param_lambda  # constant parameter of mppi
        self.param_alpha = param_alpha # constant parameter of mppi
        self.param_gamma = self.param_lambda * (1.0 - (self.param_alpha))  # constant parameter of mppi
        self.Sigma = sigma # deviation of noise
        self.stage_cost_weight = stage_cost_weight
        self.terminal_cost_weight = terminal_cost_weight
        
        # obstacle params
        self.obstacle_circles = obstacle_circles
        self.collision_safety_margin_rate = collision_safety_margin_rate

        # vehicle parameters
        self.delta_t = delta_t #[s]
        self.max_omega = max_omega # [rad]
        self.max_vel = max_vel # [m/s^2]
        self.window_size = window_size

        self.compute_cost_batch = jit(vmap(self.compute_cost, in_axes = ( 0, 0, 0, None, None, None, None )  ))

        self.compute_epsilon_batch = jit(vmap(self.compute_epsilon, in_axes = ( 1, None )  ))

        self.compute_weights_batch = jit(vmap(self._compute_weights, in_axes = ( 0, None, None )  ))

        self.compute_sampled_traj_batch = jit(vmap(self.compute_sampled_traj, in_axes = ( 0, 0 )  ))

        self.moving_average_filter_batch = jit(vmap(self.moving_average_filter, in_axes = ( 1, 1 ), out_axes= (1)  ))



    @partial(jit, static_argnums=(0,))
    def compute_cost(self, x, u, v, ref_x, ref_y, ref_yaw, ref_v):        
        # prepare buffer
        S_init = 0. # state cost list

        def lax_cost(carry,idx):

            S = carry

            # add stage cost
            S = S + self._c(x[idx,:],ref_x[idx],ref_y[idx],ref_yaw[idx],ref_v[idx],u[idx]) + self.param_gamma * u[idx].T @ jnp.linalg.inv(self.Sigma) @ v[idx]

            return (S),(0)

        carry_init = (S_init)
        carry_final,result = jax.lax.scan(lax_cost,carry_init,jnp.arange(self.T-1))
        S = carry_final

        # add terminal cost
        S = S + self._phi(x[self.T-1,:],ref_x[self.T-1],ref_y[self.T-1],ref_yaw[self.T-1],ref_v[self.T-1]) 

        return S

    @partial(jit, static_argnums=(0,))
    def compute_epsilon(self, epsilon, w): 
        w_epsilon_init = jnp.zeros((self.dim_u))

        def lax_eps(carry,idx):

            w_epsilon = carry
            w_epsilon = w_epsilon + w[idx] * epsilon[idx]

            return (w_epsilon),(0)

        carry_init = (w_epsilon_init)
        carry_final,result = jax.lax.scan(lax_eps,carry_init,jnp.arange(self.K))
        w_epsilon = carry_final

        return w_epsilon


    @partial(jit, static_argnums=(0,))
    def _calc_epsilon(self, key):
        """sample epsilon"""
        # check if sigma row size == sigma col size == size_dim_u and size_dim_u > 0
        # if sigma.shape[0] != sigma.shape[1] or sigma.shape[0] != size_dim_u or size_dim_u < 1:
        #     print("[ERROR] sigma must be a square matrix with the size of size_dim_u.")
        #     raise ValueError

        # sample epsilon
        mu = jnp.zeros(self.dim_u) # set average as a zero vector
        epsilon = random.multivariate_normal(key, mu, self.Sigma, (self.K, self.T))
        # print(epsilon[0,0])
        return epsilon


    @partial(jit, static_argnums=(0,))
    def _g(self, v) -> float:
        """clip input"""
        # limit control inputs
        v = v.at[0].set(jnp.clip(v[0], -self.max_omega, self.max_omega)) # limit ang vel
        v = v.at[1].set(jnp.clip(v[1], -self.max_vel, self.max_vel)) # limit vel
        return v

    @partial(jit, static_argnums=(0,))
    def _is_collided(self,  x_t: np.ndarray) -> bool:
        """check if the vehicle is collided with obstacles"""
        """
        # vehicle shape parameters
        vw, vl = self.vehicle_w, self.vehicle_l
        safety_margin_rate = self.collision_safety_margin_rate
        vw, vl = vw*safety_margin_rate, vl*safety_margin_rate

        # get current states
        x, y, yaw, _ = x_t

        # key points for collision check
        vehicle_shape_x = [-0.5*vl, -0.5*vl, 0.0, +0.5*vl, +0.5*vl, +0.5*vl, 0.0, -0.5*vl, -0.5*vl]
        vehicle_shape_y = [0.0, +0.5*vw, +0.5*vw, +0.5*vw, 0.0, -0.5*vw, -0.5*vw, -0.5*vw, 0.0]
        rotated_vehicle_shape_x, rotated_vehicle_shape_y = \
            self._affine_transform(vehicle_shape_x, vehicle_shape_y, yaw, [x, y]) # make the vehicle be at the center of the figure
        """
        safety_margin_rate = self.collision_safety_margin_rate
        x, y, yaw = x_t
        # check if the key points are inside the obstacles
        #for obs in self.obstacle_circles: # for each circular obstacles
        #    obs_x, obs_y, obs_r = obs # [m] x, y, radius
        #    if (x - obs_x)**2 + (y-obs_y)**2 < (obs_r + safety_margin_rate)**2:
        #        return 1.0 # collided
        #return 0.0 # not collided

        if self.obstacle_circles.size == 0:
            return 0.0

        obs_x, obs_y, obs_r = self.obstacle_circles.T
        dist = (x-obs_x)**2 + (y-obs_y)**2
        coll = dist < (obs_r + safety_margin_rate)**2
        res = jnp.where(coll, 1.0, 0.0)
        return jnp.max(res)
    

    @partial(jit, static_argnums=(0,))
    def _c(self, x_t, ref_x, ref_y, ref_yaw, ref_v, u) -> float:
        """calculate stage cost"""
        # parse x_t
        x, y, yaw = x_t
        v = u[1]

        yaw = ((yaw + 2.0*jnp.pi) % (2.0*jnp.pi)) # normalize yaw to [0, 2*pi]

        # calculate stage cost
        stage_cost = self.stage_cost_weight[0]*(x-ref_x)**2 + self.stage_cost_weight[1]*(y-ref_y)**2 + \
                     + self.stage_cost_weight[2]*(yaw-ref_yaw)**2 + self.stage_cost_weight[3]*(v-ref_v)**2
        stage_cost += self._is_collided(x_t)*1.0e4
        return stage_cost


    @partial(jit, static_argnums=(0,))
    def _phi(self, x_T, ref_x, ref_y, ref_yaw, ref_v) -> float:
        """calculate terminal cost"""
        # parse x_T
        x, y, yaw = x_T
        yaw = ((yaw + 2.0*jnp.pi) % (2.0*jnp.pi)) # normalize yaw to [0, 2*pi]

        # calculate terminal cost
        terminal_cost = self.terminal_cost_weight[0]*(x-ref_x)**2 + self.terminal_cost_weight[1]*(y-ref_y)**2 + \
                       + self.terminal_cost_weight[2]*(yaw-ref_yaw)**2
        terminal_cost += self._is_collided(x_T)*1.0e4
        return terminal_cost


    @partial(jit, static_argnums=(0,))
    def _compute_weights(self, S, rho, eta):
        """compute weights for each sample"""

        # calculate weight
        w = (1.0 / eta) * jnp.exp( (-1.0/self.param_lambda) * (S-rho) )

        return w


    @partial(jit, static_argnums=(0,))
    def compute_sampled_traj(self, x, v):
        x_init = x
        sampled_traj_list_init = jnp.zeros((self.T, self.dim_x))
        def lax_st(carry,idx):
            x, sampled_traj_list = carry
            x = self.rk4(x, self._g(v[idx]))
            sampled_traj_list = sampled_traj_list.at[idx].set(x)
            return (x,sampled_traj_list),(0)

        carry_init = (x_init,sampled_traj_list_init)
        carry_final,result = jax.lax.scan(lax_st,carry_init,jnp.arange(self.T))
        x,sampled_traj_list = carry_final

        return sampled_traj_list


    @partial(jit, static_argnums=(0,))
    def moving_average_filter(self, xx_mean, xx):
        """apply moving average filter for smoothing input sequence
        Ref. https://zenn.dev/bluepost/articles/1b7b580ab54e95
        """
        # window_size = 20 # even number only
        b = jnp.ones(self.window_size)/self.window_size
        n_conv = int(self.window_size/2)
        
        xx_mean = jnp.convolve(xx, b, mode="same")
        xx_mean = xx_mean.at[0].set(xx_mean[0] * self.window_size/n_conv)

        xx_mean_init = xx_mean
        def lax_maf(carry,idx):
            xx_mean = carry
            xx_mean = xx_mean.at[idx].set(xx_mean[idx] * self.window_size/(idx+n_conv))
            xx_mean = xx_mean.at[-idx].set(xx_mean[-idx] * self.window_size/(idx + n_conv - (self.window_size % 2)) )
            return (xx_mean),(0)

        carry_init = (xx_mean_init)
        carry_final,result = jax.lax.scan(lax_maf,carry_init,jnp.arange(1,n_conv))
        xx_mean = carry_final

        return xx_mean

    @partial(jit, static_argnums=(0,))
    def f(self,st,con):
        x, y, yaw = st
        omega, v = con

        # model
        new_x = v * jnp.cos(yaw)
        new_y = v * jnp.sin(yaw)
        new_yaw = omega

        # return updated state
        st_new = jnp.array([new_x, new_y, new_yaw])

        return st_new

    @partial(jit, static_argnums=(0,))
    def rk4(self,st,con):
        k1 = self.f(st,con)
        k2 = self.f(st+(self.delta_t*(k1/2)),con)
        k3 = self.f(st+(self.delta_t*(k2/2)),con)
        k4 = self.f(st+(self.delta_t*k3),con)
        y1 = st + ((self.delta_t/6)*(k1+(2*k2)+(2*k3)+k4))
    
        return y1

    @partial(jit, static_argnums=(0,))
    def calc_control_input(self, observed_x, ref_x, ref_y, ref_yaw, ref_v, key, u_prev):
        """calculate optimal control input"""
        # load privious control input sequence
        u = u_prev

        # set initial x value from observation
        x0 = observed_x
    
        # get the waypoint closest to current vehicle position 
        # self._get_nearest_waypoint(x0[0], x0[1])
        # if self.prev_waypoints_idx >= self.ref_path.shape[0]-1:
        #     print("[ERROR] Reached the end of the reference path.")
        #     raise IndexError

        # sample noise
        epsilon = self._calc_epsilon(key) 
        uu = jnp.tile(u,(self.K,1,1))
        x0 = jnp.tile(x0,(self.K,1))

        # control input sequence with noise
        v = uu + epsilon

        sampled_traj_list = self.compute_sampled_traj_batch(x0,v)

        S = self.compute_cost_batch(sampled_traj_list, uu, v, ref_x, ref_y, ref_yaw, ref_v)
        # print(S.shape)
        # exit(0)

        # calculate rho
        rho = S.min()
        # calculate eta
        eta = jnp.sum(jnp.exp( (-1.0/self.param_lambda) * (S-rho) ))
    
        # compute information theoretic weights for each sample
        w = self.compute_weights_batch(S,rho,eta)
        # print(w.shape)
        # exit(0)
        w_epsilon = self.compute_epsilon_batch(epsilon,w)

        # print(w_epsilon)
        # exit(0)

        # apply moving average filter for smoothing input sequence
        xx_mean = jnp.zeros(w_epsilon.shape)
        w_epsilon = self.moving_average_filter_batch(xx_mean,w_epsilon)
        # print(w_epsilon.shape)
        # exit(0)

        # update control input sequence
        u += w_epsilon

        # calculate optimal trajectory
        optimal_traj_init = jnp.zeros((self.T, self.dim_x))
        x_init = observed_x

        def lax_traj(carry,idx):
            x,optimal_traj = carry
            u_star = self._g(u[idx])
            x = self.rk4(x, u_star)
            optimal_traj = optimal_traj.at[idx].set(x)
            return (x,optimal_traj),(u_star)

        carry_init = (x_init,optimal_traj_init)
        carry_final,result = jax.lax.scan(lax_traj,carry_init,jnp.arange(self.T))
        x,optimal_traj = carry_final
        u_star = result
        
        # update privious control input sequence (shift 1 step to the left)
        u_prev = u_prev.at[:-1].set(u_star[1:])
        u_prev = u_prev.at[-1].set(u_star[-1])
        # jax.debug.print("{x}", x=self.u_prev[0])
        # print(u_prev[0])

        # return optimal control input and input sequence
        return u_star[0], u_star, optimal_traj, sampled_traj_list, u_prev

