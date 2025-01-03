

class potential_field:
    def __init__(self):
        rospy.init_node("potential_field")

        self.sample_rate = rospy.get_param("~sample_rate", 10)

        # Subscribe to the global planner using the move base package. The global plan is the path that the robot would ideally follow if 
        # there are no unknown/dynamic obstacles. In the videos this is highlighted by green color.
        self.global_path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.handle_global_path)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.handle_laser)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.handle_odom)
        
        # Subscribe to the goal topic to get the goal position given using rviz's 2D Navigation Goal option.
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.handle_goal)

        # Publish the potential field vector topic which will be subscribed by the command_velocity node in order to
        # compute velocities.
        self.potential_field_pub = rospy.Publisher("potential_field_vector", Point,queue_size=10)

        # We store the path data gotten from the global planner above and display it. We have written a custom publisher 
        # in order to get more flexibility while displaying the paths.
        self.global_path_pub = rospy.Publisher("global_path",Path,queue_size=10)

        # This is a publisher to publish the robot path. In the videos it is highlighted by red color.
        self.robot_path_pub = rospy.Publisher("robot_path",Path,queue_size=10)
        
        self.path_robot = Path()
        self.path_robot.header.frame_id = 'map'

        ## TODO Choose suitable values
        self.eta = 0.5 # scaling factor for repulsive force
        self.zeta = 0.5 # scaling factor for attractive force
        self.q_star = 3 # threshold distance for obstacles
        self.d_star = 1.5 # threshoild distance for goal

        self.laser = None
        self.odom = None
        self.goal = None

        self.path_data = Path()
        self.path_data.header.frame_id = 'map'
        
        self.position_x = []
        self.position_y = []
        self.position_all = []
        
        # Boolean variables used for proper display of robot path and global path
        self.bool_goal = False
        self.bool_path = False

        # this determines the iterations the robot tries out a random move to escape local minimi
        self.random_nudge_iters = 8
        self.random_nudge_counter = 0
        self.noise = np.zeros(2)
        self.movement_history = np.zeros((self.random_nudge_iters, 2))
        self.average_movement = 0
        # list containing the last tried random moves and how much average movement they resulted in 
        # the robot will tend its random movements toward those that seem to bring it out of the local minimum more
        self.average_movement_random_motion = np.zeros((self.random_nudge_iters, 3))
        self.cooldown = 10

def emergency_land(): 
    # TODO

    pass

def takeoff():
    # TODO
    pass

