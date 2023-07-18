import rclpy
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy)
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from tier4_external_api_msgs.srv import Engage
from tier4_system_msgs.srv import OperateMrm
from tier4_external_api_msgs.srv import SetEmergency


class VehicleCommander(Node):
    '''
    Class used to send commands to the vehicle.

    Args:
        Node (rclpy.node.Node): Node class from rclpy
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        if "vehicle1" in node_name:
            self._vehicle_name = "vehicle1"
        elif "vehicle2" in node_name:
            self._vehicle_name = "vehicle2"
        else:
            self.get_logger().error("Node name must contain either vehicle1 or vehicle2")

        self._engage_client = self.create_client(Engage, '/api/autoware/set/engage')

        self._publish_goal_counter = 0

        # Get parameters for the initial position and orientation
        self._init_pos_x = self.declare_parameter('init_pos_x').get_parameter_value().double_value
        self._init_pos_y = self.declare_parameter('init_pos_y').get_parameter_value().double_value
        self._init_pos_z = self.declare_parameter('init_pos_z').get_parameter_value().double_value
        self._init_rot_x = self.declare_parameter('init_rot_x').get_parameter_value().double_value
        self._init_rot_y = self.declare_parameter('init_rot_y').get_parameter_value().double_value
        self._init_rot_z = self.declare_parameter('init_rot_z').get_parameter_value().double_value
        self._init_rot_w = self.declare_parameter('init_rot_w').get_parameter_value().double_value

        # # Get parameters for the goal position and orientation
        self._goal_pos_x = self.declare_parameter('goal_pos_x').get_parameter_value().double_value
        self._goal_pos_y = self.declare_parameter('goal_pos_y').get_parameter_value().double_value
        self._goal_pos_z = self.declare_parameter('goal_pos_z').get_parameter_value().double_value
        self._goal_rot_x = self.declare_parameter('goal_rot_x').get_parameter_value().double_value
        self._goal_rot_y = self.declare_parameter('goal_rot_y').get_parameter_value().double_value
        self._goal_rot_z = self.declare_parameter('goal_rot_z').get_parameter_value().double_value
        self._goal_rot_w = self.declare_parameter('goal_rot_w').get_parameter_value().double_value

        # Callback groups
        callback_group1 = MutuallyExclusiveCallbackGroup()
        callback_group2 = MutuallyExclusiveCallbackGroup()
        # client_group = MutuallyExclusiveCallbackGroup()

        # Subscribe to the odom topic

        self.gnss_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/sensing/gnss/pose',
            self.gnss_pose_cb,
            10,
            callback_group=callback_group1
        )

        # Create a publisher for the PoseWithCovarianceStamped messages
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        self.publisher_initial_pose = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic='/initialpose',
            qos_profile=qos_profile)

        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            depth=5)

        # Create a publisher for the PoseStamped messages
        self.publisher_goal_pose = self.create_publisher(
            msg_type=PoseStamped,
            topic='/planning/mission_planning/goal',
            qos_profile=qos_profile)


        # # Create a publisher for the planned trajectory
        # self.trajectory_publisher = self.create_publisher(
        #     Trajectory,
        #     '/planned_trajectory',
        #     qos_profile=qos_profile
        # )

        # # Create a publisher for the vehicle control commands
        # self.control_publisher = self.create_publisher(
        #     VehicleControlCommand,
        #     '/vehicle_control_cmd',
        #     10
        # )

        self._timer = self.create_timer(2, self.run, callback_group=callback_group2)
        self._vehicle_initialized = False
        self._vehicle_goal_set = False
        self._vehicle_engaged = False

    def gnss_pose_cb(self, msg: PoseStamped):
        '''
        Callback function for the /sensing/gnss/pose topic
        This function is used to get the current position of the vehicle

        Args:
            msg (PoseStamped): Pose message received from the /sensing/gnss/pose topic
        '''

        current_position_x = msg.pose.position.x
        current_position_y = msg.pose.position.y

        # compute the distance between the current position and the goal position
        distance = ((current_position_x - self._goal_pos_x)**2 + (current_position_y - self._goal_pos_y)**2)**0.5

        # if current position is 10 meters away from the goal position, then stop the vehicle
        if distance < 10:
            self.emergency_break()
            # self.engage(False)
            # self._vehicle_engaged = False
            # self._vehicle_goal_set = False
            # self._vehicle_initialized = False
            # self._current_position

    def emergency_break(self):
        '''
        Emergency break
        '''
        client = self.create_client(SetEmergency, '/api/autoware/set/emergency')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Engage service not available, waiting again...')

        request = SetEmergency.Request()
        request.emergency = True

        future = client.call_async(request)
        future.add_done_callback(self.emergency_break_cb)

    def emergency_break_cb(self, future):
        '''
        Callback function for the emergency break service
        '''
        self.get_logger().info(f'Emergency break status: {future.result().status.code}')
        self.get_logger().info(f'Emergency break message: {future.result().status.message}')

    def engage(self, engage):
        '''
        Engage to drive towards the goal

        Args:
            engage (bool): Whether to engage or not
        '''

        while not self._engage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Engage service not available, waiting again...')

        request = Engage.Request()
        request.engage = engage

        future = self._engage_client.call_async(request)
        # future.add_done_callback(self.engage_cb)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Engage service call successful')
        else:
            self.get_logger().info('Engage service call failed')

    def engage_cb(self, future):
        '''
        Callback function for the engage service
        '''
        self.get_logger().info(f'Engage code: {future.result().response.code}')
        self.get_logger().info(f'Engage message: {future.result().response.message}')

    def run(self):
        '''
        Main loop of the node
        '''
        # self.get_logger().info(f'Running {self._vehicle_name}')
        if not self._vehicle_initialized:
            self.initialize()
            self._vehicle_initialized = True

        if not self._vehicle_goal_set and self._vehicle_initialized:
            self.set_goal()
            self._vehicle_goal_set = True

        if not self._vehicle_engaged and self._vehicle_goal_set:
            self.engage(True)
            self._vehicle_engaged = True

    def set_goal(self):
        '''
        Publish a PoseStamped message to the /planning/mission_planning/goal topic
        '''
        # Create a PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 86.395
        pose.pose.position.y = -300.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = -0.7072
        pose.pose.orientation.w = 0.7072

        # Publish the PoseStamped message
        self.publisher_goal_pose.publish(pose)
        self.get_logger().info(f'Published mission plan for {self._vehicle_name}')

    def initialize(self):
        '''
        Initialize the ego vehicle's position in the map
        '''

        # Create a PoseWithCovarianceStamped message
        pose = PoseWithCovarianceStamped()

        # Set the pose values
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self._init_pos_x
        pose.pose.pose.position.y = self._init_pos_y
        pose.pose.pose.position.z = self._init_pos_z
        pose.pose.pose.orientation.x = self._init_rot_x
        pose.pose.pose.orientation.y = self._init_rot_y
        pose.pose.pose.orientation.z = self._init_rot_z
        pose.pose.pose.orientation.w = self._init_rot_w

        # Set the covariance values
        pose.pose.covariance = [0.0] * 36  # Fill covariance matrix with zeros
        pose.pose.covariance[0] = 0.25  # x
        pose.pose.covariance[7] = 0.25  # y
        pose.pose.covariance[35] = 0.06853891945200942  # yaw
        # Publish the PoseWithCovarianceStamped message
        self.publisher_initial_pose.publish(pose)
        self.get_logger().info(f'Initialized {self._vehicle_name}')
