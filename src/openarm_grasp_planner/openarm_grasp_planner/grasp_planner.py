import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import TransformListener, Buffer
from moveit_msgs.msg import (
    Grasp,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    PlanningScene,
    AllowedCollisionMatrix,
    AllowedCollisionEntry,
)
from moveit_msgs.srv import GraspPlanning
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import copy

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')
        
        # 使用可重入回调组，允许回调嵌套
        self.callback_group = ReentrantCallbackGroup()
        
        # TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建服务 - 使用可重入回调组
        self.grasp_planning_service = self.create_service(
            GraspPlanning, 
            'plan_grasp', 
            self.plan_grasp_callback,
            callback_group=self.callback_group
        )
        
        # 创建 Action 客户端
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # MoveIt Action 客户端
        self.moveit_action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group
        )
        
        # 用于同步的锁和条件变量
        self.moveit_result = None
        self.moveit_result_event = threading.Event()

        # 订阅当前关节状态，用于设置 MoveIt start_state，避免空 JointState 影响规划
        self.joint_state_lock = threading.Lock()
        self.latest_joint_state = None
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('抓取规划模块已启动')

    def plan_grasp_callback(self, request, response):
        """异步处理抓取规划请求"""
        self.get_logger().info('收到抓取规划请求')
        
        # 1. 从TF获取目标物体的位置
        try:
            transform = self.tf_buffer.lookup_transform(
                'openarm_body_link0',
                'banana_target',
                rclpy.time.Time()
            )
            
            target_pose = transform.transform
            self.get_logger().info(f'获取到目标位置: x={target_pose.translation.x:.3f}, y={target_pose.translation.y:.3f}, z={target_pose.translation.z:.3f}')
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'无法获取目标位置: {ex}')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 2. 生成抓取姿态
        grasp = self.generate_grasp_pose(target_pose)
        
        # 3. 异步调用 MoveIt 规划
        self.get_logger().info('开始异步 MoveIt 规划...')
        
        # 重置事件
        self.moveit_result_event.clear()
        self.moveit_result = None
        
        # 异步调用 MoveIt
        future = self.plan_grasp_trajectory_async(grasp)
        
        # 等待规划完成（设置合理的超时时间）
        if not self.moveit_result_event.wait(timeout=60.0):
            self.get_logger().error('MoveIt 规划超时')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.TIMED_OUT
            return response
        
        if self.moveit_result is None:
            self.get_logger().error('MoveIt 规划失败，返回 None')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 4. 执行轨迹
        if self.execute_trajectory(self.moveit_result):
            self.get_logger().info('轨迹执行成功')
        else:
            self.get_logger().error('轨迹执行失败')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 5. 返回响应
        from moveit_msgs.msg import MoveItErrorCodes
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.grasps.append(grasp)
        
        return response

    def plan_grasp_trajectory_async(self, grasp_pose):
        """异步规划抓取轨迹"""
        # 创建 MoveIt Action Goal
        goal_msg = MoveGroup.Goal()
        
        # 设置规划参数
        goal_msg.request.group_name = "left_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        # 若配置中未提供 RRTConnectkConfigDefault，则留空使用默认配置，避免降级混乱
        goal_msg.request.planner_id = ""
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5

        # 显式设置起始状态，使用最新的 joint_states，避免 MoveIt 报空 JointState
        start_state = self.get_current_start_state()
        if start_state is not None:
            goal_msg.request.start_state = start_state
        else:
            self.get_logger().warn('未获取到 joint_states，使用 MoveIt 默认起始状态（可能导致路径异常）')

        # 允许夹爪与香蕉接触（规划阶段允许接触，避免碰撞拒绝）
        goal_msg.planning_options.planning_scene_diff = self.build_acm_for_grasp()
        
        # 创建约束
        constraints = self.create_grasp_constraints(grasp_pose)
        goal_msg.request.goal_constraints = [constraints]
        
        # 发送 Action Goal
        self.get_logger().info('发送 MoveIt Action Goal...')
        
        send_goal_future = self.moveit_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.moveit_feedback_callback
        )
        
        # 设置结果回调
        send_goal_future.add_done_callback(self.moveit_goal_response_callback)
        
        return send_goal_future

    def moveit_goal_response_callback(self, future):
        """处理 MoveIt Goal 响应"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('MoveIt Goal 被拒绝')
                self.moveit_result = None
                self.moveit_result_event.set()
                return
            
            self.get_logger().info('MoveIt Goal 已接受，等待结果...')
            
            # 获取结果
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.moveit_result_callback)
            
        except Exception as e:
            self.get_logger().error(f'MoveIt Goal 响应回调异常: {str(e)}')
            self.moveit_result = None
            self.moveit_result_event.set()

    def moveit_result_callback(self, future):
        """处理 MoveIt 结果"""
        try:
            result_response = future.result()

            if result_response is None or not hasattr(result_response, 'result'):
                self.get_logger().error('MoveIt 结果为空或无 result 字段')
                self.moveit_result = None
                self.moveit_result_event.set()
                return

            goal_result = result_response.result

            from moveit_msgs.msg import MoveItErrorCodes
            if goal_result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f'MoveIt 规划失败，错误码: {goal_result.error_code.val}')
                self.moveit_result = None
            else:
                trajectory = goal_result.planned_trajectory.joint_trajectory
                if len(trajectory.points) == 0:
                    self.get_logger().error('MoveIt 返回空轨迹')
                    self.moveit_result = None
                else:
                    self.get_logger().info(f'MoveIt 规划成功，轨迹点数: {len(trajectory.points)}')
                    self.moveit_result = trajectory
            
        except Exception as e:
            self.get_logger().error(f'MoveIt 结果回调异常: {str(e)}')
            self.moveit_result = None
        finally:
            # 设置事件，通知主线程结果已就绪
            self.moveit_result_event.set()

    def moveit_feedback_callback(self, feedback):
        """处理 MoveIt 反馈"""
        # 可选：记录规划进度
        pass

    def create_grasp_constraints(self, grasp_pose):
        """创建抓取约束"""
        constraints = Constraints()
        
        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'openarm_body_link0'
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = "openarm_left_hand_tcp"
        
        target_pos = grasp_pose.grasp_pose.pose.position
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        box_constraint = SolidPrimitive()
        box_constraint.type = SolidPrimitive.BOX
        box_constraint.dimensions = [0.02, 0.02, 0.02]
        region_pose = Pose()
        region_pose.position.x = target_pos.x
        region_pose.position.y = target_pos.y
        region_pose.position.z = target_pos.z
        position_constraint.constraint_region.primitives = [box_constraint]
        position_constraint.constraint_region.primitive_poses = [region_pose]
        position_constraint.weight = 1.0
        
        # 姿态约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'openarm_body_link0'
        orientation_constraint.header.stamp = self.get_clock().now().to_msg()
        orientation_constraint.link_name = "openarm_left_hand_tcp"
        orientation_constraint.orientation = grasp_pose.grasp_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2
        orientation_constraint.absolute_y_axis_tolerance = 0.2
        orientation_constraint.absolute_z_axis_tolerance = 0.2
        orientation_constraint.weight = 1.0
        
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]
        
        return constraints

    def build_acm_for_grasp(self):
        """构造允许夹爪与抓取物体接触的 ACM，避免规划阶段因接触判定失败"""
        acm = AllowedCollisionMatrix()
        links = ["banana", "openarm_left_left_finger", "openarm_left_right_finger"]
        acm.entry_names = links

        # 构造对称的允许矩阵：香蕉与两个手指互相允许，手指之间也允许
        for i, name in enumerate(links):
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * len(links)
            for j in range(len(links)):
                if i != j:
                    entry.enabled[j] = True
            acm.entry_values.append(entry)
        planning_scene = PlanningScene()
        planning_scene.allowed_collision_matrix = acm
        return planning_scene

    def generate_grasp_pose(self, target_pose):
        """生成抓取姿态"""
        grasp = Grasp()
        
        grasp.grasp_pose.header.frame_id = 'openarm_body_link0'
        grasp.grasp_pose.pose.position.x = target_pose.translation.x
        grasp.grasp_pose.pose.position.y = target_pose.translation.y
        grasp.grasp_pose.pose.position.z = target_pose.translation.z
        
        grasp.grasp_pose.pose.orientation.x = 1.0
        grasp.grasp_pose.pose.orientation.y = 0.0
        grasp.grasp_pose.pose.orientation.z = 0.0
        grasp.grasp_pose.pose.orientation.w = 0.0
        
        grasp.pre_grasp_approach.direction.vector.z = -1.0
        grasp.pre_grasp_approach.min_distance = 0.01
        grasp.pre_grasp_approach.desired_distance = 0.05
        
        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.01
        grasp.post_grasp_retreat.desired_distance = 0.05
        
        return grasp

    def joint_state_callback(self, msg: JointState):
        """缓存最新的关节状态供 start_state 使用"""
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def get_current_start_state(self):
        """将最新 joint_states 填入 MoveIt start_state"""
        with self.joint_state_lock:
            if self.latest_joint_state is None:
                return None
            js_copy = copy.deepcopy(self.latest_joint_state)
        from moveit_msgs.msg import RobotState
        rs = RobotState()
        rs.joint_state = js_copy
        return rs

    def execute_trajectory(self, trajectory):
        """执行轨迹"""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('轨迹控制器未就绪')
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('发送轨迹执行请求...')
        
        # 使用事件等待执行完成
        execution_event = threading.Event()
        execution_result = [None]  # 使用列表存储结果
        
        def execution_response_callback(future):
            try:
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    self.get_logger().info('轨迹执行已接受')
                    
                    # 等待执行完成
                    result_future = goal_handle.get_result_async()
                    
                    def execution_result_callback(future):
                        try:
                            result = future.result()
                            execution_result[0] = result
                            execution_event.set()
                        except Exception as e:
                            self.get_logger().error(f'轨迹执行结果异常: {str(e)}')
                            execution_event.set()
                    
                    result_future.add_done_callback(execution_result_callback)
                else:
                    self.get_logger().warn('轨迹执行被拒绝')
                    execution_event.set()
            except Exception as e:
                self.get_logger().error(f'轨迹执行响应异常: {str(e)}')
                execution_event.set()
        
        future = self.trajectory_client.send_goal_async(goal_msg)
        future.add_done_callback(execution_response_callback)
        
        # 等待执行完成（超时时间根据轨迹长度设置）
        if execution_event.wait(timeout=60.0):
            return execution_result[0] is not None
        else:
            self.get_logger().error('轨迹执行超时')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = GraspPlanner()
    
    # 使用多线程执行器，增加线程数
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()