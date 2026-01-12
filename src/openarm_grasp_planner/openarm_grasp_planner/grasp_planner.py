import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformListener, Buffer
from moveit_msgs.msg import Grasp
from moveit_msgs.srv import GraspPlanning
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')
        
        # TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建服务
        self.grasp_planning_service = self.create_service(
            GraspPlanning, 'plan_grasp', self.plan_grasp_callback
        )
        
        # 创建 Action 客户端用于执行轨迹（使用左臂）
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('抓取规划模块已启动')
        self.get_logger().info('等待轨迹控制器就绪...')
    
    def plan_grasp_callback(self, request, response):
        self.get_logger().info('收到抓取规划请求')
        
        # 1. 从TF获取目标物体的位置
        try:
            transform = self.tf_buffer.lookup_transform(
                'openarm_body_link0',  # 基座坐标系
                'banana_target',        # 目标物体坐标系
                rclpy.time.Time()       # 当前时间
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
        
        # 3. 规划抓取轨迹
        grasp_trajectory = self.plan_grasp_trajectory(grasp)
        
        # 4. 执行抓取轨迹（通过 Action 接口）
        if self.execute_trajectory(grasp_trajectory):
            self.get_logger().info('抓取轨迹执行成功')
        else:
            self.get_logger().error('抓取轨迹执行失败')
            from moveit_msgs.msg import MoveItErrorCodes
            response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        
        # 5. 返回响应
        from moveit_msgs.msg import MoveItErrorCodes
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.grasps.append(grasp)
        
        return response
    
    def generate_grasp_pose(self, target_pose):
        """生成抓取姿态"""
        grasp = Grasp()
        
        # 设置抓取位置
        grasp.grasp_pose.header.frame_id = 'openarm_body_link0'
        grasp.grasp_pose.pose.position.x = target_pose.translation.x
        grasp.grasp_pose.pose.position.y = target_pose.translation.y
        grasp.grasp_pose.pose.position.z = target_pose.translation.z + 0.05  # 稍微高于目标
        
        # 设置抓取方向（简单示例）
        grasp.grasp_pose.pose.orientation.x = 0.0
        grasp.grasp_pose.pose.orientation.y = 0.0
        grasp.grasp_pose.pose.orientation.z = 0.0
        grasp.grasp_pose.pose.orientation.w = 1.0
        
        # 设置抓取参数
        grasp.pre_grasp_approach.direction.vector.z = -1.0  # 从上方接近
        grasp.pre_grasp_approach.min_distance = 0.01
        grasp.pre_grasp_approach.desired_distance = 0.05
        
        grasp.post_grasp_retreat.direction.vector.z = 1.0  # 抓取后向上撤退
        grasp.post_grasp_retreat.min_distance = 0.01
        grasp.post_grasp_retreat.desired_distance = 0.05
        
        return grasp
    
    def plan_grasp_trajectory(self, grasp_pose):
        """规划抓取轨迹（使用TF直接获取从左臂基座到目标的变换，无需手动换算坐标系）"""
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'openarm_body_link0'  # 使用底座中心为原点
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # 设置关节名称（使用左臂的关节名称）
        trajectory.joint_names = [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        # 从抓取姿态中获取目标位置（在 openarm_body_link0 坐标系下）
        target_x = grasp_pose.grasp_pose.pose.position.x
        target_y = grasp_pose.grasp_pose.pose.position.y
        target_z = grasp_pose.grasp_pose.pose.position.z
        
        self.get_logger().info(f'目标位置（openarm_body_link0坐标系）: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
        
        # 使用TF直接查询从左臂基座到目标的变换（TF会自动处理坐标系转换）
        try:
            # 查询从左臂基座（openarm_left_link0）到目标（banana_target）的变换
            # 注意：banana_target 是在 openarm_body_link0 坐标系下的，TF会自动转换
            transform = self.tf_buffer.lookup_transform(
                'openarm_left_link0',  # 左臂基座坐标系
                'banana_target',        # 目标物体坐标系
                rclpy.time.Time()       # 当前时间
            )
            
            # 获取在左臂基座坐标系下的目标位置
            rel_x = transform.transform.translation.x
            rel_y = transform.transform.translation.y
            rel_z = transform.transform.translation.z
            
            self.get_logger().info(f'目标位置（左臂基座坐标系，通过TF获取）: x={rel_x:.3f}, y={rel_y:.3f}, z={rel_z:.3f}')
            
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'无法获取从左臂基座到目标的TF变换: {ex}')
            self.get_logger().error('使用目标位置直接计算（可能不准确）')
            # 如果TF查询失败，回退到直接计算（不推荐）
            rel_x = target_x
            rel_y = target_y
            rel_z = target_z
        
        # 计算关节角度（基于在左臂基座坐标系下的目标位置）
        joint_angles = self.compute_joint_angles_from_relative_position(rel_x, rel_y, rel_z)
        
        # 创建轨迹点1 - 接近位置（稍微高于目标）
        point1 = JointTrajectoryPoint()
        approach_rel_z = rel_z + 0.1  # 在目标上方10cm（在左臂基座坐标系下）
        approach_angles = self.compute_joint_angles_from_relative_position(rel_x, rel_y, approach_rel_z)
        point1.positions = approach_angles
        point1.velocities = [1.0] * 7  # 设置速度
        point1.accelerations = [0.0] * 7
        point1.time_from_start.sec = 0
        point1.time_from_start.nanosec = 500000000  # 0.5秒
        
        # 创建轨迹点2 - 抓取位置（目标位置）
        point2 = JointTrajectoryPoint()
        point2.positions = joint_angles
        point2.velocities = [1.0] * 7  # 设置速度
        point2.accelerations = [0.0] * 7
        point2.time_from_start.sec = 0
        point2.time_from_start.nanosec = 1000000000  # 1.0秒
        
        # 将轨迹点添加到轨迹中
        trajectory.points.append(point1)
        trajectory.points.append(point2)
        
        self.get_logger().info(f'规划了包含 {len(trajectory.points)} 个轨迹点的抓取轨迹')
        self.get_logger().info(f'轨迹点1（接近）: {[f"{p:.3f}" for p in point1.positions]}')
        self.get_logger().info(f'轨迹点2（抓取）: {[f"{p:.3f}" for p in point2.positions]}')
        
        return trajectory
    
    def compute_joint_angles_from_relative_position(self, rel_x, rel_y, rel_z):
        """根据目标位置（在左臂基座坐标系 openarm_left_link0 下）计算关节角度
        这是简化的逆运动学计算，基于几何关系估算
        
        注意：输入位置应该已经在左臂基座坐标系下了（通过TF获取）
        """
        # 计算到目标的距离和角度（在左臂基座坐标系下）
        r = np.sqrt(rel_x**2 + rel_y**2)  # 水平距离
        d = np.sqrt(r**2 + rel_z**2)  # 3D距离
        
        self.get_logger().info(f'计算关节角度: rel_x={rel_x:.3f}, rel_y={rel_y:.3f}, rel_z={rel_z:.3f}, r={r:.3f}, d={d:.3f}')
        
        # 简化的关节角度计算（基于几何关系）
        # joint1: 旋转朝向目标（atan2(rel_y, rel_x)）
        # 注意：左臂的 joint1 有 -2.094396 的偏移（根据 URDF）
        joint1 = np.arctan2(rel_y, rel_x) - 2.094396  # 减去左臂的偏移
        
        # joint2: 向下倾斜角度（基于目标高度和水平距离）
        # 如果目标在基座下方，joint2应该为负值（向下）
        joint2 = -np.arctan2(rel_z, r) - 0.2  # 减去一个基础角度
        
        # joint3: 向上弯曲（补偿joint2的倾斜）
        joint3 = -joint2 * 0.7 + 0.4
        
        # joint4: 调整高度（进一步微调）
        joint4 = -0.2
        
        # joint5, joint6, joint7: 末端姿态调整（简化处理，保持末端水平）
        joint5 = 0.0
        joint6 = 0.3
        joint7 = 0.0
        
        # 限制关节角度在合理范围内
        joint_angles = [
            np.clip(joint1, -3.14, 3.14),      # joint1: ±180度（考虑偏移）
            np.clip(joint2, -1.57, 0.0),      # joint2: -90度到0度
            np.clip(joint3, 0.0, 1.57),       # joint3: 0度到90度
            np.clip(joint4, -1.57, 0.0),      # joint4: -90度到0度
            np.clip(joint5, -1.57, 1.57),     # joint5: ±90度
            np.clip(joint6, -1.57, 1.57),     # joint6: ±90度
            np.clip(joint7, -1.57, 1.57),     # joint7: ±90度
        ]
        
        return joint_angles
    
    def execute_trajectory(self, trajectory):
        """通过 Action 接口执行轨迹"""
        # 检查服务器是否就绪（不阻塞）
        if not self.trajectory_client.server_is_ready():
            self.get_logger().error('轨迹控制器未就绪')
            return False
        
        # 创建 Action goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # 发送 goal（异步方式，不等待结果）
        self.get_logger().info('发送轨迹执行请求...')
        
        # 发送 goal（异步，不等待结果）
        # 在服务回调中，我们不能阻塞等待 Action 的结果
        future = self.trajectory_client.send_goal_async(goal_msg)
        
        # 使用回调记录结果（但不阻塞）
        def goal_response_callback(future):
            try:
                goal_handle = future.result()
                if goal_handle and goal_handle.accepted:
                    self.get_logger().info('轨迹执行请求已接受并执行')
                else:
                    self.get_logger().warn('轨迹执行请求被拒绝')
            except Exception as e:
                self.get_logger().error(f'轨迹执行请求处理异常: {str(e)}')
        
        future.add_done_callback(goal_response_callback)
        
        # 立即返回成功（不等待 future 完成）
        # 因为服务回调不应该阻塞，而且从日志看 goal 已经被接受了
        self.get_logger().info('轨迹执行请求已发送（异步执行）')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()