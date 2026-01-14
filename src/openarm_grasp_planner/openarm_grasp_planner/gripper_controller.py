import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import GripperCommand
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import threading

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')
        
        # 夹爪状态参数（根据 SRDF 定义：0.0=闭合，0.060=更大打开）
        # 注意：位置值表示手指分开的距离，0.0 表示闭合，0.060 表示完全打开
        self.gripper_open_position = 0.060  # 夹爪打开位置（手指分开）
        self.gripper_closed_position = 0.0  # 夹爪完全闭合位置（手指靠拢）
        # 夹爪抓取位置：不完全关闭，只关闭到能夹住香蕉的程度（约0.02-0.03米）
        # 香蕉直径约为0.04米，所以关闭到0.03米可以夹住香蕉
        self.gripper_grasp_position = 0.045  # 夹爪抓取位置（能夹住香蕉但不完全关闭）
        self.gripper_max_effort = 50.0  # 最大抓取力（增加以更好地夹住香蕉）
        
        # 分步夹紧参数
        self.grasp_steps = 4  # 分3步夹紧
        self.grasp_step_delay = 1.5 # 每步之间的延迟（秒），给物理仿真时间稳定
        self.grasp_in_progress = False  # 标记是否正在执行分步夹紧
        self.grasp_step_lock = threading.Lock()  # 保护分步夹紧状态
        
        # 创建Action客户端用于控制夹爪（双机械臂版本）
        self.left_gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/left_gripper_controller/gripper_cmd'  # 使用正确的主题名称和命名空间
        )
        
        self.right_gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/right_gripper_controller/gripper_cmd'  # 使用正确的主题名称和命名空间
        )
        
        # 创建服务用于控制夹爪开合，使用可重入回调组以支持异步操作
        self.callback_group = ReentrantCallbackGroup()
        self.gripper_service = self.create_service(
            SetBool, 
            'control_gripper', 
            self.control_gripper_callback,
            callback_group=self.callback_group
        )
        
        # 订阅关节状态获取夹爪当前位置
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        self.current_left_gripper_position = None
        self.current_right_gripper_position = None
        self.get_logger().info('夹爪控制模块已启动（双机械臂版本）')
        self.get_logger().info(f'左夹爪 Action 话题: /left_gripper_controller/gripper_cmd')
        self.get_logger().info(f'右夹爪 Action 话题: /right_gripper_controller/gripper_cmd')
        self.get_logger().info('等待夹爪控制器启动...')
    
    def joint_state_callback(self, msg):
        """获取夹爪当前位置"""
        try:
            # 获取左夹爪位置
            index = msg.name.index('openarm_left_finger_joint1')
            self.current_left_gripper_position = msg.position[index]
        except ValueError:
            pass
            
        try:
            # 获取右夹爪位置
            index = msg.name.index('openarm_right_finger_joint1')
            self.current_right_gripper_position = msg.position[index]
        except ValueError:
            pass
    
    def control_gripper_callback(self, request, response):
        """处理夹爪控制服务请求（只控制左夹爪）"""
        action = "抓取" if request.data else "打开"
        self.get_logger().info(f'收到夹爪控制请求: {action}')
        
        if request.data:
            # 抓取：分三步夹紧，避免冲击力导致香蕉掉落
            with self.grasp_step_lock:
                if self.grasp_in_progress:
                    self.get_logger().warn('分步夹紧正在进行中，忽略重复请求')
                    response.success = False
                    response.message = '分步夹紧正在进行中'
                    return response
                self.grasp_in_progress = True
            
            # 在后台线程中执行分步夹紧，避免阻塞服务回调
            threading.Thread(
                target=self.grasp_gradually_async,
                args=(self.gripper_grasp_position,),
                daemon=True
            ).start()
            
            response.success = True
            response.message = '分步夹紧已启动'
            self.get_logger().info('分步夹紧已启动（异步执行）')
        else:
            # 打开：直接打开到完全打开位置
            with self.grasp_step_lock:
                self.grasp_in_progress = False  # 取消分步夹紧
            
            left_result = self.send_gripper_command(self.left_gripper_client, self.gripper_open_position, "左")
            if left_result:
                response.success = True
                response.message = '左夹爪已打开'
                self.get_logger().info('左夹爪已打开')
            else:
                response.success = False
                response.message = '左夹爪打开失败'
                self.get_logger().error('左夹爪打开失败')
        
        return response
    
    def grasp_gradually_async(self, target_position):
        """异步分步夹紧，在后台线程中执行"""
        try:
            # 获取当前夹爪位置
            current_position = self.current_left_gripper_position
            if current_position is None:
                # 如果不知道当前位置，假设从完全打开位置开始
                current_position = self.gripper_open_position
                self.get_logger().warn('无法获取左夹爪当前位置，假设从完全打开位置开始')
            
            # 计算每步的位置增量
            position_diff = current_position - target_position
            step_size = position_diff / self.grasp_steps
            
            self.get_logger().info(
                f'开始分步夹紧: 从 {current_position:.4f} 到 {target_position:.4f}, '
                f'分 {self.grasp_steps} 步，每步 {step_size:.4f}'
            )
            
            # 分步夹紧
            for step in range(1, self.grasp_steps + 1):
                # 检查是否被取消
                with self.grasp_step_lock:
                    if not self.grasp_in_progress:
                        self.get_logger().info('分步夹紧被取消')
                        return
                
                step_position = current_position - (step_size * step)
                self.get_logger().info(f'步骤 {step}/{self.grasp_steps}: 夹紧到位置 {step_position:.4f}')
                
                # 发送单步命令（异步，不等待完成）
                self.send_gripper_command(
                    self.left_gripper_client, 
                    step_position, 
                    "左"
                )
                
                # 等待一段时间再执行下一步（除了最后一步）
                if step < self.grasp_steps:
                    import time
                    time.sleep(self.grasp_step_delay)
            
            self.get_logger().info(f'分步夹紧完成，最终位置: {target_position:.4f}')
            
        except Exception as e:
            self.get_logger().error(f'分步夹紧过程异常: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            with self.grasp_step_lock:
                self.grasp_in_progress = False
    
    def send_gripper_command(self, client, position, arm_side):
        """发送夹爪控制命令（只控制左夹爪）"""
        # 检查Action服务器是否就绪（不阻塞）
        if not client.server_is_ready():
            self.get_logger().error(f'{arm_side}夹爪控制服务器未就绪')
            self.get_logger().error(f'请确保 MuJoCo 和控制器已启动，Action 话题: {client._action_name}')
            return False
        
        self.get_logger().info(f'{arm_side}夹爪控制服务器已就绪')
        
        # 创建Action目标
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = self.gripper_max_effort
        
        # 发送请求（异步方式，不等待结果）
        self.get_logger().info(f'发送{arm_side}夹爪控制命令: 位置={position}, 最大力={self.gripper_max_effort}')
        try:
            # 发送 goal（异步，不等待结果）
            # 在服务回调中，我们不能阻塞等待 Action 的结果
            # 只要服务器就绪，就发送命令并假设成功
            future = client.send_goal_async(goal_msg)
        
            # 使用回调记录结果（但不阻塞）
            def goal_response_callback(future):
                try:
                    goal_handle = future.result()
                    if goal_handle and goal_handle.accepted:
                        self.get_logger().info(f'{arm_side}夹爪控制命令已被控制器接受，开始执行...')

                        # 在执行完成后再获取一次结果（完全异步，不阻塞 service 回调）
                        result_future = goal_handle.get_result_async()

                        def result_callback(result_future):
                            try:
                                result = result_future.result().result
                                # Action 返回的最终位置
                                self.get_logger().info(
                                    f'{arm_side}夹爪动作执行完成，Action 返回位置: {result.position:.4f}'
                                )

                                # 结合 /joint_states 中记录的当前位置再打印一次（如果可用）
                                if arm_side == "左" and self.current_left_gripper_position is not None:
                                    self.get_logger().info(
                                        f'执行完成后当前左夹爪 /joint_states 位置: {self.current_left_gripper_position:.4f}'
                                    )
                            except Exception as e:
                                self.get_logger().error(f'{arm_side}夹爪结果回调处理异常: {str(e)}')

                        result_future.add_done_callback(result_callback)
                    else:
                        self.get_logger().warn(f'{arm_side}夹爪控制命令被拒绝')
                except Exception as e:
                    self.get_logger().error(f'{arm_side}夹爪控制命令处理异常: {str(e)}')
            
            future.add_done_callback(goal_response_callback)
            
            # 立即返回成功（不等待 future 完成）
            # 因为服务回调不应该阻塞，而且从日志看 goal 已经被接受了
            self.get_logger().info(f'{arm_side}夹爪控制命令已发送（异步执行）')
            return True
            
        except Exception as e:
            self.get_logger().error(f'发送{arm_side}夹爪控制命令时发生异常: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()