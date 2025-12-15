import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose

# Import your custom interfaces here
# from my_project_msgs.srv import GetAssemblyPlan, DetectBricks
# from my_project_msgs.action import MoveToPose, AlignToTarget, ExecuteTask

class AssemblySupervisor(Node):

    def __init__(self):
        super().__init__('assembly_supervisor')
        
        # Callback group allows parallel execution (essential for Actions)
        self.cb_group = ReentrantCallbackGroup()

        # --- CLIENTS SETUP ---
        
        # 1. GUI Service Client
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        
        # 2. Camera Service Client
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        
        # 3. AR4 Action Clients
        self.ar4_point_client = ActionClient(self, MoveToPose, 'ar4_point_control', callback_group=self.cb_group)
        self.ar4_vs_client = ActionClient(self, AlignToTarget, 'ar4_visual_servo', callback_group=self.cb_group)
        
        # 4. ABB Action Client
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        self.get_logger().info('Supervisor Initialized. Waiting for services...')
        
        # Start the State Machine Loop
        self.timer = self.create_timer(1.0, self.state_machine_loop, callback_group=self.cb_group)
        
        # State Variables
        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []

    async def state_machine_loop(self):
        # We cancel the timer so this function doesn't get called repeatedly 
        # while we are already inside it. We will restart it or loop internally.
        self.timer.cancel()
        
        try:
            # ====================================================
            # STATE 1: SETUP & DETECTION
            # ====================================================
            if self.state == "INIT":
                self.get_logger().info('Requesting Assembly Plan from GUI...')
                # Wait for GUI service
                while not self.gui_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for GUI node...')
                
                # Call Service (Async)
                req = GetAssemblyPlan.Request()
                future = self.gui_client.call_async(req)
                result = await future
                self.assembly_queue = result.plan # Assuming response has a 'plan' list
                
                self.state = "DETECT"

            elif self.state == "DETECT":
                self.get_logger().info('Requesting Camera Detection...')
                req = DetectBricks.Request()
                result = await self.camera_client.call_async(req)
                self.detected_bricks = result.bricks
                
                # Simple logic to match Plan to Detected Bricks would go here
                self.state = "PROCESS_NEXT"

            elif self.state == "PROCESS_NEXT":
                if not self.assembly_queue:
                    self.get_logger().info('All tasks complete!')
                    self.state = "DONE"
                    return

                self.current_brick = self.assembly_queue.pop(0)
                # Determine strategy
                if self.current_brick.start_side == "ABB":
                    self.state = "EXECUTE_ABB_DIRECT"
                elif self.current_brick.target_side == "ABB":
                    self.state = "HANDOVER_SEQUENCE"
                else:
                    self.state = "EXECUTE_AR4_DIRECT"

            # ====================================================
            # STATE 2: AR4 SEQUENCE (Point -> Visual -> Grasp)
            # ====================================================
            elif self.state == "EXECUTE_AR4_DIRECT" or self.state == "AR4_PICK_FOR_HANDOVER":
                self.get_logger().info(f'Starting AR4 Pick Sequence for {self.current_brick.id}')
                
                # Step A: Approach with Z-Offset (Point Control)
                goal_msg = MoveToPose.Goal()
                goal_msg.target_pose = self.current_brick.pose
                goal_msg.strategy = "APPROACH_OFFSET" # Custom field
                
                await self.send_action_goal(self.ar4_point_client, goal_msg)
                
                # Step B: Visual Servoing Align
                self.get_logger().info('Switching to Visual Servoing...')
                vs_goal = AlignToTarget.Goal()
                vs_goal.object_id = self.current_brick.type
                
                await self.send_action_goal(self.ar4_vs_client, vs_goal)
                
                # Step C: Lower and Grasp (Point Control)
                self.get_logger().info('lowering and Grasping...')
                grasp_goal = MoveToPose.Goal()
                grasp_goal.target_pose = self.current_brick.pose # Accurate pose from VS
                grasp_goal.strategy = "GRASP"
                
                await self.send_action_goal(self.ar4_point_client, grasp_goal)

                # Check where to go next
                if self.state == "AR4_PICK_FOR_HANDOVER":
                    self.state = "HANDOVER_EXECUTION"
                else:
                    self.state = "PROCESS_NEXT"

            # ====================================================
            # STATE 3: HANDOVER SEQUENCE
            # ====================================================
            elif self.state == "HANDOVER_SEQUENCE":
                # First, get the object
                self.state = "AR4_PICK_FOR_HANDOVER"
                # The loop will cycle back to execute the AR4 pick, then come to HANDOVER_EXECUTION
                
            elif self.state == "HANDOVER_EXECUTION":
                self.get_logger().info('Starting Handover...')
                
                # 1. AR4 moves to Intermediate Pose
                goal = MoveToPose.Goal()
                goal.strategy = "GOTO_HANDOVER"
                await self.send_action_goal(self.ar4_point_client, goal)
                
                # 2. Update Camera Detection (Optional but recommended)
                # We need to see where the brick is exactly in the gripper
                req = DetectBricks.Request()
                det_result = await self.camera_client.call_async(req)
                handover_pose = det_result.handover_pose
                
                # 3. ABB Picks from AR4
                abb_goal = ExecuteTask.Goal()
                abb_goal.task_type = "PICK_FROM_HANDOVER"
                abb_goal.target_pose = handover_pose
                await self.send_action_goal(self.abb_client, abb_goal)
                
                # 4. AR4 Releases (triggered after ABB confirms grasp)
                # Ideally, ABB Action shouldn't return until it has grasped.
                release_goal = MoveToPose.Goal()
                release_goal.strategy = "RELEASE"
                await self.send_action_goal(self.ar4_point_client, release_goal)
                
                self.state = "PROCESS_NEXT"

        except Exception as e:
            self.get_logger().error(f'State Machine Failed: {e}')
        
        # Restart loop immediately to process next state
        if self.state != "DONE":
            self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)

    async def send_action_goal(self, client, goal_msg):
        """Helper to send goal and wait for result"""
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False
            
        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return False
            
        result_future = goal_handle.get_result_async()
        result = await result_future
        return result.result

def main(args=None):
    rclpy.init(args=args)
    node = AssemblySupervisor()
    
    # Use MultiThreadedExecutor so callbacks (like action feedback) can run
    # while the state machine loop awaits futures.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()