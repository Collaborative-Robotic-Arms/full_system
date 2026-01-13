#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from supervisor_package.srv import GetAssemblyPlan, DetectBricks
from supervisor_package.action import MoveToPose, AlignToTarget, ExecuteTask

# --- LOCAL IMPORTS ---
from .gripper_manager import GripperManager

class AssemblySupervisor(Node):
    def __init__(self):
        super().__init__('supervisor')

        self.cb_group = ReentrantCallbackGroup()
        
        # --- GRIPPER INITIALIZATION ---
        self.declare_parameter('use_sim', True)
        use_sim_value = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.gripper = GripperManager(self, use_sim=use_sim_value)

        # --- CLIENTS ---
        self.gui_client = self.create_client(GetAssemblyPlan, 'get_assembly_plan', callback_group=self.cb_group)
        self.camera_client = self.create_client(DetectBricks, 'detect_bricks', callback_group=self.cb_group)
        self.ar4_point_client = ActionClient(self, MoveToPose, 'ar4_point_control', callback_group=self.cb_group)
        self.ar4_vs_client = ActionClient(self, AlignToTarget, 'ar4_visual_servo', callback_group=self.cb_group)
        self.abb_client = ActionClient(self, ExecuteTask, 'abb_control', callback_group=self.cb_group)

        self.get_logger().info('Supervisor Initialized. Waiting for services...')

        self.state = "INIT"
        self.current_brick = None
        self.assembly_queue = []

        self.timer = self.create_timer(1.0, self.state_machine_loop, callback_group=self.cb_group)

    async def state_machine_loop(self):
        self.timer.cancel()
        try:
            # =========================
            # STATE 1: INIT → DETECTION
            # =========================
            if self.state == "INIT":
                self.get_logger().info('Requesting Assembly Plan from GUI...')
                while not self.gui_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for GUI node...')

                # Reset grippers at start
                self.gripper.set_ar4_grip('OPEN')
                self.gripper.set_abb_grip('OPEN')

                req = GetAssemblyPlan.Request()
                result = await self.gui_client.call_async(req)
                self.assembly_queue = result.plan
                self.state = "DETECT"

            elif self.state == "DETECT":
                self.get_logger().info('Requesting Camera Detection...')
                req = DetectBricks.Request()
                result = await self.camera_client.call_async(req)
                self.detected_bricks = result.bricks
                self.state = "PROCESS_NEXT"

            # =========================
            # STATE 2: PROCESS NEXT
            # =========================
            elif self.state == "PROCESS_NEXT":
                if not self.assembly_queue:
                    self.get_logger().info('All tasks complete!')
                    self.state = "DONE"
                    return

                self.current_brick = self.assembly_queue.pop(0)
                if self.current_brick.location == 1:
                    self.state = "EXECUTE_ABB_PICK"
                elif self.current_brick.location == 2:
                    self.state = "HANDOVER_SEQUENCE"
                else:
                    self.state = "EXECUTE_AR4_DIRECT"

            # =========================
            # STATE 3: AR4 PICK & PLACE
            # =========================
            elif self.state in ["EXECUTE_AR4_DIRECT", "AR4_PICK_FOR_HANDOVER"]:
                self.get_logger().info(f'Starting AR4 Pick Sequence for {self.current_brick.id}')

                # Step A: Approach
                goal_msg = MoveToPose.Goal()
                goal_msg.target_pose = self.current_brick.pickup_pose 
                goal_msg.strategy = "APPROACH_OFFSET"
                await self.send_action_goal(self.ar4_point_client, goal_msg)

                # Step B: Visual Servoing
                self.get_logger().info('Switching to Visual Servoing...')
                vs_goal = AlignToTarget.Goal()
                vs_goal.object_id = self.current_brick.type
                await self.send_action_goal(self.ar4_vs_client, vs_goal)

                # Step C: Lower and Grasp
                self.get_logger().info('Lowering and Grasping...')
                grasp_goal = MoveToPose.Goal()
                grasp_goal.target_pose = self.current_brick.pickup_pose 
                grasp_goal.strategy = "GRASP"
                await self.send_action_goal(self.ar4_point_client, grasp_goal)

                # Close AR4 Gripper
                self.gripper.set_ar4_grip('CLOSE')

                self.state = "AR4_PLACE_ON_GRID" if self.state == "EXECUTE_AR4_DIRECT" else "HANDOVER_EXECUTION"

            elif self.state == "AR4_PLACE_ON_GRID":
                self.get_logger().info('Moving AR4 to pre-calculated place_pose...')

                # Use direct place_pose from brick_processor
                place_goal = MoveToPose.Goal()
                place_goal.target_pose = self.current_brick.place_pose 
                place_goal.strategy = "PLACE"
                await self.send_action_goal(self.ar4_point_client, place_goal)

                # Open AR4 Gripper
                self.gripper.set_ar4_grip('OPEN')

                self.state = "PROCESS_NEXT"

            # =========================
            # STATE 4: ABB PICK & PLACE
            # =========================
            elif self.state == "EXECUTE_ABB_PICK":
                self.get_logger().info(f'Starting ABB Pick for {self.current_brick.id}')

                abb_pick_goal = ExecuteTask.Goal()
                abb_pick_goal.task_type = "PICK"
                abb_pick_goal.target_pose = self.current_brick.pickup_pose 
                await self.send_action_goal(self.abb_client, abb_pick_goal)

                # Close ABB Gripper
                self.gripper.set_abb_grip('CLOSE')

                self.state = "EXECUTE_ABB_PLACE"
                
            elif self.state == "EXECUTE_ABB_PLACE":
                self.get_logger().info('Moving ABB to pre-calculated place_pose...')

                abb_place_goal = ExecuteTask.Goal()
                abb_place_goal.task_type = "PLACE"
                abb_place_goal.target_pose = self.current_brick.place_pose 
                await self.send_action_goal(self.abb_client, abb_place_goal)

                # Open ABB Gripper
                self.gripper.set_abb_grip('OPEN')

                self.state = "PROCESS_NEXT"
                
            # =========================
            # STATE 5: HANDOVER SEQUENCE
            # =========================
            elif self.state == "HANDOVER_SEQUENCE":
                self.state = "AR4_PICK_FOR_HANDOVER"

            elif self.state == "HANDOVER_EXECUTION":
                self.get_logger().info('Starting Handover...')

                # AR4 → intermediate pose
                goal = MoveToPose.Goal()
                goal.strategy = "GOTO_HANDOVER"
                await self.send_action_goal(self.ar4_point_client, goal)

                # Update camera detection
                req = DetectBricks.Request()
                det_result = await self.camera_client.call_async(req)
                handover_pose = det_result.handover_pose

                # ABB picks
                abb_goal = ExecuteTask.Goal()
                abb_goal.task_type = "PICK_FROM_HANDOVER"
                abb_goal.target_pose = handover_pose
                await self.send_action_goal(self.abb_client, abb_goal)

                # Close ABB Gripper
                self.gripper.set_abb_grip('CLOSE')

                # AR4 releases
                self.gripper.set_ar4_grip('OPEN')
                release_goal = MoveToPose.Goal()
                release_goal.strategy = "RELEASE"
                await self.send_action_goal(self.ar4_point_client, release_goal)

                self.state = "PROCESS_NEXT"

        except Exception as e:
            self.get_logger().error(f'State Machine Failed: {e}')

        if self.state != "DONE":
            self.timer = self.create_timer(0.1, self.state_machine_loop, callback_group=self.cb_group)

    async def send_action_goal(self, client, goal_msg):
        if not client.wait_for_server(timeout_sec=5.0):
            return False
        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        result = await result_future
        return result.result

def main(args=None):
    rclpy.init(args=args)
    node = AssemblySupervisor()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()