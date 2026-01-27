#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from dual_arms_msgs.action import ExecuteTask
from supervisor_package.action import MoveToPose, AlignToTarget
from std_srvs.srv import SetBool
from action_msgs.msg import GoalStatus

from copy import deepcopy

class AR4Controller(Node):
    def __init__(self):
        super().__init__('ar4_controller')
        self.cb_group = ReentrantCallbackGroup()
        
        self.holding_object = False

        # --- GRIPPER CLIENT ---
        self.gripper_client = self.create_client(SetBool, 'ar4_gripper/set')

        # --- ARM ACTION CLIENTS ---
        self.move_client = ActionClient(self, MoveToPose, 'ar4_point_control', callback_group=self.cb_group)
        self.vs_client = ActionClient(self, AlignToTarget, 'ar4_visual_servo', callback_group=self.cb_group)

        # --- ACTION SERVER TO RECEIVE SUPERVISOR TASKS ---
        self.execute_task_server = ActionServer(
            self,
            ExecuteTask,
            'ar4_controller/execute_task',
            execute_callback=self.execute_task_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('AR4 Controller Ready.')

    # ---------- HELPER FUNCTIONS ----------
    async def send_action_goal(self, client, goal_msg):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server {client._action_name} not available!')
            return None

        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected by {client._action_name}')
            return None

        result_future = goal_handle.get_result_async()
        result = await result_future

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return result.result
        else:
            self.get_logger().error(f'Action {client._action_name} failed with status: {result.status}')
            return None

    async def set_gripper(self, close: bool):
        """Set gripper state: True = Close, False = Open"""
        if not self.gripper_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Gripper service not available!")
            return False

        req = SetBool.Request()
        req.data = close
        future = self.gripper_client.call_async(req)
        result = await future
        return result.success

    async def execute_pick_sequence(self, target_pose, goal_handle):
        self.get_logger().info(f'Starting AR4 Pick Sequence')

        feedback_msg = ExecuteTask.Feedback()
        feedback_msg.progress = 0.0
        feedback_msg.current_status = "OPEN_GRIPPER"
        goal_handle.publish_feedback(feedback_msg)
        
        await self.set_gripper(True)  # Open
        self.get_logger().info(f'AR4 Gripper Opened')

        # Step A: Approach
        feedback_msg.progress = 0.2
        feedback_msg.current_status = "APPROACHING"
        goal_handle.publish_feedback(feedback_msg)
        
        approach_goal = MoveToPose.Goal()
        approach_goal.target_pose = target_pose
        approach_goal.strategy = "APPROACH_OFFSET"
        action_result = await self.send_action_goal(self.move_client, approach_goal)
        if action_result is None: return False
        self.get_logger().info('AR4 Moved to Approach')

        # Step B: Close Gripper
        feedback_msg.progress = 0.5
        feedback_msg.current_status = "CLOSING_GRIPPER"
        goal_handle.publish_feedback(feedback_msg)
        
        await self.set_gripper(False)

        # # Step C: Visual Servoing
        # vs_goal = AlignToTarget.Goal()
        # # vs_goal.object_id = brick_type
        # action_result = await self.send_action_goal(self.vs_client, vs_goal)
        # if action_result is None: return False
        # self.get_logger().info('Visual Servoing Done')
        self.get_logger().info('Skipped VS')

        # Step D: Grasp
        
        feedback_msg.progress = 0.9
        feedback_msg.current_status = "GRASPING"
        goal_handle.publish_feedback(feedback_msg)
        
        grasp_goal = MoveToPose.Goal()
        grasp_goal.target_pose = target_pose
        grasp_goal.strategy = "GRASP"
        action_result = await self.send_action_goal(self.move_client, grasp_goal)
        if action_result is None: return False
        self.get_logger().info('Grasp Completed')
        feedback_msg.progress = 1.0
        feedback_msg.current_status = "DONE"
        goal_handle.publish_feedback(feedback_msg)
        
        self.holding_object = True
        
        return True

    async def execute_place_sequence(self, place_pose, goal_handle):
        self.get_logger().info('Starting AR4 Place Sequence')

        # --- WAIT FOR FLAG ---
        wait_time = 0
        timeout = 10.0 # Wait up to 10 seconds for the pick to register
        
        while not self.holding_object:
            if wait_time > timeout:
                self.get_logger().error("ABORTING PLACE: Timed out waiting for Pick Flag!")
                return False
            
            self.get_logger().warn(f"Waiting for object... ({wait_time}/{timeout}s)")
            await asyncio.sleep(0.5) # Yield control
            wait_time += 0.5

        # If we get here, self.holding_object is True!
        feedback = ExecuteTask.Feedback()

        # 1. PRE-PLACE
        preplace_pose = deepcopy(place_pose)
        preplace_pose.position.z += 0.1

        feedback.current_status = "PRE_PLACE"
        goal_handle.publish_feedback(feedback)

        preplace_goal = MoveToPose.Goal()
        preplace_goal.target_pose = preplace_pose
        preplace_goal.strategy = "PLACE"

        if await self.send_action_goal(self.move_client, preplace_goal) is None:
            return False

        # 2. PLACE
        feedback.current_status = "PLACE"
        goal_handle.publish_feedback(feedback)

        place_goal = MoveToPose.Goal()
        place_goal.target_pose = place_pose
        place_goal.strategy = "PLACE"

        if await self.send_action_goal(self.move_client, place_goal) is None:
            return False

        # 3. RELEASE
        feedback.current_status = "RELEASE"
        goal_handle.publish_feedback(feedback)

        await self.set_gripper(True)

        # --- RESET FLAG ---
        self.holding_object = False # Ready for next task

        # 4. HOME
        self.get_logger().info("Returning AR4 to HOME...")
        home_goal = MoveToPose.Goal()
        home_goal.strategy = "HOME"
        
        await self.send_action_goal(self.move_client, home_goal)
        
        feedback.current_status = "DONE"
        goal_handle.publish_feedback(feedback)
        
        return True
    
    # ---------- ACTION SERVER CALLBACK ----------
    async def execute_task_callback(self, goal_handle):
            goal = goal_handle.request
            task_type = goal.task_type
            target_pose = goal.target_pose

            self.get_logger().info(f"Received ExecuteTask: {task_type}")

            success = False
            if task_type == "PICK":
                success = await self.execute_pick_sequence(target_pose, goal_handle)
            elif task_type == "PLACE":
                success = await self.execute_place_sequence(target_pose, goal_handle)

            result = ExecuteTask.Result()
            result.success = success
            
            # CRITICAL: Only succeed if the sequence actually finished
            if success:
                self.get_logger().info(f"Task {task_type} Succeeded.")
                goal_handle.succeed()
            else:
                self.get_logger().error(f"Task {task_type} FAILED.")
                goal_handle.abort() # This tells the supervisor to stop!
                
            return result

# ---------- MAIN ----------
def main(args=None):
    rclpy.init(args=args)
    node = AR4Controller()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
