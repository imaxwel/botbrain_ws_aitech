#!/usr/bin/env python3
import rclpy
# Key imports for a lifecycle node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from bot_custom_interfaces.srv import Mode, SpeedLevel, CurrentMode, LightControl
from bot_custom_interfaces.msg import RobotStatus
from unitree_api.msg import Request, Response
from geometry_msgs.msg import Twist
from unitree_go.msg import SportModeState
from std_srvs.srv import SetBool
import numpy as np
import json
import time

class RobotWriteNode(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_robot_write_node')

        # Initialize internal state variables
        self.emergency_flag = True
        self.light_state = False
        self.op_mode = None
        self.gait_type = None
        self.sport_response = None
        self.robot_state_response = None
        self.sport_request_id = None

        # Initialize ROS communicator handles to None. They will be created in on_configure().
        self.callback_group = None
        self.publisher_ = None
        self.robot_state_publisher_ = None
        self.vui_publisher_ = None
        self.robot_status_publisher_ = None
        self.robot_status_timer_ = None
        self.cmd_vel_subscription = None
        self.sports_mode_subscription = None
        self.sport_response_subscription = None
        self.robot_state_response_subscription = None
        self.vui_subscription = None
        self.set_mode_srv = None
        self.set_speed_level_service_ = None
        self.get_current_mode_srv = None
        self.set_emergency_stop_srv = None
        self.set_light_srv = None
        
        self.get_logger().info("Lifecycle Mode Service Node created, in 'unconfigured' state.")


    def on_configure(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_configure() is called.")

        self.callback_group = ReentrantCallbackGroup()

        # Publishers
        self.publisher_ = self.create_publisher(Request, '/api/sport/request', 1)
        self.robot_state_publisher_ = self.create_publisher(Request, '/api/robot_state/request', 1)
        self.vui_publisher_ = self.create_publisher(Request, '/api/vui/request', 1)
        self.robot_status_publisher_ = self.create_publisher(RobotStatus, 'robot_status', 1)
        
        # Timer (created but not started)
        self.robot_status_timer_ = self.create_timer(0.5, self.robot_status_callback, callback_group=self.callback_group)
        self.robot_status_timer_.cancel()

        # QoS: BEST_EFFORT + depth 1 — DDS drops old msgs at network layer,
        # so the executor only dispatches ~1 callback per spin cycle instead of 500/s.
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscriptions
        self.cmd_vel_subscription = self.create_subscription(
            Twist, 'cmd_vel_out', self.cmd_vel_subscription_callback, 1, callback_group=self.callback_group)
        self.sports_mode_subscription = self.create_subscription(
            SportModeState, '/lf/sportmodestate', self.sport_state_subscription_callback, qos_best_effort, callback_group=self.callback_group)
        self.sport_response_subscription = self.create_subscription(
            Response, '/api/sport/response', self.sport_response_callback, 10, callback_group=self.callback_group)
        self.robot_state_response_subscription = self.create_subscription(
            Response, '/api/robot_state/response', self.robot_state_response_callback, 10, callback_group=self.callback_group)
        self.vui_subscription = self.create_subscription(
            Response, '/api/vui/response', self.vui_callback, 10, callback_group=self.callback_group)

        # Services
        self.set_mode_srv = self.create_service(Mode, 'mode', self.handle_mode, callback_group=self.callback_group)
        self.set_speed_level_service_ = self.create_service(SpeedLevel, 'speed_level', self.handle_speed_level, callback_group=self.callback_group)
        self.get_current_mode_srv = self.create_service(CurrentMode, 'current_mode', self.get_current_mode, callback_group=self.callback_group)
        self.set_emergency_stop_srv = self.create_service(SetBool, 'emergency_stop', self.set_emergency_stop, callback_group=self.callback_group)
        self.set_light_srv = self.create_service(LightControl, 'light_control', self.set_light, callback_group=self.callback_group)

        self.get_logger().info("Node configured successfully.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        """Activate the node, start timers, and allow callbacks to be processed."""
        self.get_logger().info("on_activate() is called.")

        # Activate subscriptions first so they can receive responses
        super().on_activate(state)

        # Start the status publishing timer
        self.robot_status_timer_.reset()

        self.get_logger().info("Node activated. Services and subscriptions are now responsive.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:
        """Deactivate the node, stopping timers and callback processing."""
        self.get_logger().info("on_deactivate() is called.")
        # Stop the status publishing timer
        self.robot_status_timer_.cancel()
        super().on_deactivate(state)
        self.get_logger().info("Node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_cleanup() is called.")
        
        # Destroy all communicators
        self.destroy_publisher(self.publisher_)
        self.destroy_publisher(self.robot_state_publisher_)
        self.destroy_publisher(self.vui_publisher_)
        self.destroy_publisher(self.robot_status_publisher_)
        self.destroy_timer(self.robot_status_timer_)
        self.destroy_subscription(self.cmd_vel_subscription)
        self.destroy_subscription(self.sports_mode_subscription)
        self.destroy_subscription(self.sport_response_subscription)
        self.destroy_subscription(self.robot_state_response_subscription)
        self.destroy_subscription(self.vui_subscription)
        self.destroy_service(self.set_mode_srv)
        self.destroy_service(self.set_speed_level_service_)
        self.destroy_service(self.get_current_mode_srv)
        self.destroy_service(self.set_emergency_stop_srv)
        self.destroy_service(self.set_light_srv)
        
        self.get_logger().info("Node resources cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: rclpy.lifecycle.State) -> TransitionCallbackReturn:

        self.get_logger().info("on_shutdown() is called.")
        self.on_cleanup(state)
        return TransitionCallbackReturn.SUCCESS


    def sport_state_subscription_callback(self, msg):
        self.op_mode = msg.mode
        self.gait_type = msg.gait_type

    def sport_response_callback(self, msg):
        if msg.header.identity.api_id == self.sport_request_id:
            self.sport_response = msg
        else:
            pass

    def robot_state_response_callback(self, msg):
        self.robot_state_response = msg

    def cmd_vel_subscription_callback(self, msg):
        if self.emergency_flag and (self.op_mode==1 or self.op_mode==3):
            req_cmd_vel = Request()
            req_cmd_vel.header.identity.api_id = 1008

            lx = msg.linear.x
            ly = msg.linear.y
            az = msg.angular.z

            if lx or ly or az:
                js_cmd_vel = {"x": lx,
                              "y": ly,
                              "z": az}
                req_cmd_vel.parameter = json.dumps(js_cmd_vel)
                self.publisher_.publish(req_cmd_vel)

    def handle_mode(self, request, response):
        mode = request.mode
        req = Request()

        if mode == 'damp':
            response.message = 'Change the mode to Damp'
            req.header.identity.api_id = 1001  # Damp
        elif mode == 'balance_stand':
            response.message = 'Change the mode to BalanceStand'
            req.header.identity.api_id = 1002  # BalanceStand
        elif mode == 'stand_up':
            response.message = 'Change the mode to StandUp'
            req.header.identity.api_id = 1004  # StandUp
        elif mode == 'stand_down':
            response.message = 'Change the mode to StandDown'
            # First ensure robot is in stand_up mode
            if self.op_mode != 6:  # 6 is stand_up mode
                req_stand_up = Request()
                req_stand_up.header.identity.api_id = 1004  # StandUp
                self.sport_request_id = req_stand_up.header.identity.api_id
                temp_response = Mode.Response()
                result = self.send_sport_request(req_stand_up, temp_response)
                if not result.success:
                    response.success = False
                    response.message = 'Failed to stand up before lying down'
                    return response
                # Wait a bit for the robot to stabilize in stand_up mode
                time.sleep(1.0)
            # Now send stand_down command
            req.header.identity.api_id = 1005  # StandDown    
        else:
            response.success = False
            response.message = f'Invalid mode: {mode}. Valid modes: damp, balance_stand, stand_up, stand_down'
            return response

        self.sport_request_id = req.header.identity.api_id
        return self.send_sport_request(req, response)

    def handle_speed_level(self, request, response):
        if request.level < -1 or request.level > 1:
            response.success = False
            response.message = "Speed level is out of range [-1 ~ 1]"
            return response

        js = {"data": request.level}

        req = Request()
        req.parameter = json.dumps(js)
        req.header.identity.api_id = 1015 # SpeedLevel

        return self.send_sport_request(req, response)

    def get_current_mode(self, request, response):
        if self.op_mode == 1:
            response.mode = "balance_stand"
        elif self.op_mode == 5:
            response.mode = "stand_down"
        elif self.op_mode == 6:
            response.mode = "stand_up"
        elif self.op_mode == 7:
            response.mode = "damp"
        else:
            response.mode = "Unknown"
        return response

    def set_emergency_stop(self, request, response):
        #if request.data == True:
        if self.emergency_flag == True:
            req_msg = Request()
            self.emergency_flag = False

            req_msg.header.identity.api_id = 1008

            js_cmd_vel = {"x": 0, 
                          "y": 0, 
                          "z": 0} 
            
            req_msg.parameter = json.dumps(js_cmd_vel)
            self.publisher_.publish(req_msg)

            time.sleep(0.1)
            
            req_msg.header.identity.api_id = 1007

            js_pose = {"x": 0, 
                       "y": 0, 
                       "z": 0} 
            
            req_msg.parameter = json.dumps(js_pose)
            self.publisher_.publish(req_msg)

            time.sleep(1)
            
            req_msg.header.identity.api_id = 1003
            self.publisher_.publish(req_msg)

            time.sleep(1)

            req_msg.header.identity.api_id = 1005
            self.publisher_.publish(req_msg)

            time.sleep(4)

            req_msg.header.identity.api_id = 1001
            self.publisher_.publish(req_msg)
            response.message =  "Emergency Stop Set"
        else:
            self.emergency_flag = True
            response.message = "Emergency Stop Reset"

        response.success = True
        return response

    def send_sport_request(self, request, response):
        if self.emergency_flag and self.op_mode is not None:
            # Clear previous response
            self.sport_response = None

            # Publish the request
            self.publisher_.publish(request)

            timeout = time.time()
            limit = 4.0

            while True:
                if (time.time() - timeout) > limit:
                    response.success = False
                    break

                # Check if we received a response
                if self.sport_response is not None:
                    # Check if the response matches the request api_id and status code is 0 (success)
                    if (self.sport_response.header.identity.api_id == request.header.identity.api_id and
                        self.sport_response.header.status.code == 0):
                        response.success = True
                        break
                    else:
                        # Response doesn't match or has error code
                        response.success = False
                        break
        else:
            response.success = False

        return response

    def send_robot_state_request(self, request, response):
        # Clear previous response
        self.robot_state_response = None

        # Publish the request
        self.robot_state_publisher_.publish(request)

        timeout = time.time()
        limit = 4.0

        while True:
            if (time.time() - timeout) > limit:
                response.success = False
                break

            # Check if we received a response
            if self.robot_state_response is not None:
                # Check if the response matches the request api_id and status code is 0 (success)
                if (self.robot_state_response.header.identity.api_id == request.header.identity.api_id and
                    self.robot_state_response.header.status.code == 0):
                    response.success = True
                    break
                else:
                    # Response doesn't match or has error code
                    response.success = False
                    break

            time.sleep(0.01)

        return response
    
    def set_light(self, request, response):
        if self.op_mode is not None:
            req = Request()
            req.header.identity.api_id = 1005
            js = {"brightness": request.brightness}
            if request.control == False:
                js = {"brightness": 0}
                req.parameter = json.dumps(js)
                self.vui_publisher_.publish(req)
                response.success = True
                return response
            if request.brightness < 1 or request.brightness > 10:
                response.success = False
                response.message = "Intensidade fora dos limites. Selecione um valor de 1-10"
            else:
                req.parameter = json.dumps(js)
                response.success = True
                self.vui_publisher_.publish(req)
        else:
            response.success = False
        return response

    def vui_callback(self, msg):
        if msg.header.identity.api_id == 1006:
            data = json.loads(msg.data)
            if data["brightness"] > 0:
                self.light_state = True
            else:
                self.light_state = False

    def robot_status_callback(self):
        msg = RobotStatus()
        msg.emergency_stop = not self.emergency_flag
        msg.light_state = self.light_state
        self.robot_status_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotWriteNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    period = 1.0 / 20.0  # 20 Hz
    try:
        while rclpy.ok():
            for _ in range(12):
                executor.spin_once(timeout_sec=0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()