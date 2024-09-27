#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sam_msgs.msg import PercentStamped, Topics
from smarc_msgs.msg import Leak
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header
# from sam_msgs.msg import  SystemsCheckFeedback, SystemsCheckResult
from sam_msgs.action import SystemsCheck
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup



class StartupCheckServer(Node):

    def test_leak(self):

        self.get_logger().info("Checking leak sensor...")

        # feedback_topic = "/sam/core/leak_fb"
        feedback_topic = Topics.LEAK_TOPIC_FB

        try:
             leak = self.wait_for_message(Leak,feedback_topic,3.)
        except Exception:
            self.get_logger().info(f"Could not get leak message on {feedback_topic}, aborting...")
            self._result.status = "Could not get leak message on %s, aborting..." % feedback_topic
            return False

        if leak.value:
            self.get_logger().info("Got leak sensor and detected leaks, aborting...")
            self._result.status = "Got leak sensor and detected leaks, aborting..."
            return False
        else:
            self.get_logger().info("Got leak sensor and no leaks, seems ok!")
            self._feedback.status = "Got leak sensor and no leaks, seems ok!"

        return True

    def test_pressure(self):

        self.get_logger().info("Checking pressure...")

        feedback_topic = Topics.DEPTH_TOPIC

        try:
            pressure = self.wait_for_message(FluidPressure,feedback_topic,3.)
        except Exception:
            self.get_logger().info(f"Could not get pressure on {feedback_topic}, aborting...")
            self._result.status = "Could not get pressure on %s, aborting..." % feedback_topic
            return False

        self.get_logger().info(f"Got pressure message with value {pressure.fluid_pressure}, seems ok!")
        self._feedback.status = "Got pressure message with value %f, seems ok!" % pressure.fluid_pressure

        return True

    def test_lcg(self, lcg_setpoint):

        self.get_logger().info("Checking lcg...")

        self.lcg_cmd.publish(PercentStamped(value=lcg_setpoint, header=self.header)) # publish setpoint 50
        self.get_logger().info(f"Published lcg setpoint at {lcg_setpoint}, waiting....")
        # rclpy.sleep(5.) # wait for 5s to reach 50
        time.sleep(5.)
        # get lcg feedback, wait for 1s

        feedback_topic = Topics.LCG_FB_TOPIC
        try:
            lcg_feedback = self.wait_for_message( PercentStamped,feedback_topic,1.)
        except Exception:
            self.get_logger().info(f"Could not get feedback on {feedback_topic}, aborting...")
            self._result.status = "Could not get feedback on %s, aborting..." % feedback_topic
            return False

        if abs(lcg_feedback.value - lcg_setpoint) > 2.:
            self.get_logger().info(f"Set point was {lcg_setpoint} and value was {lcg_feedback.value}, aborting test...")
            self._result.status = "Set point was %f and value was %f, aborting test..." % (lcg_setpoint, lcg_feedback.value)
            return False
        else:
            self.get_logger().info(f"Set point was {lcg_setpoint} and value was {lcg_feedback.value}, seems ok!")
            self._feedback.status = "Set point was %f and value was %f, seems ok!" % (lcg_setpoint, lcg_feedback.value)
            return True

    def test_vbs(self, vbs_setpoint):

        self.get_logger().info("Checking vbs...")

        self.vbs_cmd.publish(PercentStamped(value=vbs_setpoint, header=self.header)) # publish setpoint 50

        self.get_logger().info(f"Published vbs setpoint at {vbs_setpoint}, waiting....")

        time.sleep(20.) # wait for 5s to reach 50
        # get vbs feedback, wait for 1s
        feedback_topic = Topics.VBS_FB_TOPIC

        try:
            vbs_feedback = self.wait_for_message(PercentStamped,feedback_topic,1.)
        except Exception:
            self.get_logger().info(f"Could not get feedback on {feedback_topic}, aborting...")
            self._result.status = "Could not get feedback on %s, aborting..." % feedback_topic
            return False

        if abs(vbs_feedback.value - vbs_setpoint) > 2.:
            self.get_logger().info(f"Set point was {vbs_setpoint} and value was {vbs_feedback.value}, aborting test...")
            self._result.status = "Set point was %f and value was %f, aborting test..." % (vbs_setpoint, vbs_feedback.value)
            return False
        else:
            self.get_logger().info(f"Set point was {vbs_setpoint} and value was {vbs_feedback.value}, seems ok!")
            self._feedback.status = "Set point was %f and value was %f, seems ok!" % (vbs_setpoint, vbs_feedback.value)
            return True

    def __init__(self, name):
        super().__init__('sam_startup_check')
        self._action_name = name
        self._as = ActionServer(self, SystemsCheck ,self._action_name, execute_callback=self.execute_cb,goal_callback=self.goal_cb)

        self.lcg_cmd = self.create_publisher(PercentStamped,Topics.LCG_CMD_TOPIC ,qos_profile=10)
        self.vbs_cmd = self.create_publisher(PercentStamped,Topics.VBS_CMD_TOPIC ,qos_profile=10)
        self._feedback = SystemsCheck.Feedback()
        self._result = SystemsCheck.Result()
        # self._as.start()

    def goal_cb(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def execute_cb(self, goal_handle):

        time.sleep(1.) # sleep for 1s to wait for stuff to start up

        self.header = Header()
        self.header.stamp = self.get_clock().now().to_msg()

        self.lcg_cmd.publish(PercentStamped(value=0., header=self.header)) # publish setpoint 0

        time.sleep(1.)

        if not self.test_lcg(50.):
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_lcg(88.):
            # goal_handle.abort()
            return #self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_vbs(0.):
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_vbs(100.):
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_pressure():
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        if not self.test_leak():
            goal_handle.abort()
            return self._result

        goal_handle.publish_feedback(self._feedback)

        self.get_logger().info("All tests successful! Exiting...")
        self._result.status = "All tests successful! Exiting..."
        goal_handle.succeed()

        return self._result
    



    def wait_for_message(self, msg_type, topic, timeout_sec=3.0):
        future = rclpy.task.Future()

        def callback(msg):
            future.set_result(msg)
        sub = self.create_subscription(msg_type, topic, callback, 10)
        
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        except Exception as e:
            self.get_logger().error(f"Error while waiting for message on {topic}: {e}")
            raise

        self.destroy_subscription(sub)

        if future.done():
            return future.result()
        else:
            raise TimeoutError()

def main(args=None):
    rclpy.init(args=args)

    check_server = StartupCheckServer("sam_startup_check")
    rclpy.spin(check_server)

    check_server.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()

