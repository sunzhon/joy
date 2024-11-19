import os, sys
import rclpy
from rclpy.node import Node
from ambot_msgs.msg import UserCommand 
import ast
import operator as op
import traceback
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
import threading
import time
from collections import OrderedDict
import math
import numpy as np

class RestrictedEvaluator(object):
    def __init__(self):
        self.operators = {
            ast.Add: op.add,
            ast.Sub: op.sub,
            ast.Mult: op.mul,
            ast.Div: op.truediv,
            ast.BitXor: op.xor,
            ast.USub: op.neg,
        }
        self.functions = {
            'abs': lambda x: abs(x),
            'max': lambda *x: max(*x),
            'min': lambda *x: min(*x),
        }

    def _reval_impl(self, node, variables):
        if isinstance(node, ast.Num):
            return node.n
        elif isinstance(node, ast.BinOp):
            op = self.operators[type(node.op)]
            return op(self._reval_impl(node.left, variables),
                      self._reval_impl(node.right, variables))
        elif isinstance(node, ast.UnaryOp):
            op = self.operators[type(node.op)]
            return op(self._reval_impl(node.operand, variables))
        elif isinstance(node, ast.Call) and node.func.id in self.functions:
            func = self.functions[node.func.id]
            args = [self._reval_impl(n, variables) for n in node.args]
            return func(*args)
        elif isinstance(node, ast.Name) and node.id in variables:
            return variables[node.id]
        elif isinstance(node, ast.Subscript) and node.value.id in variables:
            var = variables[node.value.id]
            idx = node.slice.value.n
            try:
                return var[idx]
            except IndexError:
                raise IndexError("Variable '%s' out of range: %d >= %d" % (node.value.id, idx, len(var)))
        else:
            raise TypeError("Unsupported operation: %s" % node)

    def reval(self, expr, variables):
        expr = str(expr)
        if len(expr) > 1000:
            raise ValueError("The length of an expression must not be more than 1000 characters")
        try:
            return self._reval_impl(ast.parse(expr, mode='eval').body, variables)
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            raise e


class JoyRemap(object, Node):
    def __init__(self, namespace):
        self.evaluator = RestrictedEvaluator()

        # ROS2 publish
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=1
        )
        self.pub_joy = self.create_publisher(
            Joy,
            "/joy_out",
            qos_profile
        )

        # ROS2 subscribers
        self.low_state_sub = self.create_subscription(
            Joy,
            "/joy_in",
            self.callback,
            1
        )

        # ROS2 publish
        self.joy_stick_sub = self.create_publisher(
            UserCommand,
            "/joy_cmd",
            qos_profile
        )

        self.speed_gain = 1.0
        self.mapping={"axes":[1,2,3,4,5,6,7,8],"buttons":[1,2,3,4]}


    def callback(self, in_msg):
        out_msg = Joy(header=in_msg.header)
        map_axes = self.mappings["axes"]
        map_btns = self.mappings["buttons"]
        out_msg.axes = [0.0] * len(map_axes)
        out_msg.buttons = [0] * len(map_btns)
        in_dic = {"axes": in_msg.axes, "buttons": in_msg.buttons}
        for i, exp in enumerate(map_axes):
            try:
                import pdb;pdb.set_trace()
                out_msg.axes[i] = self.evaluator.reval(exp, in_dic)
            except NameError as e:
                self.get_logger().error(f"You are using vars other than 'buttons' or 'axes': {e}")
            except UnboundLocalError as e:
                self.get_logger().error(f"Wrong form: {e}")
            except Exception as e:
                raise e

        for i, exp in enumerate(map_btns):
            try:
                if self.evaluator.reval(exp, in_dic) > 0:
                    out_msg.buttons[i] = 1
            except NameError as e:
                self.get_logger().error(f"You are using vars other than 'buttons' or 'axes': {e}")
            except UnboundLocalError as e:
                self.get_logger().error(f"Wrong form: {e}")
            except Exception as e:
                raise e

        # fill twist msg
        twist_msg =  UserCommand()
        twist_msg.vx = out_msg.axes[0]
        twist_msg.vy = out_msg.axes[1]
        twist_msg.wz = out_msg.axes[2]

        # pub message
        self.pub_joy.publish(out_msg)
        self.pub_twist.publish(twist_msg)


def main(args):
    rclpy.init("Joy_node")
    n = JoyRemap(args.namespace)
    while rclpy.ok():
        rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
