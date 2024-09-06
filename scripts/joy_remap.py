#!/home/cc/.pyenv/versions/machinelearning/bin/python3
# -*- coding: utf-8 -*-
#!/home/suntao/.pyenv/versions/machinelearning/bin/python3
#/usr/bin/env python3
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import ast
import operator as op
import rospy
import traceback
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
import threading
import time
from collections import OrderedDict
import math

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


class JoyRemap(object):
    def __init__(self, namespace):
        self.namespace = namespace
        self.evaluator = RestrictedEvaluator()
        self.mappings = self.load_mappings("~mappings")
        self.warn_remap(self.namespace + "/joy_out")
        self.pub_joy = rospy.Publisher(self.namespace + "/"
            "joy_out", Joy, queue_size=1)
        self.warn_remap(self.namespace + "/joy_in")
        self.sub = rospy.Subscriber(self.namespace + "/"
            "joy_in", Joy, self.callback,
            queue_size=rospy.get_param("~queue_size", None))
        self.pub_twist = rospy.Publisher(self.namespace + "/"
            "cmd_vel", Twist, queue_size=1)
    
        # param server
        self.rosparams = {}
        operation_params = rospy.get_param("joy_remap/operation_cmds")
        for key in operation_params:
            self.rosparams[self.namespace+"/operation_cmd/"+key] = 0

        import copy
        self.rosparams_old = copy.copy(self.rosparams)
        self.rosparam_server()

    def load_mappings(self, ns):
        btn_remap = rospy.get_param(ns + "/buttons", [])
        axes_remap = rospy.get_param(ns + "/axes", [])
        rospy.loginfo("loaded remapping: %d buttons, %d axes" % (len(btn_remap), len(axes_remap)))
        return {"buttons": btn_remap, "axes": axes_remap}

    def warn_remap(self, name):
        if name == rospy.remap_name(name):
            rospy.logwarn("topic '%s' is not remapped" % name)

    def callback(self, in_msg):
        out_msg = Joy(header=in_msg.header)
        map_axes = self.mappings["axes"]
        map_btns = self.mappings["buttons"]
        out_msg.axes = [0.0] * len(map_axes)
        out_msg.buttons = [0] * len(map_btns)
        in_dic = {"axes": in_msg.axes, "buttons": in_msg.buttons}
        for i, exp in enumerate(map_axes):
            try:
                out_msg.axes[i] = self.evaluator.reval(exp, in_dic)
            except NameError as e:
                rospy.logerr("You are using vars other than 'buttons' or 'axes': %s" % e)
            except UnboundLocalError as e:
                rospy.logerr("Wrong form: %s" % e)
            except Exception as e:
                raise e

        for i, exp in enumerate(map_btns):
            try:
                if self.evaluator.reval(exp, in_dic) > 0:
                    out_msg.buttons[i] = 1
            except NameError as e:
                rospy.logerr("You are using vars other than 'buttons' or 'axes': %s" % e)
            except UnboundLocalError as e:
                rospy.logerr("Wrong form: %s" % e)
            except Exception as e:
                raise e

        # fill twist msg
        twist_msg =  Twist()
        twist_msg.linear.x = out_msg.axes[0]
        twist_msg.linear.y = out_msg.axes[1]
        twist_msg.angular.z = out_msg.axes[2]

        # fill ros parameter servers
        keys=["Vx","Vy","Rz"]
        for idx in range(3): # walking speeds
            if out_msg.axes[idx]!=0:
                tmp = self.namespace+"/operation_cmd/"+keys[idx]
                self.rosparams[tmp] = out_msg.axes[idx]
                break

        for i in range(2): # button 0-1 is gaits
            if out_msg.buttons[i]==1:
                tmp = self.namespace+"/operation_cmd/"+"gait"
                self.rosparams[tmp]  += 1 if i==0 else -1
                self.rosparams[tmp] %= 4
                break

        for i in range(2,4): # button 2,3 is fre
            if out_msg.buttons[i]==1:
                tmp = self.namespace+"/operation_cmd/"+"gait_frequency_cmd"
                self.rosparams[tmp] += 1 if i==2 else -1
                self.rosparams[tmp] = self.rosparams[tmp] if self.rosparams[tmp] > 1 else 1
                self.rosparams[tmp] %= 5
                break

        for i in range(4,6): # button 4,5 is mode
            if out_msg.buttons[i]==1:
                tmp = self.namespace+"/operation_cmd/"+"motion_mode"
                self.rosparams[tmp] += 1 if i==4 else -1
                self.rosparams[tmp] %= 4
                break

	
        self.rosparams[self.namespace+"/operation_cmd/"+"body_height_cmd"] += 0.010*out_msg.axes[6]
        self.rosparams[self.namespace+"/operation_cmd/"+"feetswing_height_cmd"] += 0.010*out_msg.axes[7]


        # pub message
        self.pub_joy.publish(out_msg)
        self.pub_twist.publish(twist_msg)

    def rosparam_server(self):
        self.lock  =  threading.Lock()
        thread = threading.Thread(target=self._rosparam_server, daemon=True)
        thread.start()

    def _rosparam_server(self):
        while not rospy.is_shutdown():
            self.lock.acquire()
            for key, value in self.rosparams.items():
                if self.rosparams_old[key] != value: # checking whether the rosparam value changed
                    self.rosparams_old[key] = value
                    if(rospy.has_param(key)):
                        rospy.set_param(key, value)
            self.lock.release()
            time.sleep(0.5)



def main(args):
    rospy.init_node("joy_remap")
    n = JoyRemap(args.namespace)
    rospy.spin()

if __name__ == '__main__':
    """ The script to run the Ambot script in ROS.
    It's designed as a main function and not designed to be a scalable code.
    """
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace",
        type= str,
        default= "/ambot_v1",                    
    )
    parser.add_argument("--logdir",
        type= str,
        help= "The log directory of the trained model",
        default= None,
    )
    parser.add_argument("--walkdir",
        type= str,
        help= "The log directory of the walking model, not for the skills.",
        default= None,
    )
    parser.add_argument("--mode",
        type= str,
        help= "The mode to determine which computer to run on.",
        choices= ["jetson", "upboard", "full"],                
    )
    parser.add_argument("--debug",
        action= "store_true",
    )

    args, unknown = parser.parse_known_args()
    print(args)
    main(args)
