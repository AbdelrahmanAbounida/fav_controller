#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from controllers.cfg import PidControlMinimalConfig
import threading
from controllers.msg import config_msg


class ConfigurableNode:
    def __init__(self):
        rospy.init_node("config_publisher")
        self.data_lock = threading.RLock()

        self.conf_pub = rospy.Publisher(
            "controllers_config", config_msg, queue_size=1
        )

        self.dyn_server = Server(PidControlMinimalConfig, self.on_pid_dyn_reconfigure)
        rospy.spin()

    def on_pid_dyn_reconfigure(self, config, level):
        # the config parameters are provided as dictionary. The keys are the
        # parameter names we specified in cfg/PidControl.cfg

        # use data_lock to avoid parallel modifications of the variables
        # from different threads (here the main thread running the loop in the
        # run() method and the thread runing the dynamic_reconfigure callback).
        with self.data_lock:
            message = config_msg()
            message.px_gain = config["px_gain"]
            message.ix_gain = config["ix_gain"]
            message.dx_gain = config["dx_gain"]
            message.py_gain = config["py_gain"]
            message.iy_gain = config["iy_gain"]
            message.dy_gain = config["dy_gain"]
            message.pz_gain = config["pz_gain"]
            message.iz_gain = config["iz_gain"]
            message.dz_gain = config["dz_gain"]
            message.px_gain = config["pr_gain"]
            message.ix_gain = config["ir_gain"]
            message.dx_gain = config["dr_gain"]
            message.py_gain = config["pp_gain"]
            message.iy_gain = config["ip_gain"]
            message.dy_gain = config["dp_gain"]
            message.pz_gain = config["pya_gain"]
            message.iz_gain = config["iya_gain"]
            message.dz_gain = config["dya_gain"]
            message.aw_gain = config["awx_gain"]
            message.aw_gain = config["awy_gain"]
            message.aw_gain = config["awz_gain"]
            message.aw_gain = config["awr_gain"]
            message.aw_gain = config["awp_gain"]
            message.aw_gain = config["awya_gain"]
            message.integral_lower_limit = config["integral_lower_limit"]
            message.integral_upper_limit = config["integral_upper_limit"]
            message.integrator_active = config["integrator_active"]
            message.controller_active = config["controller_active"]
            message.mode = config["mode"]
            message.setpoint1 = config["setpoint1"]
            message.setpoint2 = config["setpoint2"]
            message.frequency = config["frequency"]
            message.osc_point = config["osc_point"]
            message.amplitude = config["amplitude"]
            message.window = config["window"]
            message.ready = config["ready"]
            self.conf_pub.publish(message)
        return config


def main():
    ConfigurableNode()


if __name__ == "__main__":
    main()
