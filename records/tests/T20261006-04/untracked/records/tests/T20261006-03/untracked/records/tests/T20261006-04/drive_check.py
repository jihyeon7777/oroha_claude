import sys, time, rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("drive_check")
pub = n.create_publisher(TwistStamped, "/diff_cont/cmd_vel", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
last = {}
n.create_subscription(JointState, "/joint_states", lambda m: last.update(dict(zip(m.name, m.velocity))), 10)
t0 = time.monotonic(); v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
while time.monotonic() - t0 < 2.0:               # 2 s at v, ends by itself
    m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg(); m.twist.linear.x = v
    pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05)
print("joint vel after 2 s at %.2f m/s:" % v, {k: round(x, 1) for k, x in last.items()})
for _ in range(10):
    m = TwistStamped(); m.header.stamp = n.get_clock().now().to_msg(); pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05)
