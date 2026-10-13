"""Synthetic DDS test; run only in isolated ROS_DOMAIN_ID=221, no Agent/hardware."""
import math
import os
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
assert os.environ.get('ROS_DOMAIN_ID') == '221', 'Use isolated domain 221'
rclpy.init()
n = rclpy.create_node('bridge_fixture')
procs=[]
try:
    for exe,args in [('stm32_adapter',['frame_prefix:=bridge_test','transport_namespace:=/bridge_input']),
                     ('stm32_odometry',['namespace:=bridge_test','odometry.use_imu:=true','odometry.publish_tf:=true','odometry.frame_id:=odom','odometry.child_frame_id:=base_footprint'])]:
        command=['ros2','run','burger_stm32_bridge',exe,'--ros-args','-r','__ns:=/bridge_test']
        for arg in args: command+=['-p',arg]
        procs.append(subprocess.Popen(command,start_new_session=True))
    ip=n.create_publisher(Imu,'/bridge_input/imu/data',qos_profile_sensor_data)
    jp=n.create_publisher(JointState,'/bridge_input/joint_states',10)
    cp=n.create_publisher(TwistStamped,'/bridge_test/cmd_vel',10)
    odoms=[];imus=[];joints=[];cmds=[];transforms=[]
    subs=[n.create_subscription(Odometry,'/bridge_test/odom',odoms.append,10),
          n.create_subscription(Imu,'/bridge_test/imu',imus.append,10),
          n.create_subscription(JointState,'/bridge_test/joint_states',joints.append,10),
          n.create_subscription(Twist,'/bridge_input/cmd_vel',cmds.append,10),
          n.create_subscription(TFMessage,'/tf',transforms.append,10)]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=0.02)
    def feed(position,yaw,seconds=1.5,valid=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            stamp=n.get_clock().now().to_msg()
            im=Imu();im.header.stamp=stamp;im.header.frame_id='imu_link'
            im.orientation.w=math.cos(yaw/2);im.orientation.z=math.sin(yaw/2)
            im.orientation_covariance[0]=0.0 if valid else -1.0
            im.linear_acceleration.z=9.80665
            js=JointState();js.header.stamp=stamp
            js.name=['wheel_right_joint','wheel_left_joint']
            js.position=[position,position];js.velocity=[0.0,0.0]
            ip.publish(im);jp.publish(js);spin(.1)
    spin(2)
    feed(2.0,.8)
    assert odoms, 'No odometry; topic/QoS connection failed'
    assert abs(odoms[-1].pose.pose.position.x)<1e-6, 'First encoder must be a baseline'
    feed(3.0,1.0)
    out=odoms[-1]
    assert abs(out.pose.pose.position.x-.033*math.cos(.1))<1e-4,out
    assert abs(out.pose.pose.position.y-.033*math.sin(.1))<1e-4,out
    assert abs(out.pose.pose.orientation.z-math.sin(.1))<1e-4,out
    assert out.header.frame_id=='bridge_test/odom'
    assert out.child_frame_id=='bridge_test/base_footprint'
    assert imus[-1].header.frame_id=='bridge_test/imu_link'
    assert joints[-1].name==['bridge_test/wheel_left_joint','bridge_test/wheel_right_joint']
    assert transforms and transforms[-1].transforms[0].child_frame_id=='bridge_test/base_footprint'
    c=TwistStamped();c.header.stamp=n.get_clock().now().to_msg();c.twist.linear.x=.05;c.twist.angular.z=.2
    cp.publish(c);spin(.3);assert cmds and cmds[-1].linear.x==.05
    count=len(cmds);c.header.stamp.sec-=10;cp.publish(c);spin(.3);assert len(cmds)==count
    count=len(imus);feed(3.0,1.0,.5,False);assert len(imus)==count
    # Rebase after sensor gaps instead of integrating an unknown interval.
    spin(.6);feed(30.0,2.0)
    assert abs(odoms[-1].pose.pose.position.x-out.pose.pose.position.x)<1e-4
    client=n.create_client(Trigger,'/bridge_test/reset_odometry')
    assert client.wait_for_service(timeout_sec=2)
    future=client.call_async(Trigger.Request())
    spin(.3);assert future.done() and future.result().success
    feed(30.0,2.0)
    assert abs(odoms[-1].pose.pose.position.x)<1e-6
    assert abs(odoms[-1].pose.pose.orientation.z)<1e-6
    print('PASS: real DDS QoS, namespace, joint ordering, first baseline, wheel/yaw fusion, TF, cmd conversion/stale rejection, invalid IMU, gap and reset')
finally:
    for proc in procs:
        os.killpg(proc.pid,signal.SIGINT)
    for proc in procs:
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)
    n.destroy_node();rclpy.shutdown()
