import numpy as np

from yumirws import YuMi
from yumirws.constants import WAYPOINTS
from yumiplanning.yumi_kinematics import YuMiKinematics as YK

y = YuMi()
new_waypoints = []
new_waypoints_r = []
for w in WAYPOINTS:
    new_waypoints.append(np.deg2rad(YK.yumi_order_2_urdf(w)))
    w_r = w.copy()
    w_r[0] *= -1
    w_r[3] *= -1
    w_r[6] *= -1
    new_waypoints_r.append(np.deg2rad(YK.yumi_order_2_urdf(w_r)))
y.left.open_gripper()
y.right.open_gripper()
y.left.move_joints_traj(np.array(new_waypoints))
y.right.move_joints_traj(np.array(new_waypoints_r))
y.right.close_gripper()
y.left.close_gripper()
y.left.sync()
y.right.sync()
print("After sync")
if(y.left.clear_error()[0]):print("Error in left")
if(y.right.clear_error()[0]):print("Error in right")
print("ending")
# y.left.goto_pose(
#     RigidTransform(
#         translation=[0.4, 0.1, 0.1],
#         rotation=GRIP_DOWN_R,
#         from_frame="l_tcp",
#     ),
#     zone="fine",
#     speed=(100, 360),
#     linear=True,
# )
# y.left.goto_pose(
#     RigidTransform(
#         translation=[0.4, -0.1, 0.1],
#         rotation=GRIP_DOWN_R,
#         from_frame="l_tcp",
#     ),
#     speed=(500, 360),
# )
