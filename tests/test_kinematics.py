import numpy as np

from yumirws import YuMi

y = YuMi()
joints = np.zeros((10, 7)) + 0.1 * np.random.randn(10, 7)
y.left.move_joints_traj(joints)
print(joints)
rts = y.left.get_fk(joints)
jts_ik = y.left.get_ik(rts)
print(jts_ik)
print(rts)
print(y.left.get_fk(jts_ik))
