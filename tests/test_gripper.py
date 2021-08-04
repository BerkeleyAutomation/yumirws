from yumirws import YuMi

y = YuMi()
y.left.initialize_gripper()
y.left.close_gripper()

for _ in range(5):
    y.left.open_gripper()
    for _ in range(10):
        print(y._iface.services().main().is_stationary("ROB_L_7"))
    y.left.close_gripper()
import pdb; pdb.set_trace()