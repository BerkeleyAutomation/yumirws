import numpy as np
from yumirws import YuMi
from yumiplanning.yumi_kinematics import YuMiKinematics as YK
from autolab_core import RigidTransform
from tqdm import tqdm
def get_arm_rigid_trans(arm,j:np.ndarray):
        rt=arm.get_fk(j)
        trans = .001 * np.array(
                [
                    rt.pos.x.value,
                    rt.pos.y.value,
                    rt.pos.z.value,
                ]
            )
        q = np.array(
            [
                rt.orient.q1.value,
                rt.orient.q2.value,
                rt.orient.q3.value,
                rt.orient.q4.value,
            ]
        )
        wrist = RigidTransform(
            translation=trans,
            rotation=RigidTransform.rotation_from_quaternion(q),
            from_frame='wrist',
            to_frame="base_link")
        return wrist

def collect_random_points():
    y = YuMi()
    yk = YK()
    armname='right'
    if armname=='left':
        arm = y.left
    else:
        arm = y.right
    low=yk.left_joint_lims[0]
    high=yk.left_joint_lims[1]
    N=1000
    poses=np.zeros((N,4,4))
    joints=np.zeros((N,7))
    for i in tqdm(range(N)):
        j=np.random.uniform(low,high)
        wrist=get_arm_rigid_trans(arm,j)
        joints[i,:]=j
        poses[i,:,:]=wrist.matrix
    np.save(f'{armname}_joints_random.npy',joints)
    np.save(f'{armname}_poses_random.npy',poses)

def collect_grid_points():
    y = YuMi()
    yk = YK()
    armname='left'
    if armname=='left':
        arm = y.left
    else:
        arm = y.right
    low=yk.left_joint_lims[0]
    high=yk.left_joint_lims[1]
    joint_N = 150
    joint_data = np.zeros((joint_N*7,7))
    pose_data = np.zeros((joint_N*7,4,4))
    for j in range(7):
        print(f"Collecting joint {j}")
        jvals = np.linspace(low[j],high[j],joint_N)
        for n in tqdm(range(joint_N)):
            jval = jvals[n]
            joints = np.zeros(7)
            joints[j]=jval
            wrist=get_arm_rigid_trans(arm,joints)
            ind = j*joint_N+n
            joint_data[ind,:]=joints
            pose_data[ind,:,:]=wrist.matrix
    np.save(f"{armname}_joints_individual.npy",joint_data)
    np.save(f"{armname}_poses_individual.npy",pose_data)


collect_grid_points()