from abb_librws import (
    RWSStateMachineInterface,
    FileResource,
    Pos,
    Orient,
    JointTarget,
    RobTarget,
    RobJoint,
    ExtJoint,
    SpeedData,
    Coordinate,
    ZoneData,
)
import numpy as np
from autolab_core import RigidTransform
from .constants import M_TO_MM, SPEEDDATA_CONSTANTS, ZONEDATA_CONSTANTS, MM_TO_M, WAYPOINTS

"""
desired new capabilities: 
moveL with relative command (check old interface for implementation)
set absolute speeds, would be nice for move functions to accept a tuple of (mm/s,rad/s) 
set tool center point (6DOF offset from autolab_core RigidTransform)
move grippers to position (give a target value in meters)
get pose from the yumi FK
"""


class YuMi(object):
    def __init__(self, l_tcp=RigidTransform(),r_tcp=RigidTransform(),ip_address="192.168.125.1"):
        try:
            self.iface = RWSStateMachineInterface(ip_address)
        except RuntimeError:
            print("YuMi could not connect!")
        r_task, l_task = self.iface.rapid_tasks
        self.left = YuMiArm(self.iface, l_task.name,l_tcp)
        self.right = YuMiArm(self.iface, r_task.name,r_tcp)
        try:
            self.iface.services().sg().left_initialize()
            self.left._wait_for_cmd()
        except RuntimeError:
            print("Failed to initialize left gripper!")
        try:
            self.iface.services().sg().right_initialize()
            self.right._wait_for_cmd()
        except RuntimeError:
            print("Failed to initialize right gripper!")

    @property
    def auto_mode(self):
        return self.iface.auto_mode

    @property
    def connected(self):
        return self.iface.runtime_info.rws_connected

    @property
    def motors_on(self):
        return self.iface.motors_on

    @motors_on.setter
    def motors_on(self, value):
        self.iface.set_motors_on() if value else self.iface.set_motors_off()

    @property
    def rapid_running(self):
        return self.iface.rapid_running

    @property
    def rw_version(self):
        return self.iface.system_info.robot_ware_version

    @property
    def speed_ratio(self):
        return self.iface.get_speed_ratio()

    @speed_ratio.setter
    def speed_ratio(self, value):
        self.iface.set_speed_ratio(value)

    @property
    def suction_on(self):
        return self.iface.get_digital_signal("custom_DO_0")

    @suction_on.setter
    def suction_on(self, value):
        self.iface.set_digital_signal("custom_DO_0", value)

    @property
    def system_name(self):
        return self.iface.system_info.system_name

    @property
    def system_options(self):
        return self.iface.system_info.system_options

    def get_analog_signal(self, name):
        return self.iface.get_analog_signal(name)

    def get_digital_signal(self, name):
        return self.iface.get_digital_signal(name)

    def left_gripper_open(self):
        self.iface.services().sg().left_grip_out()

    def left_gripper_close(self):
        self.iface.services().sg().left_grip_in()

    def log_text(self, verbose=False):
        return self.iface.log_text(verbose)

    def log_text_latest(self):
        return self.iface.log_text_latest()

    def right_gripper_open(self):
        self.iface.services().sg().right_grip_out()

    def right_gripper_close(self):
        self.iface.services().sg().right_grip_in()

    def set_analog_signal(self, name, value):
        self.iface.set_analog_signal(name, value)

    def set_digital_signal(self, name, value):
        self.iface.set_digital_signal(name, value)

    def start_rapid(self):
        self.iface.start_rapid()

    def stop_rapid(self):
        self.iface.stop_rapid()

    def reset_program_pointer(self):
        self.iface.reset_program_pointer()
    def set_tcp(self,left_tcp,right_tcp):
        self.left.set_tcp(left_tcp)
        self.right.set_tcp(right_tcp)


class YuMiArm(object):
    def __init__(self, iface, task, tcp):
        self.iface = iface
        self.task = task
        self._temp_mod = FileResource(f"custom_{self.task.lower()}.sys")
        self._temp_mod_path = f"HOME:/{self._temp_mod.filename}"
        self.iface.services().rapid().run_module_load(
            self.task, self._temp_mod_path
        )
        self._wait_for_cmd()
        self.set_tcp(tcp)

    def set_tcp(self,tcp):
        self.tcp=tcp

    def get_joints(self):
        jt = self.iface.mechanical_unit_joint_target(self.task[2:])
        return np.array(
            [
                jt.robax.rax_1.value,
                jt.robax.rax_2.value,
                jt.extax.eax_a.value,
                jt.robax.rax_3.value,
                jt.robax.rax_4.value,
                jt.robax.rax_5.value,
                jt.robax.rax_6.value,
            ]
        )

    def move_joints(self, joints, speed=None):
        if joints.shape != (7,):
            raise ValueError("Joints shape is not (7,)")
        if speed is not None:
            sd = SpeedData(speed)
            self.iface.services().rapid().set_move_speed(self.task, sd)
            self._wait_for_cmd()
        jt = JointTarget(RobJoint(np.append(joints[:2], joints[3:])), ExtJoint(joints[2]))
        self.iface.services().rapid().run_move_abs_j(self.task, jt)
        self._wait_for_cmd()

    def move_joints_traj(self, joints, speed="v100", zone="z1",final_zone='fine'):
        if isinstance(speed, str) and speed in SPEEDDATA_CONSTANTS:
            speed = np.repeat(speed, len(joints))
        elif isinstance(speed, (np.ndarray, list, tuple)):
            speed = np.broadcast_to(speed, (len(joints), 4))
        else:
            raise ValueError(
                "Speed must either be a single string or a (4,) or (n,4) iterable"
            )
        if isinstance(zone, str) and zone in ZONEDATA_CONSTANTS:
            zone = np.repeat(zone, len(joints))
        elif isinstance(zone, (np.ndarray, list, tuple)):
            zone = np.broadcast_to(zone, (len(joints), 7))
        else:
            raise ValueError(
                "Zone must either be a single string or a (7,) or (n,7) iterable"
            )

        # Create RAPID code and execute
        wpstr = ""
        toolstr = f"PERS tooldata custom_tool := {self.tool_str};" 
        for wp, sd, zd in zip(joints[:-1], speed[:-1], zone[:-1]):
            jt = JointTarget(RobJoint(np.append(wp[:2], wp[3:])), ExtJoint(eax_a=wp[2]))
            sd = sd if isinstance(sd, str) else SpeedData(sd)
            zd = zd if isinstance(zd, str) else ZoneData(zd)
            wpstr += f"\t\tMoveAbsJ {jt}, {sd}, {zd}, custom_tool;\n"
        jt = JointTarget(
                RobJoint(np.append(joints[-1, :2], joints[-1, 3:])), ExtJoint(eax_a=joints[-1, 2])
        )
        sd = speed[-1] if isinstance(speed[-1], str) else SpeedData(speed[-1])
        wpstr += f"\t\tMoveAbsJ {jt}, {sd}, {final_zone}, tool0;"
        routine = f"MODULE customModule\n\t{toolstr}\n\tPROC custom_routine0()\n{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

    @property
    def tool_str(self):
        '''
        returns the tooltip string for setting the tcp
        '''
        t=self.tcp.translation*M_TO_MM
        q=self.tcp.quaternion
        s=f"[TRUE,[[{t[0]},{t[1]},{t[2]}],[{q[0]}, {q[1]}, {q[2]} ,{q[3]}] ], [0.001,[0, 0, 0.001], [1, 0, 0, 0], 0, 0, 0] ]"
        return s

    def read_test_pose(self,joints):
        mod_name="customModule"
        jt = JointTarget(
                RobJoint(np.append(joints[:2], joints[3:])), ExtJoint(eax_a=joints[2])
        )
        wpstr = f"\t\tresPose := CalcRobT({jt}, tool0 \WObj:=wobj0);" 
        routine = f"MODULE customModule\n\tVAR robtarget resPose;\n\tPROC custom_routine0()\n{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)
        pose = self.iface.get_rapid_symbol_data(self.task,mod_name,"resPose")
        return pose

    def get_pose(self):
        rt = self.iface.mechanical_unit_rob_target(self.task[2:],Coordinate.BASE,"tool0","wobj0")
        trans=np.array([rt.pos.x.value*MM_TO_M,rt.pos.y.value*MM_TO_M,rt.pos.z.value*MM_TO_M])
        q=np.array([rt.orient.q1.value,rt.orient.q2.value,rt.orient.q3.value,rt.orient.q4.value])
        wrist = RigidTransform(translation=trans,rotation=RigidTransform.rotation_from_quaternion(q),
                    from_frame=self.tcp.to_frame,to_frame="base_link")
        return wrist*self.tcp
    
    def goto_pose(self,pose,speed=(150,360),zone='fine'):
        rt = self.iface.mechanical_unit_rob_target(self.task[2:],Coordinate.BASE,"tool0","wobj0")
        trans=pose.translation*M_TO_MM
        rt.pos=Pos(trans)
        rt.orient=Orient(*pose.quaternion)
        toolstr = f"PERS tooldata custom_tool := {self.tool_str};\n\tVAR robtarget p1 := {rt};" 
        sd = speed if isinstance(speed, str) else SpeedData((speed[0],speed[1],2000,2000))
        wpstr = f"\t\tMoveL p1, {sd}, {zone}, custom_tool;"
        routine = f"MODULE customModule\n\t{toolstr}\n\tPROC custom_routine0()\n{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

    def _execute_custom(self, routine):
        # Upload and execute custom routine (unloading needed for new routine)
        self._wait_for_cmd()
        try:
            self.iface.services().rapid().run_module_unload(
                self.task, self._temp_mod_path
            )
        except RuntimeError:
            pass
        self._wait_for_cmd()
        self.iface.upload_file(self._temp_mod, routine)
        self._wait_for_cmd()
        self.iface.services().rapid().run_module_load(
            self.task, self._temp_mod_path
        )
        self.iface.services().rapid().run_call_by_var(self.task, "custom_routine", 0)
        self._wait_for_cmd()

    def _wait_for_cmd(self):
        while not self.iface.services().main().is_idle(self.task):
            pass


if __name__ == "__main__":
    from yumiplanning.yumi_kinematics import YuMiKinematics as YK
    L_TCP = RigidTransform(translation=[0,0,.11],from_frame="l_tcp",to_frame="wrist")
    GRIP_DOWN_R = np.diag([1,-1,-1])#orientation where the gripper is facing downwards
    y = YuMi(l_tcp=L_TCP)
    new_waypoints=[]
    for w in WAYPOINTS:
        new_waypoints.append(YK.yumi_order_2_urdf(w))
    y.left_gripper_close()
    y.left.goto_pose(RigidTransform(translation=[ 0.27877562,  0.00155846, -0.00213494],rotation = GRIP_DOWN_R, from_frame="l_tcp"),zone='fine',speed=(100,360))
    #y.left.goto_pose(RigidTransform(translation=[.4,-.1,.1],rotation = GRIP_DOWN_R, from_frame="l_tcp"),speed=(500,360))
    #y.left.move_joints_traj(np.array(new_waypoints),(500,180,2000,2000))
    '''
    p=y.right.read_test_pose(np.zeros(7))
    print(f"zero pos",p)
    for j in range(7):
        for diff in [-10,10]:
            js=np.zeros(7)
            js[j]=diff
            p=y.right.read_test_pose(js)
            print(f"joint {j} at {diff} deg",p)
    '''
