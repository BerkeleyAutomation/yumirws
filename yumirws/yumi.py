import abb_librws as abb
import numpy as np
from autolab_core import RigidTransform
from yumirws.constants import (
    M_TO_MM,
    SPEEDDATA_CONSTANTS,
    ZONEDATA_CONSTANTS,
    MM_TO_M,
    WAYPOINTS,
)
import time

"""
desired new capabilities: 
moveL with relative command (check old interface for implementation)
set absolute speeds, would be nice for move functions to accept a tuple of (mm/s,rad/s) 
set tool center point (6DOF offset from autolab_core RigidTransform)
move grippers to position (give a target value in meters)
get pose from the yumi FK
"""


class YuMi(object):
    def __init__(
        self,
        l_tcp=RigidTransform(),
        r_tcp=RigidTransform(),
        ip_address="192.168.125.1",
    ):
        try:
            self._iface = abb.RWSStateMachineInterface(ip_address)
        except RuntimeError:
            print("YuMi could not connect!")
        r_task, l_task = self._iface.rapid_tasks
        self.left = YuMiArm(self._iface, l_task.name, l_tcp)
        self.right = YuMiArm(self._iface, r_task.name, r_tcp)

    @property
    def auto_mode(self):
        return self._iface.auto_mode

    @property
    def connected(self):
        return self._iface.runtime_info.rws_connected

    @property
    def motors_on(self):
        return self._iface.motors_on

    @motors_on.setter
    def motors_on(self, value):
        self._iface.set_motors_on() if value else self._iface.set_motors_off()

    @property
    def rapid_running(self):
        return self._iface.rapid_running

    @property
    def rw_version(self):
        return self._iface.system_info.robot_ware_version

    @property
    def speed_ratio(self):
        return self._iface.get_speed_ratio()

    @speed_ratio.setter
    def speed_ratio(self, value):
        self._iface.set_speed_ratio(value)

    @property
    def suction_on(self):
        return self._iface.get_digital_signal("custom_DO_0")

    @suction_on.setter
    def suction_on(self, value):
        self._iface.set_digital_signal("custom_DO_0", value)

    @property
    def system_name(self):
        return self._iface.system_info.system_name

    @property
    def system_options(self):
        return self._iface.system_info.system_options

    def get_analog_signal(self, name):
        return self._iface.get_analog_signal(name)

    def get_digital_signal(self, name):
        return self._iface.get_digital_signal(name)

    def log_text(self, verbose=False):
        return self._iface.log_text(verbose)

    def log_text_latest(self):
        return self._iface.log_text_latest()

    def set_analog_signal(self, name, value):
        self._iface.set_analog_signal(name, value)

    def set_digital_signal(self, name, value):
        self._iface.set_digital_signal(name, value)

    def start_rapid(self):
        self._iface.start_rapid()

    def stop_rapid(self):
        self._iface.stop_rapid()

    def reset_program_pointer(self):
        self._iface.reset_program_pointer()


class YuMiArm(object):
    def __init__(self, iface, task, tcp=RigidTransform()):
        self._iface = iface
        self._task = task
        self._tcp = tcp

        self._side = "left" if self._task.lower()[-1] == "l" else "right"
        self._custom_mod = abb.FileResource(f"custom_{self._task.lower()}.sys")
        self._custom_mod_path = f"HOME:/{self._custom_mod.filename}"

        try:
            self.initialize_gripper()
            self._wait_for_cmd()
        except RuntimeError:
            print(f"Failed to initialize {self._side} gripper!")

    @property
    def tcp(self):
        return self._tcp
    
    @tcp.setter
    def tcp(self, value):
        self._tcp = value

    @property
    def tool_str(self):
        """
        returns the tooltip string for the current TCP
        """
        t = (self._tcp.translation * M_TO_MM).astype(str)
        q = self._tcp.quaternion.astype(str)
        return f"[TRUE,[[{','.join(t)}],[{','.join(q)}]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]]"

    def get_joints(self):
        jt = self._iface.mechanical_unit_joint_target(self._task[2:])
        return np.deg2rad(
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

    def calibrate_gripper(self):
        self._gripper_fn("calibrate")()

    def initialize_gripper(self):
        self._gripper_fn("initialize")()

    def open_gripper(self):
        self._gripper_fn("grip_out")()

    def close_gripper(self):
        self._gripper_fn("grip_in")()

    def move_gripper(self, value):
        self._gripper_fn("move_to")(M_TO_MM * value)

    @property
    def gripper_settings(self):
        return self._gripper_fn("get_settings")()

    @gripper_settings.setter
    def gripper_settings(self, value):
        self._gripper_fn("set_settings")(value)

    def move_joints_traj(
        self, joints, speed=(300, 360), zone="z1", final_zone="fine"
    ):
        joints = np.rad2deg(joints)
        if isinstance(speed, str) and speed in SPEEDDATA_CONSTANTS:
            speed = np.repeat(speed, len(joints))
        elif isinstance(speed, (np.ndarray, list, tuple)):
            speed = np.broadcast_to(speed, (len(joints), 2))
        else:
            raise ValueError(
                "Speed must either be a single string or a (2,) or (n,2) iterable"
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
        toolstr = f"PERS tooldata curtool := {self.tool_str};"
        for wp, sd, zd in zip(joints[:-1], speed[:-1], zone[:-1]):
            jt = abb.JointTarget(
                abb.RobJoint(np.append(wp[:2], wp[3:])),
                abb.ExtJoint(eax_a=wp[2]),
            )
            sd = (
                sd
                if isinstance(sd, str)
                else abb.SpeedData((sd[0], sd[1], 5000, 5000))
            )
            zd = zd if isinstance(zd, str) else abb.ZoneData(zd)
            wpstr += f"\t\tMoveAbsJ {jt}, {sd}, {zd}, curtool;\n"
        jt = abb.JointTarget(
            abb.RobJoint(np.append(joints[-1, :2], joints[-1, 3:])),
            abb.ExtJoint(eax_a=joints[-1, 2]),
        )
        sd = (
            speed[-1]
            if isinstance(speed[-1], str)
            else abb.SpeedData((speed[-1][0], speed[-1][1], 5000, 5000))
        )
        wpstr += f"\t\tMoveAbsJ {jt}, {sd}, {final_zone}, curtool;"
        routine = f"MODULE customModule\n\t{toolstr}\n\tPROC custom_routine0()\n{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

    def read_test_pose(self, joints):
        mod_name = "customModule"
        jt = abb.JointTarget(
            abb.RobJoint(np.append(joints[:2], joints[3:])),
            abb.ExtJoint(eax_a=joints[2]),
        )
        wpstr = f"\t\tresPose := CalcRobT({jt}, tool0 \WObj:=wobj0);"
        routine = f"MODULE customModule\n\tVAR robtarget resPose;\n\tPROC custom_routine0()\n{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)
        pose = self._iface.get_rapid_symbol_data(
            self._task, mod_name, "resPose"
        )
        return pose

    def get_pose(self):
        rt = self._iface.mechanical_unit_rob_target(
            self._task[2:], abb.Coordinate.BASE, "tool0", "wobj0"
        )
        trans = MM_TO_M * np.array(
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
            from_frame=self._tcp.to_frame,
            to_frame="base_link",
        )
        return wrist * self._tcp

    def goto_pose(self, pose, speed=(300, 360), zone="fine", linear=True):
        rt = self._iface.mechanical_unit_rob_target(
            self._task[2:], abb.Coordinate.BASE, "tool0", "wobj0"
        )
        trans = pose.translation * M_TO_MM
        rt.pos = abb.Pos(trans)
        rt.orient = abb.Orient(*pose.quaternion)
        toolstr = f"\n\tPERS tooldata custom_tool := {self.tool_str};\n\tVAR robtarget p1 := {rt};"
        sd = (
            speed
            if isinstance(speed, str)
            else abb.SpeedData((speed[0], speed[1], 2000, 2000))
        )
        cmd = "MoveL" if linear else "MoveJ"
        wpstr = f"\t\t{cmd} p1, {sd}, {zone}, custom_tool;"
        routine = f"MODULE customModule{toolstr}\n\tPROC custom_routine0()\n{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

    def _gripper_fn(self, fn_name):
        return getattr(self._iface.services().sg(), f"{self._side}_{fn_name}")

    def _execute_custom(self, routine):
        # Upload and execute custom routine (unloading needed for new routine)
        self._iface.services().rapid().run_module_unload(
            self._task, self._custom_mod_path
        )
        self._wait_for_cmd()
        self._iface.upload_file(self._custom_mod, routine)
        time.sleep(0.01)
        self._iface.services().rapid().run_module_load(
            self._task, self._custom_mod_path
        )
        self._wait_for_cmd()
        self._iface.services().rapid().run_call_by_var(
            self._task, "custom_routine", 0
        )
        self._wait_for_cmd()

    def _wait_for_cmd(self):
        while not self._iface.services().main().is_idle(self._task):
            pass


if __name__ == "__main__":
    from yumiplanning.yumi_kinematics import YuMiKinematics as YK

    L_TCP = RigidTransform(
        translation=[0, 0, 0.11], from_frame="l_tcp", to_frame="wrist"
    )
    GRIP_DOWN_R = np.diag(
        [1, -1, -1]
    )  # orientation where the gripper is facing downwards
    y = YuMi(l_tcp=L_TCP)
    new_waypoints = []
    for w in WAYPOINTS:
        new_waypoints.append(YK.yumi_order_2_urdf(w))
    y.left_gripper_close()
    y.left.move_joints_traj([np.rad2deg(YK.L_NICE_STATE)])
    y.left.goto_pose(
        RigidTransform(
            translation=[0.4, 0.1, 0.1],
            rotation=GRIP_DOWN_R,
            from_frame="l_tcp",
        ),
        zone="fine",
        speed=(100, 360),
        linear=True,
    )
    y.left.goto_pose(
        RigidTransform(
            translation=[0.4, -0.1, 0.1],
            rotation=GRIP_DOWN_R,
            from_frame="l_tcp",
        ),
        speed=(500, 360),
    )
    # y.left.move_joints_traj(np.array(new_waypoints),(500,180,2000,2000))
    """
    p=y.right.read_test_pose(np.zeros(7))
    print(f"zero pos",p)
    for j in range(7):
        for diff in [-10,10]:
            js=np.zeros(7)
            js[j]=diff
            p=y.right.read_test_pose(js)
            print(f"joint {j} at {diff} deg",p)
    """
