import json
import multiprocessing as mp
import numpy as np
import queue as Queue
import time

import abb_librws as abb
from autolab_core import RigidTransform

from .constants import (
    M_TO_MM,
    MM_TO_M,
    SPEEDDATA_CONSTANTS,
    ZONEDATA_CONSTANTS,
    SLEEP_TIME,
)

# TODO add sync option for all motions
# TODO exception handling of motion supervision


def cmd(cmd, args, tries=10):
    """
    This function tries to run the command until success for 'tries' number of times.
    This is useful for RWS commands which occasionally return Timeout
    """
    for i in range(tries):
        try:
            return cmd(*args)
        except RuntimeError:
            print(f"yumi.py: retrying cmd {cmd}")
            time.sleep(0.03)
    raise RuntimeError(f"Couldn't execute command {cmd}")


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
        self._lock = mp.Lock()
        cmd(self.stop_rapid, ())
        cmd(self.reset_program_pointer, ())
        cmd(self.start_rapid, ())
        self.left = YuMiArm(self._lock, ip_address, l_task.name, l_tcp)
        self.left.daemon = True
        self.left.start()
        self.right = YuMiArm(self._lock, ip_address, r_task.name, r_tcp)
        self.right.daemon = True
        self.right.start()

    @property
    def auto_mode(self):
        with self._lock:
            return self._iface.auto_mode

    @property
    def connected(self):
        with self._lock:
            return self._iface.runtime_info.rws_connected

    @property
    def motors_on(self):
        with self._lock:
            return self._iface.motors_on

    @motors_on.setter
    def motors_on(self, value):
        with self._lock:
            self._iface.set_motors_on() if value else self._iface.set_motors_off()

    @property
    def rapid_running(self):
        with self._lock:
            return self._iface.rapid_running

    @property
    def rw_version(self):
        with self._lock:
            return self._iface.system_info.robot_ware_version

    @property
    def speed_ratio(self):
        with self._lock:
            return self._iface.get_speed_ratio()

    @speed_ratio.setter
    def speed_ratio(self, value):
        with self._lock:
            self._iface.set_speed_ratio(value)

    @property
    def suction_on(self):
        with self._lock:
            return self._iface.get_digital_signal("custom_DO_0")

    @suction_on.setter
    def suction_on(self, value):
        with self._lock:
            self._iface.set_digital_signal("custom_DO_0", value)

    @property
    def system_name(self):
        with self._lock:
            return self._iface.system_info.system_name

    @property
    def system_options(self):
        with self._lock:
            return self._iface.system_info.system_options

    def get_analog_signal(self, name):
        with self._lock:
            return self._iface.get_analog_signal(name)

    def get_digital_signal(self, name):
        with self._lock:
            return self._iface.get_digital_signal(name)

    def log_text(self, verbose=False):
        with self._lock:
            return self._iface.log_text(verbose)

    def log_text_latest(self):
        with self._lock:
            return self._iface.log_text_latest()

    def set_analog_signal(self, name, value):
        with self._lock:
            self._iface.set_analog_signal(name, value)

    def set_digital_signal(self, name, value):
        with self._lock:
            self._iface.set_digital_signal(name, value)

    def start_rapid(self):
        if not self.motors_on:
            self.motors_on = True
            time.sleep(0.3)
        with self._lock:
            self._iface.start_rapid()
            time.sleep(0.1)

    def stop_rapid(self):
        with self._lock:
            self._iface.stop_rapid()
            time.sleep(0.1)

    def reset_program_pointer(self):
        with self._lock:
            self._iface.reset_program_pointer()
            time.sleep(0.1)

    def calibrate_grippers(self):
        self._gripper_fn("calibrate")
        time.sleep(5)

    def move_grippers(self, lpos, rpos):
        self._gripper_fn("move_to", lpos * M_TO_MM, rpos * M_TO_MM)

    def close_grippers(self):
        self._gripper_fn("grip_in")

    def open_grippers(self):
        self._gripper_fn("grip_out")

    def _gripper_fn(self, fn_name, *args):
        with self._lock:
            return getattr(self._iface.services().sg(), f"dual_{fn_name}")(*args)

    def move_joints_sync(self, l_joints, r_joints, speed=(0.3, 2 * np.pi), zone="z1", final_zone="fine"):
        if len(l_joints) != len(r_joints):
            raise Exception("Sync move must have equal joint traj lengths")
        self.left.q_add()
        self.right.q_add()
        self.left._input_queue.put(("_move_joints_sync", l_joints, speed, zone, final_zone))
        self.right._input_queue.put(("_move_joints_sync", r_joints, speed, zone, final_zone))


class YuMiArm(mp.Process):
    def __init__(self, lock, ip_address, task, tcp=RigidTransform()):
        super().__init__()
        self._input_queue = mp.Queue()
        self._q_len = mp.Value("i", 0)  # we need this to be a reliable counter for the q size
        self._iface = abb.RWSStateMachineInterface(ip_address)
        self._task = task
        self._tcp = tcp

        self._side = "left" if self._task.lower()[-1] == "l" else "right"
        self._custom_mod = abb.FileResource(f"custom_{self._task.lower()}.sys")
        self._custom_mod_path = f"HOME:/{self._custom_mod.filename}"
        self._lock = lock
        tooltip = f"""
MODULE {self._task}_tcp
\tTASK PERS tooldata tool{self._task.lower()} := {self.tool_str};
\tPERS tasks task_list{{2}} := [["T_ROB_L"],["T_ROB_R"]];
\tTASK PERS bool pending_move_err := FALSE;
\tTASK PERS errnum lasterr := 42;
ENDMODULE
"""
        tooltipmod = abb.FileResource(f"tooltip_{self._task.lower()}.sys")
        tooltippath = f"HOME:/{tooltipmod.filename}"
        time.sleep(SLEEP_TIME)
        with self._lock:
            self._wait_for_cmd()
            self._iface.services().rapid().run_module_unload(self._task, tooltippath)
        time.sleep(SLEEP_TIME)
        with self._lock:
            self._wait_for_cmd()
            self._iface.upload_file(tooltipmod, tooltip)
        time.sleep(SLEEP_TIME)
        with self._lock:
            self._wait_for_cmd()
            self._iface.services().rapid().run_module_load(self._task, tooltippath)
        time.sleep(SLEEP_TIME)

    def err_handler(self, indents):
        tab = indents * "\t"
        str = f"""
{tab}ERROR
{tab}\tlasterr := ERRNO;
{tab}\tpending_move_err := TRUE;
{tab}\tStopMoveReset;
"""
        return str

    def run(self):
        while True:
            try:
                request = self._input_queue.get(timeout=1)
            except Queue.Empty:
                continue
            # print(self._task,request[0])
            getattr(self, request[0])(*request[1:])
            self.q_dec()

    def sync(self):
        """
        blocks until queue is empty and current cmd is done
        queue empty() is not reliable when querying queue items,
        so instead we share a mp.Value which atomically gets updated when adding and
        removing items from the queue
        """
        while self._q_len.value > 0:
            pass

    def q_add(self):
        with self._q_len.get_lock():
            self._q_len.value += 1

    def q_dec(self):
        with self._q_len.get_lock():
            self._q_len.value -= 1

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
        try:
            with self._lock:
                jt = self._iface.mechanical_unit_joint_target(self._task[2:])
        except RuntimeError:
            return self.get_joints()
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

    def clear_error(self):
        with self._lock:
            err = cmd(self._iface.get_rapid_symbol_data, (self._task, f"{self._task.lower()}_tcp", "pending_move_err"))
            errnum = cmd(self._iface.get_rapid_symbol_data, (self._task, f"{self._task.lower()}_tcp", "lasterr"))
            if err == "TRUE":
                cmd(
                    self._iface.set_rapid_symbol_data,
                    (self._task, f"{self._task.lower()}_tcp", "pending_move_err", "FALSE"),
                )
        return err == "TRUE", errnum

    def calibrate_gripper(self):
        self.q_add()
        self._input_queue.put(("_gripper_fn", "calibrate"))

    def initialize_gripper(self):
        self.q_add()
        self._input_queue.put(("_gripper_fn", "initialize"))

    def open_gripper(self):
        self.q_add()
        self._input_queue.put(("_gripper_fn", "grip_out"))

    def close_gripper(self):
        self.q_add()
        self._input_queue.put(("_gripper_fn", "grip_in"))

    def move_gripper(self, value):
        self.q_add()
        self._input_queue.put(("_gripper_fn", "move_to", M_TO_MM * value))

    @property
    def gripper_settings(self):
        return self._gripper_fn("get_settings")

    @gripper_settings.setter
    def gripper_settings(self, value):
        self._gripper_fn("set_settings")(value)

    def get_fk(self, joints):
        self.sync()
        if joints.ndim == 1:
            joints = joints.reshape(1, -1)
        joints_deg = np.rad2deg(joints)

        varstr = ""
        calcstr = ""
        for i, jts_deg in enumerate(joints_deg):
            varstr += f"\tVAR robtarget outrt{i};\n"
            jt = abb.JointTarget(
                abb.RobJoint(np.append(jts_deg[:2], jts_deg[3:])),
                abb.ExtJoint(eax_a=jts_deg[2]),
            )
            calcstr += f"\t\toutrt{i}:=CalcRobT({jt}, tool{self._task.lower()});\n"
        routine = "MODULE customModule\n" f"{varstr}\tPROC custom_routine0()\n" f"{calcstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

        rts = []
        for i in range(len(joints_deg)):
            ret = json.loads(self._iface.get_rapid_symbol_data(self._task, "customModule", f"outrt{i}"))
            rts.append(abb.RobTarget(abb.Pos(ret[0]), abb.Orient(ret[1]), abb.ConfData(ret[2]), abb.ExtJoint(ret[3])))
        return rts if len(rts) > 1 else rts[0]

    def get_ik(self, rts):
        self.sync()
        varstr = ""
        calcstr = ""
        for i, rt in enumerate(rts):
            varstr += f"\tVAR jointtarget outjt{i};\n"
            calcstr += f"\t\toutjt{i}:=CalcJointT({rt}, tool{self._task.lower()});\n"
        routine = "MODULE customModule\n" f"{varstr}\tPROC custom_routine0()\n" f"{calcstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

        jts = np.zeros((len(rts), 7))
        for i in range(len(rts)):
            try:
                ret = json.loads(self._iface.get_rapid_symbol_data(self._task, "customModule", f"outjt{i}"))
                jts[i] = np.deg2rad(ret[0][:2] + ret[1][:1] + ret[0][2:])
            except RuntimeError:
                pass
        return jts if len(jts) > 1 else jts[0]

    def move_joints_traj(self, joints, speed=(0.3, 2 * np.pi), zone="z1", final_zone="fine"):
        self.q_add()
        self._input_queue.put(("_move_joints_traj", joints, speed, zone, final_zone))

    def _move_joints_sync(self, joints, speed, zone, final_zone):
        """
        Inputs:
            joints : (n, 7)
                NumPy array of joint configurations (in rad)
            speed (optional) : str or (2,) or (n, 2)
                Speed for motion either in string form (e.g., "v100") or
                in array form (m/s, rad/s)
            zone (optional) : str or (7,) or (n, 7)
                Zone data for each waypoint, either as a string (e.g., "z1")
                or in array form (finep, pzone_tcp, pzone_ori, pzone_eax,
                zone_ori, zone_leax, zone_reax)
            final_zone (optional) : str
                Zone data for final waypoint
        """
        if joints.ndim == 1:
            joints = joints.reshape(1, -1)
        joints = np.rad2deg(joints)
        if isinstance(speed, str) and speed in SPEEDDATA_CONSTANTS:
            speed = np.repeat(speed, len(joints))
        elif isinstance(speed, (np.ndarray, list, tuple)):
            speed = np.broadcast_to(speed, (len(joints), 2))
        else:
            raise ValueError("Speed must either be a single string or a (2,) or (n,2) iterable")
        if isinstance(zone, str) and zone in ZONEDATA_CONSTANTS:
            zone = np.repeat(zone, len(joints))
        elif isinstance(zone, (np.ndarray, list, tuple)):
            zone = np.broadcast_to(zone, (len(joints), 7))
        else:
            raise ValueError("Zone must either be a single string or a (7,) or (n,7) iterable")

        # Create RAPID code and execute
        wpstr = "\t\tVAR syncident sync1;\n\t\tVAR syncident sync2;\n\t\tSyncMoveOn sync1, task_list;\n"
        for wp, sd, zd, id in zip(joints[:-1], speed[:-1], zone[:-1], range(len(joints))):
            jt = abb.JointTarget(
                abb.RobJoint(np.append(wp[:2], wp[3:])),
                abb.ExtJoint(eax_a=wp[2]),
            )
            sd = sd if isinstance(sd, str) else abb.SpeedData((sd[0] * M_TO_MM, np.rad2deg(sd[1]), 5000, 5000))
            zd = zd if isinstance(zd, str) else abb.ZoneData(zd)
            wpstr += f"\t\tMoveAbsJ {jt}, \ID:={id}, {sd}, {zd}, tool{self._task.lower()};\n"
        jt = abb.JointTarget(
            abb.RobJoint(np.append(joints[-1, :2], joints[-1, 3:])),
            abb.ExtJoint(eax_a=joints[-1, 2]),
        )
        sd = (
            speed[-1]
            if isinstance(speed[-1], str)
            else abb.SpeedData((speed[-1][0] * M_TO_MM, np.rad2deg(speed[-1][1]), 5000, 5000))
        )
        wpstr += f"\t\tMoveAbsJ {jt}, \ID:={len(joints)}, {sd}, {final_zone}, tool{self._task.lower()};\n"
        wpstr += f"\t\tSyncMoveOff sync2;"
        routine = f"MODULE customModule\n" "\tPROC custom_routine0()\n" f"{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

    def _move_joints_traj(self, joints, speed, zone, final_zone):
        """
        Inputs:
            joints : (n, 7)
                NumPy array of joint configurations (in rad)
            speed (optional) : str or (2,) or (n, 2)
                Speed for motion either in string form (e.g., "v100") or
                in array form (m/s, rad/s)
            zone (optional) : str or (7,) or (n, 7)
                Zone data for each waypoint, either as a string (e.g., "z1")
                or in array form (finep, pzone_tcp, pzone_ori, pzone_eax,
                zone_ori, zone_leax, zone_reax)
            final_zone (optional) : str
                Zone data for final waypoint
        """
        if joints.ndim == 1:
            joints = joints.reshape(1, -1)
        joints = np.rad2deg(joints)
        if isinstance(speed, str) and speed in SPEEDDATA_CONSTANTS:
            speed = np.repeat(speed, len(joints))
        elif isinstance(speed, (np.ndarray, list, tuple)):
            speed = np.broadcast_to(speed, (len(joints), 2))
        else:
            raise ValueError("Speed must either be a single string or a (2,) or (n,2) iterable")
        if isinstance(zone, str) and zone in ZONEDATA_CONSTANTS:
            zone = np.repeat(zone, len(joints))
        elif isinstance(zone, (np.ndarray, list, tuple)):
            zone = np.broadcast_to(zone, (len(joints), 7))
        else:
            raise ValueError("Zone must either be a single string or a (7,) or (n,7) iterable")

        # Create RAPID code and execute
        wpstr = ""
        for wp, sd, zd in zip(joints[:-1], speed[:-1], zone[:-1]):
            jt = abb.JointTarget(
                abb.RobJoint(np.append(wp[:2], wp[3:])),
                abb.ExtJoint(eax_a=wp[2]),
            )
            sd = sd if isinstance(sd, str) else abb.SpeedData((sd[0] * M_TO_MM, np.rad2deg(sd[1]), 5000, 5000))
            zd = zd if isinstance(zd, str) else abb.ZoneData(zd)
            wpstr += f"\t\tMoveAbsJ {jt}, {sd}, {zd}, tool{self._task.lower()};\n"
        jt = abb.JointTarget(
            abb.RobJoint(np.append(joints[-1, :2], joints[-1, 3:])),
            abb.ExtJoint(eax_a=joints[-1, 2]),
        )
        sd = (
            speed[-1]
            if isinstance(speed[-1], str)
            else abb.SpeedData((speed[-1][0] * M_TO_MM, np.rad2deg(speed[-1][1]), 5000, 5000))
        )
        wpstr += f"\t\tMoveAbsJ {jt}, {sd}, {final_zone}, tool{self._task.lower()};"
        routine = f"MODULE customModule\n" "\tPROC custom_routine0()\n" f"{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

    def get_pose(self):
        with self._lock:
            time.sleep(SLEEP_TIME)
            rt = cmd(self._iface.mechanical_unit_rob_target, (self._task[2:], abb.Coordinate.BASE, "tool0", "wobj0"))
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

    def goto_pose(self, pose, speed=(0.3, 2 * np.pi), zone="fine", linear=True):
        self.q_add()
        self._input_queue.put(("_goto_pose", pose, speed, zone, linear))

    def _goto_pose(self, pose, speed=(0.3, 2 * np.pi), zone="fine", linear=True):
        with self._lock:
            time.sleep(SLEEP_TIME)
            rt = self._iface.mechanical_unit_rob_target(self._task[2:], abb.Coordinate.BASE, "tool0", "wobj0")
        trans = pose.translation * M_TO_MM
        rt.pos = abb.Pos(trans)
        rt.orient = abb.Orient(*pose.quaternion)
        toolstr = f"\n\tVAR robtarget p1 := {rt};"
        sd = speed if isinstance(speed, str) else abb.SpeedData((speed[0] * M_TO_MM, np.rad2deg(speed[1]), 5000, 5000))
        cmd = "MoveL" if linear else "MoveJ"
        wpstr = f"\t\t{cmd} p1, {sd}, {zone}, tool{self._task.lower()};"
        routine = f"MODULE customModule{toolstr}\n\tPROC custom_routine0()\n{wpstr}\n\tENDPROC\nENDMODULE"
        self._execute_custom(routine)

    def _gripper_fn(self, fn_name, *args):
        with self._lock:
            res = getattr(self._iface.services().sg(), f"{self._side}_{fn_name}")(*args)
            time.sleep(SLEEP_TIME)
            return res

    def _execute_custom(self, routine):
        # Upload and execute custom routine (unloading needed for new routine)
        time.sleep(SLEEP_TIME)
        with self._lock:
            self._wait_for_cmd()
            self._iface.services().rapid().run_module_unload(self._task, self._custom_mod_path)
        time.sleep(SLEEP_TIME)
        with self._lock:
            self._wait_for_cmd()
            self._iface.upload_file(self._custom_mod, routine)
        time.sleep(SLEEP_TIME)
        with self._lock:
            self._wait_for_cmd()
            self._iface.services().rapid().run_module_load(self._task, self._custom_mod_path)
        time.sleep(SLEEP_TIME)
        with self._lock:
            self._wait_for_cmd()
            self._iface.services().rapid().run_call_by_var(self._task, "custom_routine", 0)
        time.sleep(SLEEP_TIME)
        self._wait_for_cmd_lock()

    def _wait_for_cmd(self):
        while True:
            bad = not cmd(self._iface.services().main().is_idle, (self._task,)) or not cmd(
                self._iface.services().main().is_stationary, (self._task[2:],)
            )
            if not bad:
                break

    def _wait_for_cmd_lock(self):
        while True:
            with self._lock:
                bad = not cmd(self._iface.services().main().is_idle, (self._task,)) or not cmd(
                    self._iface.services().main().is_stationary, (self._task[2:],)
                )
            if not bad:
                break
            time.sleep(SLEEP_TIME)
