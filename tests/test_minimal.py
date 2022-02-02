import pytest
import time

from yumirws import YuMi


@pytest.fixture(scope="module")
def yumi():
    return YuMi()


def test_connection(yumi):
    assert yumi.connected
    print(f"Auto Mode: {yumi.auto_mode}")
    print(f"RW Version: {yumi.rw_version}")
    print(f"System Name: {yumi.system_name}")
    print(f"System Options: {yumi.system_options}")
    print(f"System Log: {yumi.log_text()}")
    print(f"Latest System Log: {yumi.log_text_latest()}")


def test_suction(yumi):
    print(f"Suction On: {yumi.suction_on}")
    print("Setting Suction On")
    yumi.suction_on = True
    time.sleep(0.5)
    assert yumi.suction_on
    print("Setting Suction Off")
    yumi.suction_on = False
    time.sleep(0.5)
    assert not yumi.suction_on


# TODO: This currently causes a crash
def test_motors(yumi):
    print(f"Motors On: {yumi.motors_on}")
    # print("Setting Motors Off")
    # yumi.motors_on = False
    # time.sleep(1)
    # assert not yumi.motors_on
    # print("Setting Motors On")
    # yumi.motors_on = True
    # time.sleep(1)
    # assert yumi.motors_on


# def test_rapid_stop_start(yumi):
#     print(f"RAPID Running: {yumi.rapid_running}")
#     print("Stopping RAPID")
#     yumi.stop_rapid()
#     time.sleep(3)
#     assert not yumi.rapid_running
#     print("Starting RAPID")
#     yumi.reset_program_pointer()
#     time.sleep(3)
#     yumi.start_rapid()
#     time.sleep(3)
#     assert yumi.rapid_running
