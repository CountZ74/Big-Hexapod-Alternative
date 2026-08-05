"""Tests für die Hexapod-Klasse."""

from __future__ import annotations

import math

import pytest

from hexapod.config import CameraAxis, Joint
from hexapod.drivers.simulator import SimulatorDriver
from hexapod.kinematics import forward_kinematics
from hexapod.robot import Hexapod
from hexapod.servo_mapper import OutOfRangeError
from hexapod.servo_mapper.mapper import MAX_ANGLE_RAD


class TestLifecycle:
    def test_has_six_legs(self, sim_hexapod: Hexapod) -> None:
        assert len(sim_hexapod.leg_names) == 6

    def test_leg_names_correct(self, sim_hexapod: Hexapod) -> None:
        assert sim_hexapod.leg_names == [
            "front_right", "front_left",
            "mid_right", "mid_left",
            "back_right", "back_left",
        ]

    def test_context_manager_closes(self, sim_hexapod: Hexapod) -> None:
        with sim_hexapod as robot:
            assert not any(d.is_closed for d in robot.drivers.values())
        assert all(d.is_closed for d in robot.drivers.values())

    def test_close_disables_all_servos(self, sim_hexapod: Hexapod, leg_bus: str) -> None:
        driver = sim_hexapod.bus_driver(leg_bus)
        assert isinstance(driver, SimulatorDriver)
        sim_hexapod.set_servo_us(leg_bus, 1, 1500.0)
        sim_hexapod.close()
        assert driver.snapshot()[1] == 0.0


class TestHome:
    def test_home_sets_all_channels(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.home()
        # Jeder Servo wird auf SEINEM Bus geprueft: Kanalnummern wiederholen
        # sich ueber Busse hinweg, eine flache Tabelle waere mehrdeutig.
        snaps = {b: d.snapshot() for b, d in sim_hexapod.drivers.items()}
        for servo in sim_hexapod.config.servos:
            if servo.kind != "leg":
                continue  # home() bewegt nur die Beine
            got = snaps[servo.bus][servo.channel]
            assert got == pytest.approx(servo.center_us), \
                f"{servo.bus}/{servo.channel} falsch"

    def test_home_updates_leg_state(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.home()
        for name in sim_hexapod.leg_names:
            state = sim_hexapod.get_leg_state(name)
            assert state.theta1 == pytest.approx(0.0)
            assert state.theta2 == pytest.approx(0.0)
            assert state.theta3 == pytest.approx(0.0)


class TestSetLegAngles:
    def test_zero_angles_give_center_us(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.set_leg_angles("front_right", 0.0, 0.0, 0.0)
        for joint in [Joint.COXA, Joint.FEMUR, Joint.TIBIA]:
            servo = sim_hexapod.config.get_leg_servo("front_right", joint)
            driver = sim_hexapod.bus_driver(servo.bus)
            assert isinstance(driver, SimulatorDriver)
            assert driver.snapshot()[servo.channel] == pytest.approx(servo.center_us)

    def test_other_legs_unaffected(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.set_leg_angles("front_right", 0.0, 0.0, 0.0)
        # Alle Servos, die nicht zu front_right gehoeren, sind unberuehrt (0.0).
        snaps = {b: d.snapshot() for b, d in sim_hexapod.drivers.items()}
        for servo in sim_hexapod.config.servos:
            if getattr(servo, "leg", None) == "front_right":
                continue
            assert snaps[servo.bus][servo.channel] == 0.0, \
                f"{servo.bus}/{servo.channel} haette unberuehrt bleiben muessen"

    def test_updates_leg_state_angles(self, sim_hexapod: Hexapod) -> None:
        t1, t2, t3 = math.radians(10), math.radians(20), math.radians(30)
        sim_hexapod.set_leg_angles("mid_right", t1, t2, t3)
        state = sim_hexapod.get_leg_state("mid_right")
        assert state.theta1 == pytest.approx(t1)
        assert state.theta2 == pytest.approx(t2)
        assert state.theta3 == pytest.approx(t3)

    def test_updates_leg_state_position_via_fk(self, sim_hexapod: Hexapod) -> None:
        t1, t2, t3 = math.radians(5), math.radians(30), math.radians(40)
        sim_hexapod.set_leg_angles("back_left", t1, t2, t3)
        state = sim_hexapod.get_leg_state("back_left")
        ex, ey, ez = forward_kinematics(t1, t2, t3, sim_hexapod.leg_lengths)
        assert state.foot_x == pytest.approx(ex)
        assert state.foot_y == pytest.approx(ey)
        assert state.foot_z == pytest.approx(ez)

    def test_rejects_unknown_leg(self, sim_hexapod: Hexapod) -> None:
        with pytest.raises(KeyError):
            sim_hexapod.set_leg_angles("no_such_leg", 0.0, 0.0, 0.0)

    def test_rejects_out_of_range_angle(self, sim_hexapod: Hexapod) -> None:
        with pytest.raises(OutOfRangeError):
            sim_hexapod.set_leg_angles("front_right", 0.0, math.radians(150), 0.0)

    def test_clip_mode_does_not_raise(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.set_leg_angles("front_right", 0.0, math.radians(150), 0.0, clip=True)


class TestSetFootPosition:
    def test_reachable_point(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.set_foot_position("front_right", 160.0, 0.0, -20.0)
        state = sim_hexapod.get_leg_state("front_right")
        assert state.foot_x == pytest.approx(160.0, abs=1e-6)
        assert state.foot_y == pytest.approx(0.0, abs=1e-6)
        assert state.foot_z == pytest.approx(-20.0, abs=1e-6)

    def test_state_foot_matches_target(self, sim_hexapod: Hexapod) -> None:
        x, y, z = 170.0, 0.0, -10.0
        sim_hexapod.set_foot_position("mid_left", x, y, z)
        state = sim_hexapod.get_leg_state("mid_left")
        assert state.foot_x == pytest.approx(x, abs=1e-6)
        assert state.foot_y == pytest.approx(y, abs=1e-6)
        assert state.foot_z == pytest.approx(z, abs=1e-6)

    def test_unreachable_point_raises(self, sim_hexapod: Hexapod) -> None:
        from hexapod.kinematics import UnreachableError
        with pytest.raises(UnreachableError):
            sim_hexapod.set_foot_position("front_right", 999.0, 0.0, 0.0)


class TestSetAllFootPositions:
    def test_sets_multiple_legs(self, sim_hexapod: Hexapod) -> None:
        targets = {
            "front_right": (160.0, 0.0, -20.0),
            "front_left":  (160.0, 0.0, -20.0),
        }
        sim_hexapod.set_all_foot_positions(targets, clip=True)
        for name in ["front_right", "front_left"]:
            state = sim_hexapod.get_leg_state(name)
            assert state.foot_x == pytest.approx(160.0, abs=1e-6)
            assert state.foot_z == pytest.approx(-20.0, abs=1e-6)

    def test_unspecified_legs_unaffected(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.home()
        targets = {"front_right": (160.0, 0.0, -20.0)}
        sim_hexapod.set_all_foot_positions(targets, clip=True)
        state = sim_hexapod.get_leg_state("mid_right")
        assert state.theta1 == pytest.approx(0.0)
        assert state.theta2 == pytest.approx(0.0)
        assert state.theta3 == pytest.approx(0.0)


class TestDirectServoControl:
    def test_set_and_get_servo_us(self, sim_hexapod: Hexapod, leg_bus: str) -> None:
        sim_hexapod.set_servo_us(leg_bus, 1, 1600.0)
        assert sim_hexapod.get_servo_us(leg_bus, 1) == pytest.approx(1600.0)

    def test_unbekannter_bus_wirft(self, sim_hexapod: Hexapod) -> None:
        with pytest.raises(KeyError, match="gibt_es_nicht"):
            sim_hexapod.set_servo_us("gibt_es_nicht", 0, 1500.0)

    def test_disable_all(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.home()
        sim_hexapod.disable_all()
        snaps = {b: d.snapshot() for b, d in sim_hexapod.drivers.items()}
        for servo in sim_hexapod.config.servos:
            got = snaps[servo.bus][servo.channel]
            assert got == 0.0, f"{servo.bus}/{servo.channel} sollte 0 sein"


class TestCamera:
    def test_set_camera_pan_tilt(self, sim_hexapod: Hexapod) -> None:
        # Kamera-Servos haengen am eigenen Bus (PCA9685), nicht am Maestro.
        driver = sim_hexapod.bus_driver(
            sim_hexapod.config.get_camera_servo(CameraAxis.PAN).bus
        )
        assert isinstance(driver, SimulatorDriver)
        sim_hexapod.set_camera(pan_deg=0.0, tilt_deg=0.0)
        snap = driver.snapshot()
        pan = sim_hexapod.config.get_camera_servo(CameraAxis.PAN)
        tilt = sim_hexapod.config.get_camera_servo(CameraAxis.TILT)
        assert snap[pan.channel] == pytest.approx(pan.center_us)
        assert snap[tilt.channel] == pytest.approx(tilt.center_us)

    def test_camera_pan_positive(self, sim_hexapod: Hexapod) -> None:
        driver = sim_hexapod.bus_driver(
            sim_hexapod.config.get_camera_servo(CameraAxis.PAN).bus
        )
        assert isinstance(driver, SimulatorDriver)
        sim_hexapod.set_camera(pan_deg=45.0, tilt_deg=0.0)
        snap = driver.snapshot()
        pan = sim_hexapod.config.get_camera_servo(CameraAxis.PAN)
        expected = (
            pan.center_us
            + pan.direction * math.radians(45) * (pan.range_us / MAX_ANGLE_RAD)
        )
        assert snap[pan.channel] == pytest.approx(expected)


class TestLegState:
    def test_initial_state_zero(self, sim_hexapod: Hexapod) -> None:
        state = sim_hexapod.get_leg_state("front_right")
        assert state.foot_x == 0.0
        assert state.foot_y == 0.0
        assert state.foot_z == 0.0

    def test_unknown_leg_raises(self, sim_hexapod: Hexapod) -> None:
        with pytest.raises(KeyError):
            sim_hexapod.get_leg_state("fantasy_leg")


class TestBodyPose:
    def test_initial_pose_is_neutral(self, sim_hexapod: Hexapod) -> None:
        pose = sim_hexapod.body_pose
        assert pose.tx == 0.0
        assert pose.tz == 0.0
        assert pose.roll == 0.0

    def test_set_body_pose_updates_pose(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.set_body_pose(tz=15.0, roll=math.radians(5))
        assert sim_hexapod.body_pose.tz == pytest.approx(15.0)
        assert sim_hexapod.body_pose.roll == pytest.approx(math.radians(5))

    def test_elevation_lowers_all_feet_in_leg_frame(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.set_body_pose()
        z_neutral = {
            name: sim_hexapod.get_leg_state(name).foot_z
            for name in sim_hexapod.leg_names
        }
        sim_hexapod.set_body_pose(tz=15.0)
        for name in sim_hexapod.leg_names:
            z_elevated = sim_hexapod.get_leg_state(name).foot_z
            assert z_elevated == pytest.approx(z_neutral[name] - 15.0, abs=1e-4)

    def test_roll_breaks_z_symmetry(self, sim_hexapod: Hexapod) -> None:
        sim_hexapod.set_body_pose(roll=math.radians(10))
        fr = sim_hexapod.get_leg_state("front_right").foot_z
        fl = sim_hexapod.get_leg_state("front_left").foot_z
        assert fr != pytest.approx(fl, abs=1.0)

    def test_neutral_pose_matches_stance(self, sim_hexapod: Hexapod) -> None:
        # Neutrale Körperpose (alle Null) muss EXAKT der Standpose
        # entsprechen: Körperpose und Stance/Gait teilen einen Nullpunkt,
        # inklusive der per-Bein Z-Trims. (Früher prüfte dieser Test
        # Links/Rechts-Symmetrie — das galt nur, solange die Körperpose
        # die Z-Trims ignorierte.)
        sim_hexapod.stance(clip=True)
        stance_angles = {
            name: (
                sim_hexapod.get_leg_state(name).theta1,
                sim_hexapod.get_leg_state(name).theta2,
                sim_hexapod.get_leg_state(name).theta3,
            )
            for name in sim_hexapod.leg_names
        }
        sim_hexapod.set_body_pose(clip=True)
        for name in sim_hexapod.leg_names:
            st = sim_hexapod.get_leg_state(name)
            t1, t2, t3 = stance_angles[name]
            assert st.theta1 == pytest.approx(t1, abs=1e-9)
            assert st.theta2 == pytest.approx(t2, abs=1e-9)
            assert st.theta3 == pytest.approx(t3, abs=1e-9)

    def test_foot_positions_world_property(self, sim_hexapod: Hexapod) -> None:
        positions = sim_hexapod.foot_positions_world
        assert set(positions.keys()) == set(sim_hexapod.leg_names)


class TestMoveToBodyPose:
    def test_move_to_body_pose_reaches_target(self, sim_hexapod: Hexapod) -> None:
        from hexapod.kinematics import BodyPose
        from hexapod.gait.posture import move_to_body_pose

        target = BodyPose(tz=15.0, roll=math.radians(5.0))
        move_to_body_pose(sim_hexapod, target, steps=4, rate_hz=2000.0)
        ramped = {
            name: (
                sim_hexapod.get_leg_state(name).theta1,
                sim_hexapod.get_leg_state(name).theta2,
                sim_hexapod.get_leg_state(name).theta3,
            )
            for name in sim_hexapod.leg_names
        }
        # Direktes (hartes) set_body_pose auf dasselbe Ziel muss identisch sein
        sim_hexapod.set_body_pose(tz=15.0, roll=math.radians(5.0))
        for name in sim_hexapod.leg_names:
            st = sim_hexapod.get_leg_state(name)
            t1, t2, t3 = ramped[name]
            assert st.theta1 == pytest.approx(t1, abs=1e-9)
            assert st.theta2 == pytest.approx(t2, abs=1e-9)
            assert st.theta3 == pytest.approx(t3, abs=1e-9)
        assert sim_hexapod.body_pose.tz == pytest.approx(15.0)
