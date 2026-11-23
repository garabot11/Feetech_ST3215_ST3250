import unittest
from struct import pack_into
from time import monotonic, sleep
from unittest.mock import patch

from feetech_panel.bus import (
    GOAL, LIMITS, LOCK, OPERATING_MODE, READ, SERVO_ID, SPEED_MODE, TORQUE, WRITE,
    BusError, Motion, NoResponse, ServoBus, frame,
)
from feetech_panel.demo import DemoPort


class CorruptPort(DemoPort):
    def write(self, packet):
        count = super().write(packet)
        if self.buffer and packet[2] == 2:
            self.buffer[-1] ^= 1
        return count


class IgnoreRenamePort(DemoPort):
    def write(self, packet):
        if packet[4] == WRITE and packet[5] == SERVO_ID:
            self.buffer.extend(frame(packet[2], 0))
            return len(packet)
        return super().write(packet)


class IgnoreHoldGoalPort(DemoPort):
    def write(self, packet):
        if packet[4] == WRITE and packet[5] == GOAL:
            self.buffer.extend(frame(packet[2], 0))
            return len(packet)
        return super().write(packet)


class BusTests(unittest.TestCase):
    def setUp(self):
        self.port = DemoPort(echo=True)
        self.bus = ServoBus(self.port, timeout=0.006)

    def test_frame_and_identification_with_echo(self):
        self.assertEqual(frame(1, 2, b"\x03\x03"), bytes.fromhex("ff ff 01 04 02 03 03 f2"))
        self.assertEqual(self.bus.identify(1).name, "ST3215")
        self.assertEqual(self.bus.identify(2).name, "ST3250")
        self.assertTrue(all(self.bus.feedback(servo_id).voltage == 12.0 for servo_id in (1, 2, 7)))
        with self.assertRaises(NoResponse):
            self.bus.identify(253)
        with self.assertRaises(BusError):
            self.bus.identify(254)

    def test_damaged_reply_is_not_treated_as_a_free_address(self):
        bus = ServoBus(CorruptPort(echo=True), timeout=0.006)
        with self.assertRaisesRegex(BusError, "повреждённый"):
            bus.change_id(1, 2)
        self.assertEqual(bus.port.devices[1].memory[LOCK], 1)

    def test_config_reads_limits_and_mode_without_a_long_block(self):
        class ShortConfigPort(DemoPort):
            def write(self, packet):
                if packet[4:6] == bytes((READ, LIMITS)) and packet[6] > 4:
                    return len(packet)
                return super().write(packet)

        port = ShortConfigPort()
        port.devices[7].memory[33] = 1
        config = ServoBus(port, timeout=0.006).config(7)
        self.assertEqual((config.minimum, config.maximum, config.resolution, config.mode), (1400, 3200, 1, 1))
        self.assertIn("Режим 1", config.position_problem)

    def test_hold_uses_current_position_and_confirms_motion_with_no_write_replies(self):
        servo = self.port.devices[1]
        servo.memory[8] = 0
        pack_into("<H", servo.memory, GOAL, 700)
        with self.assertRaisesRegex(BusError, "удержание"):
            self.bus.move(1, Motion(1200, 100, 10))
        self.bus.set_torque(1, True)
        initial = self.bus.feedback(1)
        self.assertEqual((initial.position, initial.goal, initial.torque), (1024, 1024, 1))

        self.bus.move(1, Motion(1240, 500, 10))
        first = self.bus.feedback(1)
        self.assertEqual(first.goal, 1240)
        self.assertLess(first.position, first.goal)
        deadline = monotonic() + 2
        while monotonic() < deadline and first.position != first.goal:
            sleep(0.01)
            first = self.bus.feedback(1)
        self.assertEqual(first.position, first.goal)
        self.bus.set_torque(1, False)
        self.assertEqual(self.bus.feedback(1).torque, 0)

    def test_firmware_may_require_an_explicit_torque_write(self):
        class ManualTorquePort(DemoPort):
            def write(self, packet):
                count = super().write(packet)
                if packet[4] == WRITE and packet[5] == GOAL:
                    self.devices[packet[2]].memory[TORQUE] = 0
                return count

        bus = ServoBus(ManualTorquePort(), timeout=0.006)
        bus.set_torque(1, True)
        self.assertEqual(bus.feedback(1).torque, 1)

    def test_hold_replaces_an_active_goal_with_the_current_position(self):
        servo = self.port.devices[1]
        pack_into("<H", servo.memory, GOAL, 600)
        servo.memory[TORQUE] = 1

        with patch("feetech_panel.demo.monotonic", return_value=servo.updated):
            self.bus.set_torque(1, True)
            feedback = self.bus.feedback(1)

        self.assertEqual((feedback.goal, feedback.torque), (1024, 1))

    def test_failed_hold_preserves_previous_torque(self):
        for torque in (0, 1):
            with self.subTest(torque=torque):
                port = IgnoreHoldGoalPort()
                servo = port.devices[1]
                pack_into("<H", servo.memory, GOAL, 600)
                servo.memory[TORQUE] = torque
                bus = ServoBus(port, timeout=0.006)

                with self.assertRaisesRegex(BusError, "не удалось подтвердить"):
                    bus.set_torque(1, True)

                self.assertEqual(bus.feedback(1).torque, torque)

    def test_limits_mode_and_status_are_checked_against_live_registers(self):
        self.bus.set_torque(7, True)
        with self.assertRaisesRegex(BusError, "1400–3200"):
            self.bus.move(7, Motion(1000, 30, 4))
        self.assertEqual(self.bus.feedback(7).goal, 2950)
        self.port.devices[7].memory[33] = 1
        with self.assertRaisesRegex(BusError, "однооборотный"):
            self.bus.move(7, Motion(2000, 30, 4))
        self.port.devices[7].memory[33] = 0
        self.port.devices[7].memory[65] = 4
        self.assertEqual(self.bus.feedback(7).status, 4)
        with self.assertRaisesRegex(BusError, "температура"):
            self.bus.move(7, Motion(2000, 30, 4))
        self.bus.set_torque(7, False)
        self.assertEqual(self.bus.feedback(7).torque, 0)

    def test_position_mode_is_verified_and_relocked_with_torque_off(self):
        class LockedModePort(DemoPort):
            def write(self, packet):
                if packet[4:6] == bytes((WRITE, OPERATING_MODE)) and self.devices[packet[2]].memory[LOCK]:
                    self.buffer.extend(frame(packet[2], 0))
                    return len(packet)
                return super().write(packet)

        port = LockedModePort()
        servo = port.devices[7]
        servo.memory[OPERATING_MODE] = SPEED_MODE
        servo.memory[TORQUE] = 1
        bus = ServoBus(port, timeout=0.006)
        with self.assertRaisesRegex(BusError, "Снимите удержание"):
            bus.set_position_mode(7)
        self.assertEqual(servo.memory[OPERATING_MODE], SPEED_MODE)
        self.assertEqual(servo.memory[LOCK], 1)

        bus.set_torque(7, False)
        config = bus.set_position_mode(7)
        self.assertTrue(config.single_turn)
        self.assertEqual((config.minimum, config.maximum), (1400, 3200))
        self.assertEqual(servo.memory[LOCK], 1)
        self.assertEqual(bus.feedback(7).torque, 0)

    def test_unconfirmed_position_mode_still_relocks(self):
        class IgnoreModePort(DemoPort):
            def write(self, packet):
                if packet[4:6] == bytes((WRITE, OPERATING_MODE)):
                    self.buffer.extend(frame(packet[2], 0))
                    return len(packet)
                return super().write(packet)

        port = IgnoreModePort()
        servo = port.devices[1]
        servo.memory[OPERATING_MODE] = SPEED_MODE
        with self.assertRaisesRegex(BusError, "не подтверждены"):
            ServoBus(port, timeout=0.006).set_position_mode(1)
        self.assertEqual(servo.memory[LOCK], 1)
        self.assertEqual(servo.memory[TORQUE], 0)

    def test_change_id_uses_unicast_with_readback_and_relocks(self):
        with self.assertRaisesRegex(BusError, "занят"):
            self.bus.change_id(1, 2)
        self.bus.set_torque(1, True)
        with self.assertRaisesRegex(BusError, "снимите удержание"):
            self.bus.change_id(1, 253)
        self.bus.set_torque(1, False)
        self.assertEqual(self.bus.change_id(1, 253).name, "ST3215")
        with self.assertRaises(NoResponse):
            self.bus.identify(1)
        self.assertEqual(self.port.devices[253].memory[LOCK], 1)
        self.assertEqual(self.bus.change_id(253, 0).servo_id, 0)
        self.assertEqual(self.port.devices[0].memory[LOCK], 1)
        self.assertEqual(self.bus.identify(2).name, "ST3250")

    def test_unsuccessful_rename_relocks_old_id(self):
        port = IgnoreRenamePort(echo=True)
        bus = ServoBus(port, timeout=0.006)
        with self.assertRaisesRegex(BusError, "Смена ID не подтверждена"):
            bus.change_id(1, 3)
        self.assertEqual(bus.identify(1).servo_id, 1)
        self.assertEqual(port.devices[1].memory[LOCK], 1)

    def test_motion_input_ranges(self):
        for values in ((-1, 20, 10), (4096, 20, 10), (100, 0, 10), (100, 20, 255)):
            with self.subTest(values=values), self.assertRaises(BusError):
                Motion(*values)
