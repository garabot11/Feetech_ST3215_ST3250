import importlib.util
import os
import select
import unittest
from queue import Empty, Queue
from threading import Event, Thread
from time import monotonic
from unittest.mock import patch

from feetech_panel.bus import LIMITS, OPERATING_MODE, READ, SPEED_MODE, Motion
from feetech_panel.demo import DemoPort, DemoServo
from feetech_panel.worker import BusWorker


class WorkerTests(unittest.TestCase):
    def wait_for(self, queue, condition, timeout=3):
        end = monotonic() + timeout
        received = []
        while monotonic() < end:
            try:
                event, args = queue.get(timeout=max(0, end - monotonic()))
            except Empty:
                break
            received.append(event)
            if event == "port_error":
                self.fail(f"Ошибка последовательного порта: {args}")
            if condition(event, args):
                return args
        self.fail(f"Событие не получено. Получены: {received}")

    def test_stopping_a_scan_leaves_the_connection_available(self):
        events = Queue()
        worker = BusWorker("", 1_000_000, events, demo=True)
        worker.submit("scan", 0, 253)
        worker.cancel_scan.set()
        worker.start()
        try:
            self.wait_for(events, lambda event, args: event == "message" and "Поиск остановлен" in args[0])
            self.wait_for(events, lambda event, args: event == "ready")
            worker.submit("scan", 1, 2)
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 2)
        finally:
            worker.stop_requested.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_polling_reaches_each_motor_on_a_larger_bus(self):
        events = Queue()
        demo = DemoPort()
        demo.devices = {servo_id: DemoServo(servo_id, 2825, 1000, 120) for servo_id in range(48)}
        worker = BusWorker("", 1_000_000, events, demo=True)
        with patch("feetech_panel.worker.DemoPort", return_value=demo):
            worker.start()
            try:
                worker.submit("scan", 0, 47)
                self.wait_for(events, lambda event, args: event == "message" and "Найдено: 48" in args[0])
                seen = set()

                def all_polled(event, args):
                    if event == "feedback":
                        seen.add(args[0])
                    return len(seen) == 48

                self.wait_for(events, all_polled, timeout=5)
            finally:
                worker.stop_requested.set()
                worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_polling_recovers_configuration_after_discovery_timeout(self):
        class DelayedConfigPort(DemoPort):
            drop_next_config = True

            def write(self, packet):
                if self.drop_next_config and packet[4:6] == bytes((READ, LIMITS)):
                    self.drop_next_config = False
                    return len(packet)
                return super().write(packet)

        events = Queue()
        demo = DelayedConfigPort()
        worker = BusWorker("", 1_000_000, events, demo=True)
        try:
            with patch("feetech_panel.worker.DemoPort", return_value=demo):
                worker.start()
                self.wait_for(events, lambda event, args: event == "connected")
            worker.submit("scan", 1, 1)
            self.wait_for(events, lambda event, args: event == "device" and args[1] is None)
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 1)
            _, config = self.wait_for(events, lambda event, args: event == "device" and args[1] is not None)
            self.assertTrue(config.single_turn)

            demo.devices[1].memory[65] = 1
            worker.submit("torque", (1,), True)
            message, error = self.wait_for(events, lambda event, args: event == "message")
            self.assertTrue(error)
            self.assertIn("напряжение", message.splitlines()[0])
            self.assertIn("0/1", message)

            demo.devices[1].memory[65] = 0
            worker.submit("torque", (1,), True)
            self.wait_for(events, lambda event, args: event == "message" and "1/1" in args[0])
        finally:
            worker.stop_requested.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_sweep_rejects_a_limit_violation_and_stops_when_a_motor_stalls(self):
        class StalledServo(DemoServo):
            def update(self):
                self.updated = monotonic()
                super().update()

        events = Queue()
        demo = DemoPort()
        demo.devices[1] = StalledServo(1, 777, 1024, 120)
        worker = BusWorker("", 1_000_000, events, demo=True)
        worker.SWEEP_STALL_TIMEOUT = 0.1
        try:
            with patch("feetech_panel.worker.DemoPort", return_value=demo):
                worker.start()
                self.wait_for(events, lambda event, args: event == "connected")
            worker.submit("scan", 1, 1)
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 1)
            worker.submit("torque", (1,), True)
            self.wait_for(events, lambda event, args: event == "message" and "1/1" in args[0])

            worker.submit("start_sweep", 1, 1025, 30, 10)
            self.wait_for(events, lambda event, args: event == "message" and args[1] and "±1024" in args[0])

            worker.submit("start_sweep", 1, 50, 30, 10)
            self.wait_for(events, lambda event, args: event == "sweep" and args == (1,))
            self.wait_for(events, lambda event, args: event == "sweep" and args == (None,))
            self.wait_for(events, lambda event, args: event == "message" and args[1] and "перестал приближаться" in args[0])
            self.wait_for(events, lambda event, args: event == "feedback" and args[1].goal == 1024 and args[1].torque == 1)
        finally:
            worker.stop_requested.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    @unittest.skipUnless(hasattr(os, "openpty") and importlib.util.find_spec("serial"), "Для проверки нужен PTY и pyserial")
    def test_worker_uses_a_serial_port_and_polls_growing_bus(self):
        master, slave = os.openpty()
        stop_adapter = Event()
        demo = DemoPort(echo=True)
        demo.devices[1].memory[OPERATING_MODE] = SPEED_MODE

        def adapter():
            request = bytearray()
            while not stop_adapter.is_set():
                readable, _, _ = select.select([master], [], [], 0.02)
                if not readable:
                    continue
                request.extend(os.read(master, 4096))
                while len(request) >= 4:
                    size = request[3] + 4
                    if len(request) < size:
                        break
                    demo.write(bytes(request[:size]))
                    del request[:size]
                    response = demo.read(demo.in_waiting)
                    if response:
                        os.write(master, response)

        adapter_thread = Thread(target=adapter, daemon=True)
        adapter_thread.start()
        events = Queue()
        worker = BusWorker(os.ttyname(slave), 1_000_000, events)
        worker.start()
        try:
            self.wait_for(events, lambda event, args: event == "connected")
            worker.submit("scan", 0, 8)
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 7)

            worker.submit("position_mode", 1)
            self.wait_for(events, lambda event, args: event == "device" and args[0].servo_id == 1 and args[1].single_turn)
            self.wait_for(events, lambda event, args: event == "message" and "Режим позиции (0) подтверждён" in args[0])

            worker.submit("torque", (1, 2), True)
            self.wait_for(events, lambda event, args: event == "message" and "2/2" in args[0])
            worker.submit("start_sweep", 1, 50, 700, 10)
            self.wait_for(events, lambda event, args: event == "motion" and args[1].position == 1074)
            self.wait_for(events, lambda event, args: event == "sweep" and args == (1,))
            self.wait_for(events, lambda event, args: event == "motion" and args[1].position == 974)
            self.wait_for(events, lambda event, args: event == "motion" and args[1].position == 1074)
            worker.submit("stop_sweep")
            self.wait_for(events, lambda event, args: event == "sweep" and args == (None,))
            self.wait_for(events, lambda event, args: event == "message" and not args[1] and "остановлено" in args[0])
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 1
                          and args[1].position == args[1].goal and args[1].torque == 1)

            worker.submit("move", (1, 2), Motion(1600, 1000, 10))
            self.wait_for(events, lambda event, args: event == "message" and "2/2" in args[0])
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 2 and args[1].position == 1600)

            demo.devices[8] = DemoServo(8, 2825, 2000, 120)
            worker.submit("scan", 8, 8)
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 8)
            worker.submit("change_id", 8, 12)
            self.wait_for(events, lambda event, args: event == "renamed" and args == (8, 12))
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 12)

            detached = demo.devices.pop(7)
            self.wait_for(events, lambda event, args: event == "missing" and args[0] == 7)
            demo.devices[7] = detached
            self.wait_for(events, lambda event, args: event == "feedback" and args[0] == 7)
        finally:
            worker.stop_requested.set()
            worker.join(2)
            stop_adapter.set()
            adapter_thread.join(1)
            os.close(slave)
            os.close(master)
        self.assertFalse(worker.is_alive())
