"""Serialize access to one physical or demo bus outside of the UI thread."""

from dataclasses import dataclass, replace
from queue import Empty, Queue
from threading import Event, Thread
from time import monotonic

from .bus import BusError, Identity, Motion, NoResponse, ServoBus, status_text
from .demo import DemoPort


@dataclass
class Device:
    identity: Identity
    due: float = 0
    failures: int = 0
    config_retry_at: float | None = None


@dataclass
class Sweep:
    servo_id: int
    target: Motion
    opposite: Motion
    closest: int
    last_progress: float


class BusWorker(Thread):
    POLL_PERIOD = 1 / 30
    DEVICE_PERIOD = 0.20
    CONFIG_RETRY_PERIOD = 1
    SWEEP_TOLERANCE = 2
    SWEEP_STALL_TIMEOUT = 5

    def __init__(self, device, baud_rate, events, *, demo=False):
        super().__init__(name="feetech-bus", daemon=True)
        self.device = device
        self.baud_rate = baud_rate
        self.events = events
        self.demo = demo
        self.commands = Queue()
        self.stop_requested = Event()
        self.cancel_scan = Event()
        self.bus = None
        self.devices = {}
        self.sweep = None

    def submit(self, operation, *args):
        if operation == "scan":
            self.cancel_scan.clear()
        self.commands.put((operation, args))

    def emit(self, event, *args):
        self.events.put((event, args))

    def run(self):
        try:
            self.bus = ServoBus(DemoPort(), timeout=0.003) if self.demo else ServoBus.open(self.device, self.baud_rate)
            self.emit("connected")
            self._serve()
        except (BusError, OSError, ValueError) as exc:
            self.emit("port_error", str(exc))
        finally:
            if self.bus is not None:
                self._stop_sweep()
                try:
                    self.bus.close()
                except OSError as exc:
                    self.emit("port_error", str(exc))
            self.emit("disconnected")

    def _serve(self):
        next_poll = 0
        handlers = {
            "scan": self._scan,
            "move": self._move,
            "torque": self._torque,
            "start_sweep": self._start_sweep,
            "stop_sweep": self._stop_sweep,
            "position_mode": self._set_position_mode,
            "change_id": self._change_id,
        }
        while not self.stop_requested.is_set():
            due = min((device.due for device in self.devices.values()), default=float("inf"))
            wait = min(0.05, max(0, max(next_poll, due) - monotonic()))
            try:
                operation, args = self.commands.get(timeout=wait)
            except Empty:
                if wait < 0.05 and not self.stop_requested.is_set() and self.devices:
                    device = min(self.devices.values(), key=lambda item: item.due)
                    self._poll(device)
                    next_poll = monotonic() + self.POLL_PERIOD
                continue
            if self.stop_requested.is_set():
                break
            try:
                if self.sweep is not None and operation not in ("stop_sweep", "torque"):
                    raise BusError("Сначала остановите движение вперёд-назад.")
                handlers[operation](*args)
            except BusError as exc:
                self.emit("message", str(exc), True)
            finally:
                self.emit("ready")

    def _remember(self, identity):
        device = Device(identity)
        self.devices[identity.servo_id] = device
        self.emit("device", identity, self._read_config(device))
        return device

    def _read_config(self, device):
        try:
            config = self.bus.config(device.identity.servo_id)
        except BusError as exc:
            if device.config_retry_at is None:
                self.emit("log", f"Не удалось прочитать настройки мотора. {exc}", True)
            device.config_retry_at = monotonic() + self.CONFIG_RETRY_PERIOD
            return None
        device.config_retry_at = None
        return config

    def _scan(self, first, last):
        found = 0
        bad = []
        checked = 0
        for servo_id in range(first, last + 1):
            if self.stop_requested.is_set() or self.cancel_scan.is_set():
                break
            try:
                identity = self.bus.identify(servo_id)
            except NoResponse as exc:
                if servo_id in self.devices:
                    self.emit("missing", servo_id, str(exc))
            except BusError as exc:
                bad.append(servo_id)
                if servo_id in self.devices:
                    self.emit("missing", servo_id, str(exc))
            else:
                self._remember(identity)
                found += 1
            checked += 1
            if checked == 1 or checked % 8 == 0 or servo_id == last:
                self.emit("progress", checked, last - first + 1)
        state = "Поиск завершён" if checked == last - first + 1 else "Поиск остановлен"
        message = f"{state}. Найдено: {found}. Проверено адресов: {checked}."
        if bad:
            message += f" Ошибки связи на ID: {', '.join(map(str, bad))}."
        self.emit("message", message, bool(bad))

    def _poll(self, device):
        servo_id = device.identity.servo_id
        if device.config_retry_at is not None and monotonic() >= device.config_retry_at:
            config = self._read_config(device)
            if config is not None:
                self.emit("device", device.identity, config)
        try:
            feedback = self.bus.feedback(servo_id)
        except BusError as exc:
            device.failures += 1
            device.due = monotonic() + min(5, 0.5 * device.failures)
            if device.failures == 1:
                self.emit("missing", servo_id, str(exc))
            if self.sweep is not None and self.sweep.servo_id == servo_id:
                self._stop_sweep(str(exc))
        else:
            device.failures = 0
            device.due = monotonic() + self.DEVICE_PERIOD
            self.emit("feedback", servo_id, feedback)
            if self.sweep is not None and self.sweep.servo_id == servo_id:
                self._advance_sweep(feedback)

    def _batch(self, ids, name, action):
        success = 0
        errors = []
        for servo_id in ids:
            if self.stop_requested.is_set():
                break
            try:
                action(servo_id)
            except BusError as exc:
                errors.append(str(exc))
            else:
                success += 1
            self.devices[servo_id].due = 0
        summary = f"{name}. Подтверждено: {success}/{len(ids)}."
        if len(ids) == 1 and errors:
            self.emit("message", f"{errors[0]}\n{summary}", True)
            return
        self.emit("message", summary, bool(errors))
        for error in errors:
            self.emit("log", error, True)

    def _move(self, ids, motion):
        self._batch(ids, "Команда позиции", lambda servo_id: self._send_motion(servo_id, motion))

    def _send_motion(self, servo_id, motion):
        self.bus.move(servo_id, motion)
        self.emit("motion", servo_id, motion)

    def _start_sweep(self, servo_id, amplitude, speed, acceleration):
        if servo_id not in self.devices:
            raise BusError(f"ID {servo_id}. Сначала найдите мотор.")
        current = self.bus.feedback(servo_id)
        if current.status:
            raise BusError(f"ID {servo_id}. Мотор сообщает об ошибке ({status_text(current.status)}).")
        if current.torque != 1:
            raise BusError(f"ID {servo_id}. Сначала включите удержание.")
        config = self.bus.config(servo_id)
        if problem := config.position_problem:
            raise BusError(f"ID {servo_id}. {problem}")
        available = min(current.position - config.minimum, config.maximum - current.position)
        if not 1 <= amplitude <= available:
            raise BusError(f"ID {servo_id}. Доступная амплитуда от текущей позиции ±{max(0, available)} отсч.")

        upper = Motion(current.position + amplitude, speed, acceleration)
        lower = Motion(current.position - amplitude, speed, acceleration)
        self.sweep = Sweep(servo_id, upper, lower, amplitude, monotonic())
        try:
            self._send_motion(servo_id, upper)
        except BusError as exc:
            self._stop_sweep(str(exc))
            return
        self.sweep.last_progress = monotonic()
        self.emit("sweep", servo_id)
        self.emit("message", f"ID {servo_id}. Движение вперёд-назад {lower.position}–{upper.position} отсч.", False)

    def _advance_sweep(self, feedback):
        if self.stop_requested.is_set():
            return
        sweep = self.sweep
        if feedback.torque != 1:
            self._stop_sweep("Удержание выключено.", hold=False)
            return
        if feedback.status:
            self._stop_sweep(f"Мотор сообщает об ошибке ({status_text(feedback.status)}).")
            return
        if feedback.goal != sweep.target.position:
            self._stop_sweep("Цель мотора изменилась.", hold=False)
            return

        distance = abs(feedback.position - sweep.target.position)
        if distance <= self.SWEEP_TOLERANCE and not feedback.moving:
            try:
                self._send_motion(sweep.servo_id, sweep.opposite)
            except BusError as exc:
                self._stop_sweep(str(exc))
                return
            sweep.target, sweep.opposite = sweep.opposite, sweep.target
            sweep.closest = abs(feedback.position - sweep.target.position)
            sweep.last_progress = monotonic()
        elif distance < sweep.closest:
            sweep.closest = distance
            sweep.last_progress = monotonic()
        elif monotonic() - sweep.last_progress >= self.SWEEP_STALL_TIMEOUT:
            self._stop_sweep("Мотор перестал приближаться к цели.")

    def _stop_sweep(self, error=None, *, hold=True):
        if self.sweep is None:
            return
        sweep, self.sweep = self.sweep, None
        message = f"ID {sweep.servo_id}. Движение вперёд-назад остановлено."
        if error:
            message += f" {error}"
        if hold:
            try:
                current = self.bus.feedback(sweep.servo_id)
                if current.torque == 1:
                    if current.status:
                        raise BusError(f"Мотор сообщает об ошибке ({status_text(current.status)}).")
                    self._send_motion(sweep.servo_id, replace(sweep.target, position=current.position))
            except (BusError, OSError) as exc:
                message = f"ID {sweep.servo_id}. Остановку мотора подтвердить не удалось. {exc}"
                error = message
        self.emit("sweep", None)
        self.emit("message", message, bool(error))

    def _torque(self, ids, enabled):
        if self.sweep is not None:
            if enabled:
                raise BusError("Сначала остановите движение вперёд-назад.")
            if self.sweep.servo_id in ids:
                self._stop_sweep(hold=False)
        label = "Включение удержания" if enabled else "Снятие удержания"
        self._batch(ids, label, lambda servo_id: self.bus.set_torque(servo_id, enabled))

    def _set_position_mode(self, servo_id):
        device = self.devices[servo_id]
        try:
            config = self.bus.set_position_mode(servo_id)
        except BusError:
            device.config_retry_at = 0
            raise
        finally:
            device.due = 0
        device.config_retry_at = None
        self.emit("device", device.identity, config)
        self.emit("message", f"ID {servo_id}. Режим позиции (0) подтверждён.", False)

    def _change_id(self, old, new):
        if new != old and new in self.devices:
            raise BusError(f"ID {new} уже есть в списке моторов этой сессии.")
        try:
            identity = self.bus.change_id(old, new)
        except BusError:
            self.devices[old].due = 0
            if new in self.devices:
                self.devices[new].due = 0
            raise
        del self.devices[old]
        self._remember(identity)
        self.emit("renamed", old, new)
        self.emit("message", f"Новый ID прочитан из мотора: {old} → {new}.", False)
