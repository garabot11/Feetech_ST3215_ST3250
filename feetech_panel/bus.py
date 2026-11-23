"""Feetech STS half-duplex communication and register operations."""

from dataclasses import dataclass, replace
from struct import pack, unpack_from
from time import monotonic, sleep


BAUD_RATES = (1_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400)
MAX_ID = 253
MAX_POSITION = 4095
POSITION_MODE, SPEED_MODE = 0, 1
MODELS = {777: "ST3215", 2825: "ST3250"}

READ, WRITE = 2, 3
MODEL = 3
SERVO_ID = 5
LIMITS = 9
ANGLE_SETTINGS = 30
OPERATING_MODE = 33
TORQUE = 40
ACCELERATION = 41
GOAL = 42
LOCK = 55
PRESENT = 56

HEADER = b"\xff\xff"
EEPROM_SETTLE = 0.050


class BusError(Exception):
    """A servo could not complete the requested operation."""


class NoResponse(BusError):
    """The address did not return any response bytes."""


def status_text(status):
    flags = {1: "напряжение", 2: "энкодер", 4: "температура", 8: "ток", 32: "нагрузка"}
    names = [name for mask, name in flags.items() if status & mask]
    unknown = status & ~sum(flags)
    if unknown:
        names.append(f"код 0x{unknown:02X}")
    return ", ".join(names)


def frame(servo_id, code, data=b""):
    body = bytes((servo_id, len(data) + 2, code)) + data
    return HEADER + body + bytes((~sum(body) & 0xFF,))


def signed(raw, bit=15):
    sign_bit = 1 << bit
    return -(raw & (sign_bit - 1)) if raw & sign_bit else raw


@dataclass(frozen=True)
class Identity:
    servo_id: int
    model: int

    @property
    def name(self):
        return MODELS.get(self.model, f"Код {self.model}")


@dataclass(frozen=True)
class Config:
    minimum: int
    maximum: int
    resolution: int
    mode: int

    @property
    def single_turn(self):
        return not self.position_problem

    @property
    def position_problem(self):
        if self.mode != POSITION_MODE:
            return f"Режим {self.mode}. Для позиционного управления нужен режим 0."
        if self.resolution != 1:
            return f"Разрешение {self.resolution}. Для одного оборота нужно 1."
        if not 0 <= self.minimum < self.maximum <= MAX_POSITION:
            return f"Пределы {self.minimum}–{self.maximum}. Минимум должен быть меньше максимума в пределах 0–{MAX_POSITION}."
        return ""


@dataclass(frozen=True)
class Feedback:
    timestamp: float
    torque: int
    goal: int
    position: int
    speed: int
    load: float
    voltage: float
    temperature: int
    moving: bool
    current: float
    status: int


@dataclass(frozen=True)
class Motion:
    position: int
    speed: int
    acceleration: int

    def __post_init__(self):
        if not 0 <= self.position <= MAX_POSITION:
            raise BusError(f"Позиция должна быть от 0 до {MAX_POSITION}.")
        if not 1 <= self.speed <= 32767:
            raise BusError("Скорость должна быть от 1 до 32767.")
        if not 1 <= self.acceleration <= 254:
            raise BusError("Ускорение должно быть от 1 до 254.")


class ServoBus:
    def __init__(self, port, timeout=0.030):
        self.port = port
        self.timeout = timeout

    @classmethod
    def open(cls, device, baud_rate):
        try:
            import serial
        except ImportError as exc:
            raise BusError("Установите pyserial: python -m pip install -r requirements.txt") from exc
        return cls(serial.Serial(device, baud_rate, timeout=0.004, write_timeout=0.2))

    def close(self):
        self.port.close()

    def _send(self, request):
        self.port.reset_input_buffer()
        if self.port.write(request) != len(request):
            raise OSError("Последовательный порт записал только часть пакета.")

    def _read(self, servo_id, address, size):
        request = frame(servo_id, READ, bytes((address, size)))
        self._send(request)
        return self._receive(request, (servo_id,), size)

    def _receive(self, request, expected_ids, size):
        servo_id = request[2]
        deadline = monotonic() + self.timeout
        buffer = bytearray()
        unexpected = False
        while monotonic() < deadline:
            buffer.extend(self.port.read(self.port.in_waiting or 1))
            while len(buffer) >= 4:
                start = buffer.find(HEADER)
                if start < 0:
                    unexpected = True
                    del buffer[:-1]
                    break
                if start:
                    unexpected = True
                    del buffer[:start]
                if len(buffer) < 4:
                    break
                length = buffer[3]
                if buffer[2] == 0xFF or not 2 <= length <= max(size + 2, request[3]):
                    unexpected = True
                    del buffer[0]
                    continue
                end = length + 4
                if len(buffer) < end:
                    break
                packet = bytes(buffer[:end])
                if sum(packet[2:]) & 0xFF != 0xFF:
                    unexpected = True
                    del buffer[0]
                    continue
                del buffer[:end]
                if packet == request:
                    continue
                if packet[2] in expected_ids and length == size + 2:
                    return packet[5:-1], packet[4]
                unexpected = True
        if buffer or unexpected:
            raise BusError(f"ID {servo_id}: повреждённый или неожиданный ответ. Проверьте шину и уникальность ID.")
        raise NoResponse(f"ID {servo_id}: нет ответа.")

    def _write(self, servo_id, address, value):
        request = frame(servo_id, WRITE, bytes((address,)) + value)
        self._send(request)
        ids = (servo_id, value[0]) if address == SERVO_ID else (servo_id,)
        try:
            self._receive(request, ids, 0)
        except NoResponse:
            # Reply level 0 suppresses write replies. The next step verifies the register.
            pass

    def _write_verified(self, servo_id, address, value):
        self._write(servo_id, address, value)
        observed, _ = self._read(servo_id, address, len(value))
        if observed != value:
            raise BusError(f"ID {servo_id}: мотор вернул другое значение регистра {address}.")

    def identify(self, servo_id):
        if not 0 <= servo_id <= MAX_ID:
            raise BusError(f"ID должен быть от 0 до {MAX_ID}.")
        data, _ = self._read(servo_id, MODEL, 3)
        model, registered_id = unpack_from("<HB", data)
        if registered_id != servo_id:
            raise BusError(f"ID {servo_id}: мотор сообщает адрес {registered_id}.")
        return Identity(servo_id, model)

    def config(self, servo_id):
        limits, _ = self._read(servo_id, LIMITS, 4)
        settings, _ = self._read(servo_id, ANGLE_SETTINGS, 4)
        minimum, maximum = unpack_from("<HH", limits)
        return Config(minimum, maximum, settings[0], settings[3])

    def feedback(self, servo_id):
        control, control_status = self._read(servo_id, TORQUE, 4)
        present, packet_status = self._read(servo_id, PRESENT, 15)
        position, speed, load = unpack_from("<HHH", present)
        return Feedback(
            timestamp=monotonic(), torque=control[0], goal=signed(unpack_from("<H", control, 2)[0]),
            position=signed(position), speed=signed(speed), load=signed(load, 10) / 10,
            voltage=present[6] / 10, temperature=present[7], moving=bool(present[10]),
            current=signed(unpack_from("<H", present, 13)[0]) * 0.0065,
            status=control_status | packet_status | present[9],
        )

    def _known_model(self, servo_id):
        identity = self.identify(servo_id)
        if identity.model not in MODELS:
            raise BusError(f"ID {servo_id}: запись доступна для ST3215 и ST3250. Код модели {identity.model}.")
        return identity

    def _check_motion(self, servo_id, position):
        config = self.config(servo_id)
        if problem := config.position_problem:
            raise BusError(f"ID {servo_id}. Требуется однооборотный режим. {problem}")
        if not config.minimum <= position <= config.maximum:
            raise BusError(f"ID {servo_id}. Позиция {position} вне диапазона {config.minimum}–{config.maximum} отсч.")

    def move(self, servo_id, motion):
        self._known_model(servo_id)
        self._check_motion(servo_id, motion.position)
        torque, status = self._read(servo_id, TORQUE, 1)
        if status:
            raise BusError(f"ID {servo_id}: мотор сообщает об ошибке ({status_text(status)}).")
        if torque != b"\x01":
            raise BusError(f"ID {servo_id}: сначала включите удержание.")
        self._write_verified(
            servo_id, ACCELERATION, pack("<BHHH", motion.acceleration, motion.position, 0, motion.speed),
        )

    def set_torque(self, servo_id, enabled):
        self._known_model(servo_id)
        if not enabled:
            self._write_verified(servo_id, TORQUE, b"\x00")
            return
        current = self.feedback(servo_id)
        if current.status:
            raise BusError(f"ID {servo_id}: мотор сообщает об ошибке ({status_text(current.status)}).")
        self._check_motion(servo_id, current.position)
        already_enabled = current.torque == 1
        if already_enabled and current.goal == current.position:
            return

        # Some firmware enables torque as soon as a goal is written.
        goal = pack("<H", current.position)
        try:
            self._write(servo_id, GOAL, goal)
            control, status = self._read(servo_id, TORQUE, 4)
            if status:
                raise BusError(f"ID {servo_id}: мотор сообщает об ошибке ({status_text(status)}).")
            if control[2:] != goal:
                raise BusError(f"ID {servo_id}: не удалось подтвердить удержание текущей позиции.")
            if control[0] != 1:
                self._write_verified(servo_id, TORQUE, b"\x01")
        except (BusError, OSError):
            if already_enabled:
                raise
            try:
                self._write_verified(servo_id, TORQUE, b"\x00")
            except (BusError, OSError) as exc:
                raise BusError(f"ID {servo_id}: состояние удержания неизвестно. {exc}") from exc
            raise

    def set_position_mode(self, servo_id):
        self._known_model(servo_id)
        config = self.config(servo_id)
        expected = replace(config, mode=POSITION_MODE)
        if problem := expected.position_problem:
            raise BusError(f"ID {servo_id}. {problem}")
        if config.mode == POSITION_MODE:
            return config
        if config.mode != SPEED_MODE:
            raise BusError(f"ID {servo_id}. Переключение из режима {config.mode} недоступно.")
        if self._read(servo_id, TORQUE, 1)[0] != b"\x00":
            raise BusError(f"ID {servo_id}. Снимите удержание перед сменой режима.")

        try:
            self._write_verified(servo_id, LOCK, b"\x00")
            self._write(servo_id, OPERATING_MODE, bytes((POSITION_MODE,)))
            sleep(EEPROM_SETTLE)
            confirmed = self.config(servo_id)
            if confirmed != expected:
                raise BusError(f"ID {servo_id}. Настройки режима позиции не подтверждены.")
            if self._read(servo_id, TORQUE, 1)[0] != b"\x00":
                self._write_verified(servo_id, TORQUE, b"\x00")
        finally:
            try:
                self._write_verified(servo_id, LOCK, b"\x01")
            except (BusError, OSError) as exc:
                raise BusError(f"ID {servo_id}. Блокировка EEPROM не подтверждена. {exc}") from exc
        return confirmed

    def change_id(self, old, new):
        if old == new:
            raise BusError("Укажите ID, отличный от текущего.")
        source = self._known_model(old)
        for _ in range(2):
            try:
                self.identify(new)
            except NoResponse:
                continue
            raise BusError(f"ID {new} уже занят.")
        if self._read(old, TORQUE, 1)[0] != b"\x00":
            raise BusError(f"ID {old}: снимите удержание перед сменой ID.")

        for_relock = old
        try:
            self._write_verified(old, LOCK, b"\x00")
            self._write(old, SERVO_ID, bytes((new,)))
            sleep(EEPROM_SETTLE)
            for_relock = new
            identity = self.identify(new)
            if identity.model != source.model:
                raise BusError(f"ID {new}: обнаружен другой код модели.")
            try:
                self.identify(old)
            except NoResponse:
                pass
            else:
                raise BusError(f"Старый ID {old} всё ещё отвечает. Проверьте, что на шине один мотор.")
            self._write_verified(new, LOCK, b"\x01")
        except (BusError, OSError) as exc:
            relocked = False
            for address in (for_relock, new if for_relock == old else old):
                try:
                    if self.identify(address).model == source.model:
                        self._write_verified(address, LOCK, b"\x01")
                        relocked = True
                except (BusError, OSError):
                    continue
            message = f"Смена ID не подтверждена. Повторите поиск адресов {old} и {new}. {exc}"
            if not relocked:
                message += " Блокировка EEPROM не подтверждена."
            raise BusError(message) from exc
        return identity
