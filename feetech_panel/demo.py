"""In-memory STS adapter used by the demo and protocol tests."""

from struct import pack_into, unpack_from
from time import monotonic, sleep

from .bus import ACCELERATION, GOAL, LOCK, READ, SERVO_ID, TORQUE, WRITE, frame


class DemoServo:
    def __init__(self, servo_id, model, position, voltage):
        self.memory = bytearray(128)
        pack_into("<HBB", self.memory, 3, model, servo_id, 0)
        self.memory[8] = 1
        pack_into("<HH", self.memory, 9, 0, 4095)
        self.memory[30] = 1
        pack_into("<H", self.memory, GOAL, position)
        pack_into("<H", self.memory, 46, 170)
        self.memory[ACCELERATION] = 20
        self.memory[LOCK] = 1
        self.memory[62] = voltage
        self.position = float(position)
        self.updated = monotonic()

    def update(self):
        now = monotonic()
        goal = unpack_from("<H", self.memory, GOAL)[0]
        raw_speed = unpack_from("<H", self.memory, 46)[0]
        holding = self.memory[TORQUE] == 1
        distance = goal - self.position
        speed = min(3500, (raw_speed or 600) * 5)
        step = min(speed * (now - self.updated), abs(distance))
        if holding and distance:
            self.position += step if distance > 0 else -step
        moving = holding and step < abs(distance)
        self.updated = now
        reported_speed = (raw_speed | 0x8000 if distance < 0 else raw_speed) if moving else 0
        pack_into("<HHH", self.memory, 56, round(self.position), reported_speed, 100 if moving else 15 if holding else 0)
        self.memory[63] = 31 if holding else 28
        self.memory[66] = int(moving)
        pack_into("<HH", self.memory, 67, goal, 80 if moving else 24 if holding else 0)


class DemoPort:
    def __init__(self, echo=False):
        self.devices = {
            1: DemoServo(1, 777, 1024, 120),
            2: DemoServo(2, 2825, 2100, 120),
            7: DemoServo(7, 777, 2950, 120),
        }
        pack_into("<HH", self.devices[7].memory, 9, 1400, 3200)
        self.echo = echo
        self.buffer = bytearray()

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, packet):
        if self.echo:
            self.buffer.extend(packet)
        servo_id, code, address = packet[2], packet[4], packet[5]
        servo = self.devices.get(servo_id)
        if servo is None:
            return len(packet)

        servo.update()
        memory = servo.memory
        if code == READ:
            size = packet[6]
            self.buffer.extend(frame(servo_id, memory[65], memory[address:address + size]))
        elif code == WRITE:
            value = packet[6:-1]
            if address == SERVO_ID:
                if memory[LOCK] == 0:
                    memory[SERVO_ID] = value[0]
                    self.devices[value[0]] = self.devices.pop(servo_id)
            else:
                memory[address:address + len(value)] = value
                if address <= GOAL and address + len(value) >= GOAL + 2:
                    memory[TORQUE] = 1
            if memory[8] == 1:
                self.buffer.extend(frame(servo_id, memory[65]))
        return len(packet)

    def read(self, size):
        if not self.buffer:
            sleep(0.001)
        chunk = bytes(self.buffer[:size])
        del self.buffer[:size]
        return chunk

    def close(self):
        self.buffer.clear()
