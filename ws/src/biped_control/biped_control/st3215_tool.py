"""ST3215 command line tool: scan bus, change IDs, calibrate middle, monitor positions.

Examples:
  ros2 run biped_control st3215_tool scan --port /dev/ttyACM0
  ros2 run biped_control st3215_tool set-id 1 4 --port /dev/ttyACM0   # ONE servo on the bus!
  ros2 run biped_control st3215_tool set-middle 1 2 3 --port /dev/biped_left
  ros2 run biped_control st3215_tool monitor 1 2 3 --port /dev/biped_left
  ros2 run biped_control st3215_tool torque off 1 2 3 --port /dev/biped_left
  ros2 run biped_control st3215_tool move 1 2048 --port /dev/biped_left
"""
import argparse
import sys
import time

import serial

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03

REG_ID = 5
REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_LOCK = 55
REG_PRESENT_POSITION = 56
REG_VOLTAGE = 62
REG_TEMPERATURE = 63


class StsBus:

    def __init__(self, port, baud=1000000, timeout=0.02):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.pending = b''

    def _send(self, sid, instr, params=()):
        length = len(params) + 2
        body = [sid, length, instr, *params]
        chk = (~sum(body)) & 0xFF
        pkt = bytes([0xFF, 0xFF, *body, chk])
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self._drop_echo(pkt)

    def _drop_echo(self, pkt):
        got = self.ser.read(len(pkt))
        for echo in (pkt, pkt[1:]):
            if got.startswith(echo):
                got = got[len(echo):]
                break
        self.pending = got

    def _read(self, n):
        data, self.pending = self.pending[:n], self.pending[n:]
        if len(data) < n:
            data += self.ser.read(n - len(data))
        return data

    def _recv(self):
        hdr = self._read(4)
        if len(hdr) < 4 or hdr[0] != 0xFF or hdr[1] != 0xFF:
            return None
        rest = self._read(hdr[3])
        if len(rest) < hdr[3]:
            return None
        if (~(hdr[2] + hdr[3] + sum(rest[:-1]))) & 0xFF != rest[-1]:
            return None
        return hdr[2], rest[0], bytes(rest[1:-1])

    def ping(self, sid):
        self._send(sid, INST_PING)
        r = self._recv()
        return r is not None and r[0] == sid

    def read(self, sid, addr, n):
        self._send(sid, INST_READ, [addr, n])
        r = self._recv()
        return None if r is None else r[2]

    def write(self, sid, addr, data):
        self._send(sid, INST_WRITE, [addr, *data])
        if sid != 0xFE:
            self._recv()

    def read_u16(self, sid, addr):
        d = self.read(sid, addr, 2)
        if d is None or len(d) < 2:
            return None
        v = d[0] | (d[1] << 8)
        return -(v & 0x7FFF) if v & 0x8000 else v


def cmd_scan(bus, a):
    found = []
    for sid in range(0, a.max_id + 1):
        if bus.ping(sid):
            pos = bus.read_u16(sid, REG_PRESENT_POSITION)
            volt = bus.read(sid, REG_VOLTAGE, 2)
            v = volt[0] / 10 if volt else float('nan')
            t = volt[1] if volt else -1
            print(f'  ID {sid:3d}  pos {pos}  {v:.1f} V  {t} C')
            found.append(sid)
    print(f'found: {found}' if found else 'no servos found (port? power? baud?)')


def cmd_set_id(bus, a):
    if not bus.ping(a.old):
        sys.exit(f'servo {a.old} does not answer')
    if a.new != a.old and bus.ping(a.new):
        sys.exit(f'ID {a.new} already on the bus - connect one servo at a time')
    bus.write(a.old, REG_LOCK, [0])
    bus.write(a.old, REG_ID, [a.new])
    time.sleep(0.05)
    bus.write(a.new, REG_LOCK, [1])
    print('ok' if bus.ping(a.new) else 'FAILED - servo does not answer on new ID')


def cmd_set_middle(bus, a):
    for sid in a.ids:
        before = bus.read_u16(sid, REG_PRESENT_POSITION)
        if before is None:
            print(f'ID {sid}: no answer, skipped')
            continue
        bus.write(sid, REG_TORQUE_ENABLE, [0])
        bus.write(sid, REG_TORQUE_ENABLE, [128])
        time.sleep(0.05)
        now = bus.read_u16(sid, REG_PRESENT_POSITION)
        if now is not None:
            bus.write(sid, REG_GOAL_POSITION, [now & 0xFF, (now >> 8) & 0xFF])
        bus.write(sid, REG_TORQUE_ENABLE, [0])
        print(f'ID {sid}: {before} -> {now} (expect ~2048), torque off')


def cmd_torque(bus, a):
    for sid in a.ids:
        bus.write(sid, REG_TORQUE_ENABLE, [1 if a.state == 'on' else 0])
    print(f'torque {a.state}: {a.ids}')


def cmd_move(bus, a):
    bus.write(a.id, REG_TORQUE_ENABLE, [1])
    bus.write(a.id, REG_GOAL_POSITION, [a.ticks & 0xFF, (a.ticks >> 8) & 0xFF])


def cmd_monitor(bus, a):
    try:
        while True:
            vals = [f'{sid}:{bus.read_u16(sid, REG_PRESENT_POSITION)!s:>5}' for sid in a.ids]
            print('  '.join(vals), end='\r', flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', default='/dev/ttyACM0')
    p.add_argument('--baud', type=int, default=1000000)
    sub = p.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('scan')
    s.add_argument('--max-id', type=int, default=20)
    s.set_defaults(fn=cmd_scan)

    s = sub.add_parser('set-id', help='change ID (only ONE servo connected!)')
    s.add_argument('old', type=int)
    s.add_argument('new', type=int)
    s.set_defaults(fn=cmd_set_id)

    s = sub.add_parser('set-middle', help='store current position as 2048')
    s.add_argument('ids', type=int, nargs='+')
    s.set_defaults(fn=cmd_set_middle)

    s = sub.add_parser('torque')
    s.add_argument('state', choices=['on', 'off'])
    s.add_argument('ids', type=int, nargs='+')
    s.set_defaults(fn=cmd_torque)

    s = sub.add_parser('move')
    s.add_argument('id', type=int)
    s.add_argument('ticks', type=int)
    s.set_defaults(fn=cmd_move)

    s = sub.add_parser('monitor')
    s.add_argument('ids', type=int, nargs='+')
    s.set_defaults(fn=cmd_monitor)

    a = p.parse_args(argv)
    a.fn(StsBus(a.port, a.baud), a)


if __name__ == '__main__':
    main()
