#!/usr/bin/env python3
import fcntl
import os
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String

from common import run

I2C_SLAVE = 0x0703


def fit(label, value, cols=16):
    text = f'{label} {value}'
    return (text if len(text) <= cols else value)[:cols].ljust(cols)


class Screen:
    def __init__(self, cols=16, hold=3.0, background=('white',), page_time=3.5):
        self.cols, self.hold, self.background = cols, hold, set(background)
        self.latest = self.shown = self.marker = self.marked = None
        self.since = 0.0
        self.page_time = page_time
        self.current, self.flipped = 'colour', -float('inf')

    def colour(self, name, now):
        self.latest = name
        self.update(now)

    def update(self, now):
        if self.latest in (None, self.shown):
            return
        held = (self.shown not in (None, *self.background) and self.latest in self.background
                and now - self.since < self.hold)
        if not held:
            self.shown, self.since = self.latest, now

    def rows(self, now):
        self.update(now)
        colour = self.shown.replace('_', ' ') if self.shown else '--'
        marker = '--' if self.marker is None else str(self.marker)
        return fit('Color', colour, self.cols), fit('ArUco', marker, self.cols)

    def mark(self, marker, now):
        if marker != self.marker:
            self.marker, self.marked = marker, now

    def page(self, now):
        self.update(now)
        news = []
        if self.shown not in (None, *self.background):
            news.append((self.since, 'colour'))
        if self.marked is not None:
            news.append((self.marked, 'marker'))
        fresh = [n for n in news if n[0] > self.flipped]
        due = now - self.flipped >= self.page_time
        if fresh and (self.current != 'marker' or due):
            self.current, self.flipped = max(fresh)[1], now
        elif self.marker is not None and due:
            self.current = 'colour' if self.current == 'marker' else 'marker'
            self.flipped = now
        if self.current == 'marker' and self.marker is not None:
            return str(self.marker)
        return (self.shown or '--').replace('_', ' ').upper()


FONT = bytes.fromhex(
    '0000000000' '00005f0000' '0007000700' '147f147f14' '242a7f2a12' '2313086462'
    '3649552250' '0005030000' '001c224100' '0041221c00' '082a1c2a08' '08083e0808'
    '0050300000' '0808080808' '0060600000' '2010080402' '3e5149453e' '00427f4000'
    '4261514946' '2141454b31' '1814127f10' '2745454539' '3c4a494930' '0171090503'
    '3649494936' '064949291e' '0036360000' '0056360000' '0008142241' '1414141414'
    '4122140800' '0201510906' '324979413e' '7e1111117e' '7f49494936' '3e41414122'
    '7f4141221c' '7f49494941' '7f09090101' '3e41415132' '7f0808087f' '00417f4100'
    '2040413f01' '7f08142241' '7f40404040' '7f0204027f' '7f0408107f' '3e4141413e'
    '7f09090906' '3e4151215e' '7f09192946' '4649494931' '01017f0101' '3f4040403f'
    '1f2040201f' '7f2018207f' '6314081463' '0304780403' '6151494543' '00007f4141'
    '0204081020' '41417f0000' '0402010204' '4040404040' '0001020400' '2054545478'
    '7f48444438' '3844444420' '384444487f' '3854545418' '087e090102' '081454543c'
    '7f08040478' '00447d4000' '2040443d00' '007f102844' '00417f4000' '7c04180478'
    '7c08040478' '3844444438' '7c14141408' '081414187c' '7c08040408' '4854545420'
    '043f444020' '3c4040207c' '1c2040201c' '3c4030403c' '4428102844' '0c5050503c'
    '4464544c44' '0008364100' '00007f0000' '0041360800' '0804081008')


def glyph(ch):
    i = ((ord(ch) if 32 <= ord(ch) < 127 else ord('?')) - 32) * 5
    return FONT[i:i + 5]


def layout(text, width=128, height=64):
    best = None
    for lines in ([text], text.split()):
        if not lines:
            continue
        sx = max(1, width // (6 * max(map(len, lines)) - 1))
        sy = max(1, height // (8 * len(lines) - 1))
        sx, sy = min(sx, 2 * sy), min(sy, 2 * sx)
        if best is None or sx * sy > best[1] * best[2]:
            best = (lines, sx, sy)
    return best


def render(text, width=128, height=64):
    lines, sx, sy = layout(text, width, height)
    cols = [0] * width
    y = (height - (8 * len(lines) - 1) * sy) // 2
    for line in lines:
        x = max(0, (width - (6 * len(line) - 1) * sx) // 2)
        for ch in line:
            for col in glyph(ch):
                big = 0
                for dot in range(7):
                    if col >> dot & 1:
                        big |= ((1 << sy) - 1) << (y + dot * sy)
                for _ in range(sx):
                    if x < width:
                        cols[x] |= big
                    x += 1
            x += sx
        y += 8 * sy
    return [bytes(c >> 8 * page & 0xFF for c in cols) for page in range(height // 8)]


class Oled:
    INIT = (0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0xA1, 0xC8,
            0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6)

    def __init__(self, bus=1, address=0x3C, offset=0):
        self.offset = offset
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)
        self.command(*self.INIT)
        for page in range(8):
            self.page(page, bytes(128))
        self.command(0xAF)

    def command(self, *codes):
        os.write(self.fd, bytes((0x00, *codes)))

    def page(self, page, data):
        self.command(0xB0 | page, self.offset & 0x0F, 0x10 | self.offset >> 4)
        os.write(self.fd, bytes((0x40, *data)))

    def show(self, text):
        for i, data in enumerate(render(text)):
            self.page(i, data)


class Display(Node):
    def __init__(self):
        super().__init__('display')
        p = self.declare_parameter
        backend = p('backend', 'auto').value
        self.screen = Screen(hold=p('hold', 3.0).value, page_time=p('page_time', 3.5).value)
        self.oled = None
        if backend != 'log':
            try:
                self.oled = Oled(p('bus', 1).value, p('address', 0x3C).value, p('column_offset', 0).value)
            except OSError:
                self.get_logger().info('no OLED found, logging only')
        self.rows = self.drawn = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'display', latched)
        self.create_subscription(String, 'color', lambda m: self.screen.colour(m.data, time.monotonic()), 10)
        self.create_subscription(Int32, 'aruco/id', lambda m: self.screen.mark(m.data, time.monotonic()), latched)
        self.create_timer(0.2, self.refresh)

    def refresh(self):
        now = time.monotonic()
        page = self.screen.page(now)
        if self.oled and page != self.drawn:
            try:
                self.oled.show(page)
                self.drawn = page
            except OSError:
                pass
        rows = self.screen.rows(now)
        if rows == self.rows:
            return
        self.rows = rows
        self.get_logger().info('[%s] [%s]' % rows)
        self.pub.publish(String(data='\n'.join(rows)))


def main():
    run(Display)


if __name__ == '__main__':
    main()
