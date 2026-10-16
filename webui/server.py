#!/usr/bin/env python3
"""hqdd_1 QDD 用 Web UI バックエンド。

ブラウザ <-WebSocket-> このサーバ <-SocketCAN(can0) または UART-> 基板

起動:  python3 server.py [--port 8080]   ->  http://localhost:8080
依存:  aiohttp, pyserial (CANは標準ライブラリのSocketCANのみ。python-can不要)
"""
import argparse, asyncio, json, re, socket, struct, threading, time
from pathlib import Path

import serial
import serial.tools.list_ports
from aiohttp import web, WSMsgType

CMD_BASE, FB_BASE, REPLY_BASE = 0x100, 0x200, 0x300
STATE = {}  # link / stop_on_close
GAIN = re.compile(r"GAIN (\w\w) (-?\d+(?:\.\d+)?(?:e-?\d+)?)")
PFX = re.compile(r"#(\d+) (.*)")
KV = re.compile(r"(\w+)=(-?\d+(?:\.\d+)?)")


class Link:
    """現在の接続(CAN or UART)1本分。"""

    def __init__(self, app):
        self.app = app
        self.kind = None
        self.sock = None
        self.ser = None
        self.dev_id = 0
        self.stop_flag = threading.Event()
        self.target = 0.0
        self.mode = None
        self.cap = None  # UARTのキャプチャ収集用
        self.last_sa = 0.0  # 直近の出力軸角度(モード切替時の目標追跡用)

    # ---- 共通 ----
    def emit(self, msg):
        loop = self.app["loop"]
        loop.call_soon_threadsafe(asyncio.ensure_future, self.app["broadcast"](msg))

    def track(self, text):
        m = re.fullmatch(r"M([-+]?\d+(?:\.\d*)?)", text)
        if m:
            self.target = float(m.group(1))
        m = re.fullmatch(r"MC(\d)", text)
        if m:
            # 基板はモード切替時に目標を「位置モード=現在位置 / それ以外=0」にする
            self.mode = int(m.group(1))
            self.target = self.last_sa if self.mode == 2 else 0.0
        if text == "Z":
            self.last_sa = 0.0
            if self.mode == 2:
                self.target = 0.0
        if text == "S":
            self.target, self.mode = 0.0, 1

    def send(self, text, all_=False):
        text = text.strip()
        if not text or self.kind is None:
            raise RuntimeError("未接続")
        if self.kind == "can":
            if self.dev_id < 0:
                raise RuntimeError("デバイスID検出待ち(テレメトリ受信後に送信できます)")
            data = text.encode("ascii")
            if len(data) > 8:
                raise RuntimeError(f"CANは1フレーム8文字まで({len(data)}文字): {text}")
            frame = struct.pack("=IB3x8s", CMD_BASE + self.dev_id, len(data), data.ljust(8, b"\0"))
            self.sock.send(frame)
        else:
            # 同じUARTバスに複数台が乗るため、宛先IDを行頭に付ける(一斉は"@*")
            head = "@* " if all_ else f"@{self.dev_id} "
            self.ser.write((head + text + "\n").encode("ascii"))
        self.track(text)
        self.emit({"type": "log", "dir": "tx", "text": ("[全台] " if all_ else "") + text})

    # ---- CAN ----
    def open_can(self, iface, dev_id, bitrate_note=""):
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
        s.settimeout(0.2)
        self.sock, self.kind, self.dev_id = s, "can", dev_id
        threading.Thread(target=self._can_rx, daemon=True).start()

    def _can_rx(self):
        t0 = time.monotonic()
        while not self.stop_flag.is_set():
            try:
                raw = self.sock.recv(16)
            except socket.timeout:
                continue
            except OSError:
                break
            cid, dlc, data = struct.unpack("=IB3x8s", raw)
            cid &= 0x1FFFFFFF
            if self.dev_id >= 0 and cid == REPLY_BASE + self.dev_id and dlc >= 8:
                if data[0:2] == b"WS":
                    self.emit({"type": "saved", "status": data[2]})
                elif data[0:2] == b"FR":
                    self.emit({"type": "reset", "status": data[2]})
                elif data[2]:
                    self.emit({"type": "gain", "name": data[0:2].decode("ascii", "replace"),
                               "value": struct.unpack("<f", data[4:8])[0]})
                else:
                    self.emit({"type": "log", "dir": "rx", "text": "ゲイン名が不明: " + data[0:2].decode("ascii", "replace")})
                continue
            if not (FB_BASE <= cid < FB_BASE + 8) or dlc < 8:
                continue
            if self.dev_id < 0:  # 自動検出: 最初に見えたノードを採用
                self.dev_id = cid - FB_BASE
                self.emit({"type": "log", "dir": "rx", "text": f"デバイスID自動検出: {self.dev_id}"})
                self.emit({"type": "devid", "id": self.dev_id})
            elif cid != FB_BASE + self.dev_id:
                continue
            ang, vel, iq, trq = struct.unpack("<4h", data)
            self.last_sa = ang / 1000
            self.emit({"type": "telem", "t": time.monotonic() - t0,
                       "sa": ang / 1000, "v": vel / 100, "iq": iq / 1000, "trq": trq / 1000,
                       "tgt": self.target})

    # ---- UART ----
    def open_uart(self, port, baud, dev_id):
        if dev_id < 0:
            raise RuntimeError("UARTではデバイスIDを指定してください")
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.kind, self.dev_id = "uart", dev_id
        threading.Thread(target=self._uart_rx, daemon=True).start()
        # 共有バスでテレメトリが衝突しないよう、全台を黙らせてから対象だけ有効にする
        self.send("T0", all_=True)
        time.sleep(0.05)
        self.send("T1")

    def _uart_rx(self):
        t0 = time.monotonic()
        while not self.stop_flag.is_set():
            try:
                line = self.ser.readline().decode("ascii", "replace").strip()
            except Exception:
                break
            if not line:
                continue
            m = PFX.match(line)   # "#<id> ..." は対象ID以外を捨てる(接頭辞なしは従来形式として通す)
            if m:
                if int(m.group(1)) != self.dev_id:
                    continue
                line = m.group(2)
            if line == "SAVED" or line.startswith("SAVE "):
                self.emit({"type": "saved", "status": 1 if line == "SAVED" else 2 if "BUSY" in line else 0})
                continue
            if line.startswith("RESET "):
                self.emit({"type": "reset", "status": 1 if line == "RESET OK" else 2 if "BUSY" in line else 0})
                continue
            if line == "CAPBEGIN":
                self.cap = []
                continue
            if line == "CAPEND":
                self.emit({"type": "capture", "rows": self.cap or []})
                self.cap = None
                continue
            if self.cap is not None:
                try:
                    self.cap.append([float(x) for x in line.split(",")])
                except ValueError:
                    pass
                continue
            g = GAIN.fullmatch(line)
            if g:
                self.emit({"type": "gain", "name": g.group(1), "value": float(g.group(2))})
                continue
            kv = dict(KV.findall(line))
            if "v" in kv and "Iq" in kv:
                self.emit({"type": "telem", "t": time.monotonic() - t0,
                           "v": float(kv["v"]), "tgt": float(kv.get("tgt", 0)),
                           "iq": float(kv["Iq"]), "id": float(kv.get("Id", 0)),
                           "vq": float(kv.get("Vq", 0)), "sa": float(kv.get("sa", 0)),
                           "trq": float(kv.get("trq", 0)), "hz": float(kv.get("loopHz", 0))})
            else:
                self.emit({"type": "log", "dir": "rx", "text": line})

    def close(self):
        self.stop_flag.set()
        for o in (self.sock, self.ser):
            try:
                o and o.close()
            except Exception:
                pass
        self.sock = self.ser = self.kind = None


async def index(request):
    return web.FileResponse(Path(__file__).parent / "static" / "index.html")


async def ports(request):
    p = [d.device for d in serial.tools.list_ports.comports() if not d.device.startswith("/dev/ttyS")]
    nets = [n for n in socket.if_nameindex()]
    cans = [n for _, n in nets if n.startswith(("can", "vcan", "slcan"))]
    return web.json_response({"serial": sorted(p), "can": cans})


async def ws_handler(request):
    app = request.app
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    app["clients"].add(ws)
    link = STATE.get("link")
    await ws.send_json({"type": "status", "connected": bool(link and link.kind),
                        "kind": link.kind if link else None})
    try:
        async for m in ws:
            if m.type != WSMsgType.TEXT:
                continue
            req = json.loads(m.data)
            try:
                await handle(app, ws, req)
            except Exception as e:
                await ws.send_json({"type": "error", "text": str(e)})
    finally:
        app["clients"].discard(ws)
        # 最後のブラウザが閉じたら安全のため停止コマンドを送る
        link = STATE.get("link")
        if not app["clients"] and link and link.kind and STATE.get("stop_on_close"):
            try:
                link.send("S")
            except Exception:
                pass
    return ws


async def handle(app, ws, req):
    t = req.get("type")
    link = STATE.get("link")
    if t == "connect":
        if link and link.kind:
            link.close()
        link = Link(app)
        if req["mode"] == "can":
            link.open_can(req["iface"], int(req["dev_id"]))
        else:
            link.open_uart(req["port"], int(req["baud"]), int(req["dev_id"]))
        STATE["link"] = link
        STATE["stop_on_close"] = bool(req.get("stop_on_close", True))
        await app["broadcast"]({"type": "status", "connected": True, "kind": link.kind})
    elif t == "disconnect":
        if link and link.kind:
            try:
                link.send("S")
            except Exception:
                pass
            link.close()
        await app["broadcast"]({"type": "status", "connected": False, "kind": None})
    elif t == "cmd":
        link.send(req["text"], bool(req.get("all"))) if link else (_ for _ in ()).throw(RuntimeError("未接続"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="127.0.0.1")
    a = ap.parse_args()
    app = web.Application()
    app["clients"] = set()

    async def broadcast(msg):
        for c in list(app["clients"]):
            try:
                await c.send_json(msg)
            except Exception:
                pass

    async def on_start(app):
        app["loop"] = asyncio.get_running_loop()
    app["broadcast"] = broadcast
    app.on_startup.append(on_start)
    app.router.add_get("/", index)
    app.router.add_get("/api/ports", ports)
    app.router.add_get("/ws", ws_handler)
    print(f"http://{a.host}:{a.port}")
    web.run_app(app, host=a.host, port=a.port, print=None)


if __name__ == "__main__":
    main()
