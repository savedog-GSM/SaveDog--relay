# 실행 전 확인 체크리스트
# 1) Thonny Python에 kocoafab_robodog, websockets, opencv-python이 설치되어 있는지 확인한다.
# 2) 로보독 전원·블루투스 동글·카메라를 연결하고 ROBODOG_PORT와 ROBODOG_CAMERA_INDEX를 확인한다.
# 3) ROBODOG_SERVER를 실제 서버 주소(예: 192.168.0.10:8080)로 설정한다. 주소에 http://를 붙이지 않는다.
# 4) 서버 인증을 사용하면 DEVICE_API_TOKEN을 설정하고 서버의 X-Device-Token 값과 일치하는지 확인한다.
# 5) 서버가 /ws/robot, /api/robot/telemetry, /api/robot/events, /api/robot/hazards를 제공하는지 확인한다.
# 6) /api/robot/hazards가 multipart의 image, robotId, hazardType, detectedAt, source, qrCode, roll, pitch를 받는지 확인한다.
# 7) 스마트폰 GPS가 같은 ROBODOG_ID로 서버에 전송되고 서버가 최신 위치를 위험 신고와 결합하는지 확인한다.
# 8) 보호자 웹에서 위험 위치·사진 확인, 승인·거절, 모의 접수 상태가 표시되는지 확인한다.
# 9) 실제 장비에서 전진 방향, 좌우 회전(LEFT_SPIN/RIGHT_SPIN), 속도, 회전 각도, 경고음을 안전하게 보정한다.
# 10) warning/fire/ground QR로 정지·잠금·경고음·사진 전송을 시험하고 resume 전에는 이동하지 않는지 확인한다.
# 11) 실제 정부 민원 자동 제출은 하지 않으며 보호자 승인 후 서버의 모의 신고 기능만 사용한다.

"""로보독 블루투스 동글과 Spring Boot 서버를 연결하는 PC 중계 프로그램."""

from kocoafab_robodog import *

import json
import os
import queue
import threading
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime

from websockets.exceptions import ConnectionClosed
from websockets.sync.client import connect

try:
    import cv2
except ImportError:
    cv2 = None


SERVER = os.getenv("ROBODOG_SERVER", "localhost:8080").strip()
DEVICE_TOKEN = os.getenv("DEVICE_API_TOKEN", "").strip()
ROBODOG_PORT = os.getenv("ROBODOG_PORT", "").strip() or None
DEFAULT_SPEED = int(os.getenv("ROBODOG_DEFAULT_SPEED", "50"))
LEG_BEND = int(os.getenv("ROBODOG_LEG_BEND", "60"))
TILT_INTERVAL_SECONDS = float(os.getenv("ROBODOG_TILT_INTERVAL", "1.0"))
RECONNECT_SECONDS = float(os.getenv("ROBODOG_RECONNECT_SECONDS", "5.0"))
HTTP_TIMEOUT_SECONDS = float(os.getenv("ROBODOG_HTTP_TIMEOUT", "3.0"))
EVENT_URL = os.getenv(
    "ROBODOG_EVENT_URL", "http://{}/api/robot/events".format(SERVER)
).strip()
HAZARD_URL = os.getenv(
    "ROBODOG_HAZARD_URL", "http://{}/api/robot/hazards".format(SERVER)
).strip()
ROBOT_ID = os.getenv("ROBODOG_ID", "robodog-01").strip()
CAMERA_INDEX = int(os.getenv("ROBODOG_CAMERA_INDEX", "0"))
CAMERA_JPEG_QUALITY = int(os.getenv("ROBODOG_CAMERA_JPEG_QUALITY", "80"))
HAZARD_COOLDOWN_SECONDS = float(os.getenv("ROBODOG_HAZARD_COOLDOWN", "300"))
QR_TURN_DEGREES = float(os.getenv("ROBODOG_QR_TURN_DEGREES", "90"))

# 실제 회전 방향이 반대라면 두 값을 서로 바꾼다.
LEFT_SPIN = CCW
RIGHT_SPIN = CW

SUPPORTED_COMMANDS = {
    "STOP",
    "EMERGENCY_STOP",
    "RESUME",
    "BARK",
    "FORWARD",
    "BACKWARD",
    "TURN_LEFT",
    "TURN_RIGHT",
}

QR_COMMANDS = {
    "start": [{"type": "FORWARD"}],
    "stop": [{"type": "EMERGENCY_STOP"}],
    "resume": [{"type": "RESUME"}],
    "left": [
        {"type": "STOP"},
        {"type": "TURN_LEFT", "degrees": QR_TURN_DEGREES},
        {"type": "FORWARD"},
    ],
    "right": [
        {"type": "STOP"},
        {"type": "TURN_RIGHT", "degrees": QR_TURN_DEGREES},
        {"type": "FORWARD"},
    ],
    "warning": [{"type": "EMERGENCY_STOP"}, {"type": "BARK"}],
    "fire": [{"type": "EMERGENCY_STOP"}, {"type": "BARK"}],
    "ground": [{"type": "EMERGENCY_STOP"}, {"type": "BARK"}],
    "reroute": [{"type": "STOP"}],
    "cross": [{"type": "STOP"}],
    "arrive": [{"type": "EMERGENCY_STOP"}],
    "report": [],
}

HAZARD_TYPES = {
    "warning": "GENERAL_DANGER",
    "fire": "FIRE",
    "ground": "GROUND_DAMAGE",
}

stopping = threading.Event()
movement_locked = threading.Event()
hazard_report_lock = threading.Lock()


def log(message):
    print(datetime.now().strftime("%H:%M:%S"), message, flush=True)


def describe(error):
    return "{}: {}".format(type(error).__name__, error)


def validate_settings():
    if not SERVER:
        raise ValueError("ROBODOG_SERVER가 비어 있습니다.")
    if not ROBOT_ID:
        raise ValueError("ROBODOG_ID가 비어 있습니다.")
    if not 0 <= DEFAULT_SPEED <= 100:
        raise ValueError("ROBODOG_DEFAULT_SPEED는 0~100 범위여야 합니다.")
    if not 0 <= LEG_BEND <= 100:
        raise ValueError("ROBODOG_LEG_BEND는 0~100 범위여야 합니다.")
    if min(TILT_INTERVAL_SECONDS, RECONNECT_SECONDS, HTTP_TIMEOUT_SECONDS) <= 0:
        raise ValueError("시간 관련 설정은 0보다 커야 합니다.")
    if HAZARD_COOLDOWN_SECONDS < 0:
        raise ValueError("ROBODOG_HAZARD_COOLDOWN은 0 이상이어야 합니다.")
    if not 0 < QR_TURN_DEGREES <= 360:
        raise ValueError("ROBODOG_QR_TURN_DEGREES는 0~360 범위여야 합니다.")
    if not 1 <= CAMERA_JPEG_QUALITY <= 100:
        raise ValueError("ROBODOG_CAMERA_JPEG_QUALITY는 1~100 범위여야 합니다.")


class Robot:
    """로보독 SDK 호출을 한곳에 모은다."""

    def __init__(self, dog):
        self.dog = dog
        self.lock = threading.RLock()

    def stop(self):
        with self.lock:
            self.dog.move(STOP)
            self.dog.spin(STOP)

    def move(self, direction, speed):
        with self.lock:
            self.dog.move(STOP)
            self.dog.spin(STOP)
            self.dog.move(direction, speed)

    def spin(self, direction, speed):
        with self.lock:
            self.dog.move(STOP)
            self.dog.spin(STOP)
            self.dog.spin(direction, speed)

    def rotate(self, direction, degrees, speed):
        # rotate()는 완료까지 기다리므로 외부 잠금을 오래 잡지 않는다.
        self.stop()
        self.dog.rotate(direction, degrees, speed)

    def bark(self):
        with self.lock:
            self.dog.sound(멍멍, 크게)

    def tilt(self):
        value = self.dog.get_tilt()
        if value is None or len(value) < 2:
            return None
        roll, pitch = float(value[0]), float(value[1])
        if not (-180 <= roll <= 180 and -180 <= pitch <= 180):
            raise ValueError(
                "기울기 범위 오류: roll={}, pitch={}".format(roll, pitch)
            )
        return roll, pitch


class CommandWorker(threading.Thread):
    """일반 명령을 순서대로 실행하고 시간제 이동을 관리한다."""

    def __init__(self, robot, send_result):
        super().__init__(daemon=True, name="command-worker")
        self.robot = robot
        self.send_result = send_result
        self.commands = queue.Queue()

    def add(self, command):
        self.commands.put(command)

    def clear(self):
        while True:
            try:
                self.commands.get_nowait()
                self.commands.task_done()
            except queue.Empty:
                return

    def run(self):
        stop_at = None
        while not stopping.is_set():
            timeout = 0.5
            if stop_at is not None:
                timeout = min(0.5, max(0.0, stop_at - time.monotonic()))
            command = self._next(timeout)
            if command is None:
                if stop_at is not None and time.monotonic() >= stop_at:
                    self._safe_stop()
                    stop_at = None
                continue

            try:
                changes_motion, seconds = self._execute(command)
                result = "OK"
            except Exception as error:
                changes_motion, seconds = True, None
                self._safe_stop()
                result = describe(error)
                log("명령 실행 실패: {}".format(result))
            finally:
                self.commands.task_done()

            if changes_motion:
                stop_at = None if seconds is None else time.monotonic() + seconds
            self.send_result(command.get("commandId"), result)

    def _next(self, timeout):
        try:
            return self.commands.get(timeout=timeout)
        except queue.Empty:
            return None

    def _execute(self, command):
        kind = command["type"]
        speed = DEFAULT_SPEED if command.get("speed") is None else int(command["speed"])
        if not 0 <= speed <= 100:
            raise ValueError("speed는 0~100 범위여야 합니다.")
        duration = command.get("durationMs")
        seconds = None if duration is None else float(duration) / 1000
        if seconds is not None and seconds < 0:
            raise ValueError("durationMs는 0 이상이어야 합니다.")
        degrees = command.get("degrees")
        if degrees is not None:
            degrees = float(degrees)
            if not 0 < degrees <= 360:
                raise ValueError("degrees는 0보다 크고 360 이하여야 합니다.")

        if kind == "BARK":
            self.robot.bark()
            return False, None
        if kind == "FORWARD":
            self.robot.move(FORWARD, speed)
        elif kind == "BACKWARD":
            self.robot.move(BACKWARD, speed)
        elif kind == "TURN_LEFT":
            if degrees is None:
                self.robot.spin(LEFT_SPIN, speed)
            else:
                self.robot.rotate(LEFT_SPIN, degrees, speed)
        elif kind == "TURN_RIGHT":
            if degrees is None:
                self.robot.spin(RIGHT_SPIN, speed)
            else:
                self.robot.rotate(RIGHT_SPIN, degrees, speed)
        else:
            raise ValueError("모르는 명령입니다: {}".format(kind))
        log("명령 {} (속도 {})".format(kind, speed))
        return True, seconds

    def _safe_stop(self):
        try:
            self.robot.stop()
        except Exception as error:
            log("로보독 정지 실패: {}".format(describe(error)))


class TiltSender(threading.Thread):
    """기울기를 일정 간격으로 서버에 전송한다."""

    def __init__(self, robot):
        super().__init__(daemon=True, name="tilt-sender")
        self.robot = robot
        self.problem = None

    def run(self):
        while not stopping.wait(TILT_INTERVAL_SECONDS):
            self._report(self._send_once())

    def _send_once(self):
        try:
            tilt = self.robot.tilt()
        except Exception as error:
            return "기울기 읽기 실패 ({})".format(describe(error))
        if tilt is None:
            return "로보독이 아직 기울기 값을 주지 않습니다."

        body = json.dumps(
            {"sensor": {"roll": tilt[0], "pitch": tilt[1]}}
        ).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if DEVICE_TOKEN:
            headers["X-Device-Token"] = DEVICE_TOKEN
        request = urllib.request.Request(
            "http://{}/api/robot/telemetry".format(SERVER),
            data=body,
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=HTTP_TIMEOUT_SECONDS
            ) as response:
                response.read()
            return None
        except urllib.error.HTTPError as error:
            return "서버가 기울기를 받지 않았습니다 (HTTP {})".format(error.code)
        except OSError as error:
            return "기울기 전송 실패 ({})".format(error)

    def _report(self, problem):
        if problem == self.problem:
            return
        log("기울기 전송 정상" if problem is None else problem)
        self.problem = problem


def send_qr_event(qr_code, commands):
    """보호자 웹이 상태를 갱신할 수 있도록 QR 인식 이벤트를 서버에 알린다."""
    if not EVENT_URL:
        return
    body = json.dumps(
        {
            "eventType": "QR_DETECTED",
            "qrCode": qr_code,
            "commands": [command["type"] for command in commands],
            "recordedAt": datetime.now().isoformat(),
        }
    ).encode("utf-8")
    headers = {"Content-Type": "application/json"}
    if DEVICE_TOKEN:
        headers["X-Device-Token"] = DEVICE_TOKEN
    request = urllib.request.Request(
        EVENT_URL, data=body, headers=headers, method="POST"
    )
    try:
        with urllib.request.urlopen(
            request, timeout=HTTP_TIMEOUT_SECONDS
        ) as response:
            response.read()
    except (urllib.error.HTTPError, OSError) as error:
        log("QR 이벤트 전송 실패: {}".format(error))


def _multipart_body(fields, image_bytes):
    boundary = "----RoboDog{}".format(uuid.uuid4().hex)
    chunks = []
    for name, value in fields.items():
        chunks.extend(
            [
                "--{}\r\n".format(boundary).encode("ascii"),
                (
                    'Content-Disposition: form-data; name="{}"\r\n\r\n'.format(
                        name
                    )
                ).encode("utf-8"),
                str(value).encode("utf-8"),
                b"\r\n",
            ]
        )
    chunks.extend(
        [
            "--{}\r\n".format(boundary).encode("ascii"),
            b'Content-Disposition: form-data; name="image"; filename="hazard.jpg"\r\n',
            b"Content-Type: image/jpeg\r\n\r\n",
            image_bytes,
            b"\r\n",
            "--{}--\r\n".format(boundary).encode("ascii"),
        ]
    )
    return b"".join(chunks), boundary


def create_hazard_report(qr_code, robot):
    """위험 사진과 센서 값을 서버에 보내 보호자 검토 대기 건을 만든다."""
    if not HAZARD_URL:
        return
    if cv2 is None:
        log("위험 신고 생략: OpenCV(cv2)가 설치되어 있지 않습니다.")
        return

    # 여러 위험 이벤트가 겹쳐도 카메라는 한 작업만 사용한다.
    with hazard_report_lock:
        camera = cv2.VideoCapture(CAMERA_INDEX)
        try:
            if not camera.isOpened():
                log("사진 촬영 실패: 카메라 {}을 열 수 없습니다.".format(CAMERA_INDEX))
                return
            ok, frame = camera.read()
            if not ok:
                log("사진 촬영 실패: 카메라 프레임을 읽지 못했습니다.")
                return
            ok, jpeg = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), CAMERA_JPEG_QUALITY],
            )
            if not ok:
                log("사진 촬영 실패: JPEG 변환에 실패했습니다.")
                return

            fields = {
                "robotId": ROBOT_ID,
                "hazardType": HAZARD_TYPES[qr_code],
                "detectedAt": datetime.now().astimezone().isoformat(),
                "source": "QR_DEMO",
                "qrCode": qr_code,
            }
            try:
                tilt = robot.tilt()
                if tilt is not None:
                    fields["roll"] = tilt[0]
                    fields["pitch"] = tilt[1]
            except Exception as error:
                log("기울기 없이 신고를 전송합니다: {}".format(describe(error)))
            body, boundary = _multipart_body(fields, jpeg.tobytes())
            headers = {
                "Content-Type": "multipart/form-data; boundary={}".format(boundary)
            }
            if DEVICE_TOKEN:
                headers["X-Device-Token"] = DEVICE_TOKEN
            request = urllib.request.Request(
                HAZARD_URL, data=body, headers=headers, method="POST"
            )
            with urllib.request.urlopen(
                request, timeout=HTTP_TIMEOUT_SECONDS
            ) as response:
                response_body = response.read().decode("utf-8", errors="replace")
            log("위험 신고 후보 생성: {} {}".format(qr_code, response_body))
        except (urllib.error.HTTPError, OSError, KeyError, ValueError) as error:
            log("위험 신고 전송 실패: {}".format(error))
        except Exception as error:
            log("위험 신고 처리 실패: {}".format(describe(error)))
        finally:
            camera.release()


class QRReader(threading.Thread):
    """QR을 읽어 서버 명령과 같은 형식으로 중계 프로그램에 전달한다."""

    def __init__(self, dog, server_link):
        super().__init__(daemon=True, name="qr-reader")
        self.dog = dog
        self.server_link = server_link
        self.last_hazard_at = {}

    def run(self):
        handled_qr = None
        missing_count = 0
        while not stopping.is_set():
            try:
                qr_code = self.dog.get_qrcode()
            except Exception as error:
                log("QR 읽기 실패: {}".format(describe(error)))
                stopping.wait(0.5)
                continue

            if qr_code:
                qr_code = str(qr_code).strip().lower()
                missing_count = 0
                if qr_code != handled_qr:
                    handled_qr = qr_code
                    commands = QR_COMMANDS.get(qr_code)
                    if commands is None:
                        log("지원하지 않는 QR: {}".format(qr_code))
                        send_qr_event(qr_code, [])
                    else:
                        log("QR 인식: {}".format(qr_code))
                        for command in commands:
                            self.server_link.submit(command)
                        send_qr_event(qr_code, commands)
                        if qr_code in HAZARD_TYPES:
                            now = time.monotonic()
                            last = self.last_hazard_at.get(qr_code, float("-inf"))
                            if now - last >= HAZARD_COOLDOWN_SECONDS:
                                self.last_hazard_at[qr_code] = now
                                threading.Thread(
                                    target=create_hazard_report,
                                    args=(qr_code, self.server_link.robot),
                                    daemon=True,
                                    name="hazard-report-{}".format(qr_code),
                                ).start()
                            else:
                                log("중복 위험 신고 생략: {}".format(qr_code))
            else:
                missing_count += 1
                if missing_count >= 10:
                    handled_qr = None
            stopping.wait(0.05)


class ServerLink:
    """명령 WebSocket을 유지하고 끊기면 정지 후 재접속한다."""

    def __init__(self, robot):
        self.robot = robot
        self.socket = None
        self.send_lock = threading.Lock()
        self.worker = CommandWorker(robot, self.send_result)

    def send_result(self, command_id, result):
        if command_id is None:
            return
        with self.send_lock:
            if self.socket is None:
                return
            try:
                self.socket.send(
                    json.dumps({"commandId": command_id, "result": result})
                )
            except Exception:
                pass

    def run(self):
        self.worker.start()
        url = "ws://{}/ws/robot".format(SERVER)
        headers = {"X-Device-Token": DEVICE_TOKEN} if DEVICE_TOKEN else None
        while not stopping.is_set():
            try:
                with connect(
                    url, additional_headers=headers, open_timeout=5
                ) as connection:
                    self.socket = connection
                    log("서버 연결: {}".format(url))
                    while not stopping.is_set():
                        try:
                            message = connection.recv(timeout=1)
                        except TimeoutError:
                            continue
                        self._receive(message)
            except ConnectionClosed:
                log("서버 연결이 끊겼습니다.")
            except Exception as error:
                log("서버 연결 실패: {}".format(describe(error)))
            finally:
                self.socket = None
                self.worker.clear()
                self._safe_stop()
            stopping.wait(RECONNECT_SECONDS)

    def _receive(self, message):
        try:
            command = json.loads(message)
        except (TypeError, ValueError):
            log("서버 메시지를 해석하지 못했습니다: {!r}".format(message))
            return
        if not isinstance(command, dict) or "type" not in command:
            log("명령이 아닌 메시지: {!r}".format(message))
            return

        self.submit(command)

    def submit(self, command):
        """서버와 QR이 공통으로 사용하는 명령 진입점."""
        kind = str(command.get("type", "")).upper()
        command = dict(command)
        command["type"] = kind
        if kind not in SUPPORTED_COMMANDS:
            self.send_result(
                command.get("commandId"),
                "ValueError: 모르는 명령입니다: {}".format(kind),
            )
            return

        # 일반 정지는 현재 동작만 멈춘다. 비상 정지는 RESUME 전까지 이동을 막는다.
        if kind in {"STOP", "EMERGENCY_STOP"}:
            self.worker.clear()
            if kind == "EMERGENCY_STOP":
                movement_locked.set()
            try:
                self.robot.stop()
                result = "OK"
                log("명령 {} (즉시 실행)".format(kind))
            except Exception as error:
                result = describe(error)
            self.send_result(command.get("commandId"), result)
            return

        if kind == "RESUME":
            movement_locked.clear()
            command["type"] = "FORWARD"
            log("안전 잠금 해제 후 전진")

        if command["type"] in {
            "FORWARD",
            "BACKWARD",
            "TURN_LEFT",
            "TURN_RIGHT",
        } and movement_locked.is_set():
            result = "RuntimeError: 안전 잠금 상태입니다. RESUME이 필요합니다."
            log(result)
            self.send_result(command.get("commandId"), result)
            return
        self.worker.add(command)

    def _safe_stop(self):
        try:
            self.robot.stop()
        except Exception as error:
            log("로보독 정지 실패: {}".format(describe(error)))


def main():
    validate_settings()
    dog = RoboDog()
    opened = dog.Open() if ROBODOG_PORT is None else dog.Open(ROBODOG_PORT)
    if not opened:
        log("로보독 연결 실패: 동글과 전원을 확인하세요.")
        return

    log("로보독에 연결했습니다.")
    robot = Robot(dog)
    try:
        mode_response = dog.ai_mode(QRCODE_READ, timeout=5.0)
        log("QR 모드 응답: {}".format(mode_response))
        dog.leg_bend(ALL_LEG, LEG_BEND)
        server_link = ServerLink(robot)
        TiltSender(robot).start()
        QRReader(dog, server_link).start()
        server_link.run()
    except KeyboardInterrupt:
        log("종료합니다.")
    finally:
        stopping.set()
        try:
            robot.stop()
        finally:
            dog.Close()
            log("로보독 연결을 닫았습니다.")


if __name__ == "__main__":
    main()
