# 실행 전 확인 체크리스트
# 1) Thonny Python에 kocoafab_robodog, websockets, opencv-python이 설치되어 있는지 확인한다.
# 2) 로보독 전원·블루투스 동글·카메라를 연결하고 ROBODOG_PORT와 ROBODOG_CAMERA_INDEX를 확인한다.
# 3) ROBODOG_SERVER를 실제 서버 주소(예: 192.168.0.10:8080)로 설정한다. 주소에 http://를 붙이지 않는다.
# 4) 서버 인증을 사용하면 DEVICE_API_TOKEN을 설정하고 서버의 X-Device-Token 값과 일치하는지 확인한다.
# 5) 서버가 /ws/robot, /api/robot/telemetry, /api/robot/events, /api/robot/hazards를 제공하는지 확인한다.
# 6) /api/robot/hazards가 multipart의 image, robotId, hazardType, detectedAt, source, roll, pitch를 받는지 확인한다.
# 7) 스마트폰 GPS가 같은 ROBODOG_ID로 서버에 전송되고 서버가 최신 위치를 위험 신고와 결합하는지 확인한다.
# 8) 보호자 웹에서 위험 위치·사진 확인, 승인·거절, 모의 접수 상태가 표시되는지 확인한다.
# 9) 실제 장비에서 전진 방향, 좌우 회전(LEFT_SPIN/RIGHT_SPIN), 속도, 회전 각도, 경고음을 안전하게 보정한다.
# 10) 거리·기울기 위험 감지 시 정지·잠금·경고음·사진 전송을 시험하고 resume 전에는 이동하지 않는지 확인한다.
# 11) 실제 정부 민원 자동 제출은 하지 않으며 보호자 승인 후 서버의 모의 신고 기능만 사용한다.
# 12) 거리·기울기 안전 기준과 연속 감지 횟수를 실제 장비에서 보수적으로 조정한다.

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


SERVER = os.getenv("ROBODOG_SERVER", "savedog.https.gsmsv.site").strip()
DEVICE_TOKEN = "d8ea5a1949aedf113f5f618b47d6ad0ee8da51bef9598b09"
ROBODOG_PORT = os.getenv("ROBODOG_PORT", "").strip() or None
DEFAULT_SPEED = int(os.getenv("ROBODOG_DEFAULT_SPEED", "50"))
LEG_BEND = int(os.getenv("ROBODOG_LEG_BEND", "60"))
TILT_INTERVAL_SECONDS = float(os.getenv("ROBODOG_TILT_INTERVAL", "1.0"))
SAFETY_INTERVAL_SECONDS = float(os.getenv("ROBODOG_SAFETY_INTERVAL", "0.05"))
RECONNECT_SECONDS = float(os.getenv("ROBODOG_RECONNECT_SECONDS", "5.0"))
HTTP_TIMEOUT_SECONDS = float(os.getenv("ROBODOG_HTTP_TIMEOUT", "3.0"))
EVENT_URL = os.getenv(
    "ROBODOG_EVENT_URL", "https://{}/api/robot/events".format(SERVER)
).strip()
HAZARD_URL = os.getenv(
    "ROBODOG_HAZARD_URL", "https://{}/api/robot/hazards".format(SERVER)
).strip()
CAMERA_UPLOAD_URL = os.getenv("ROBODOG_CAMERA_URL", "").strip()
TELEMETRY_URL = os.getenv(
    "ROBODOG_TELEMETRY_URL", "https://{}/api/robot/telemetry".format(SERVER)
).strip()
COMMAND_WS_URL = os.getenv(
    "ROBODOG_WS_URL", "wss://{}/ws/robot".format(SERVER)
).strip()
ROBOT_ID = os.getenv("ROBODOG_ID", "robodog-01").strip()
CAMERA_INDEX = int(os.getenv("ROBODOG_CAMERA_INDEX", "0"))
CAMERA_JPEG_QUALITY = int(os.getenv("ROBODOG_CAMERA_JPEG_QUALITY", "80"))
DISTANCE_WARNING_CM = float(os.getenv("ROBODOG_DISTANCE_WARNING_CM", "50"))
DISTANCE_DANGER_CM = float(os.getenv("ROBODOG_DISTANCE_DANGER_CM", "20"))
TILT_WARNING_DEG = float(os.getenv("ROBODOG_TILT_WARNING_DEG", "15"))
TILT_DANGER_DEG = float(os.getenv("ROBODOG_TILT_DANGER_DEG", "30"))
DANGER_CONFIRM_COUNT = int(os.getenv("ROBODOG_DANGER_CONFIRM_COUNT", "3"))
BARK_THEME = int(os.getenv("ROBODOG_BARK_THEME", "0"))
BARK_TRACK = int(os.getenv("ROBODOG_BARK_TRACK", "1"))
BARK_VOLUME = int(os.getenv("ROBODOG_BARK_VOLUME", "100"))
BARK_COOLDOWN_SECONDS = float(os.getenv("ROBODOG_BARK_COOLDOWN", "5.0"))

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

IDLE = "IDLE"
MOVING = "MOVING"
STOPPED = "STOPPED"
TURNING = "TURNING"
HAZARD = "HAZARD"

stopping = threading.Event()
movement_locked = threading.Event()
hazard_report_lock = threading.Lock()
runtime_lock = threading.RLock()
event_queue = queue.Queue()

runtime_state = {
    "robotState": IDLE,
    "speed": 0,
    "websocketConnected": False,
    "riskLevel": "safe",
    "riskScore": 0,
    "riskReason": "normal",
}


def log(message):
    print(datetime.now().strftime("%H:%M:%S"), message, flush=True)


def describe(error):
    return "{}: {}".format(type(error).__name__, error)


def update_runtime(**changes):
    with runtime_lock:
        runtime_state.update(changes)


def runtime_snapshot():
    with runtime_lock:
        return dict(runtime_state)


def queue_event(event_type, **details):
    """로봇 제어를 막지 않도록 서버 이벤트를 별도 전송 큐에 넣는다."""
    snapshot = runtime_snapshot()
    payload = {
        "eventType": event_type,
        "robotId": ROBOT_ID,
        "robotState": snapshot["robotState"],
        "speed": snapshot["speed"],
        "riskLevel": snapshot["riskLevel"],
        "riskScore": snapshot["riskScore"],
        "riskReason": snapshot["riskReason"],
        "details": details,
        "recordedAt": datetime.now().astimezone().isoformat(),
    }
    payload.update(details)
    event_queue.put(payload)


def validate_settings():
    if not SERVER:
        raise ValueError("ROBODOG_SERVER가 비어 있습니다.")
    if not ROBOT_ID:
        raise ValueError("ROBODOG_ID가 비어 있습니다.")
    if not TELEMETRY_URL or not COMMAND_WS_URL:
        raise ValueError("텔레메트리 또는 WebSocket 주소가 비어 있습니다.")
    if not 0 <= DEFAULT_SPEED <= 100:
        raise ValueError("ROBODOG_DEFAULT_SPEED는 0~100 범위여야 합니다.")
    if not 0 <= LEG_BEND <= 100:
        raise ValueError("ROBODOG_LEG_BEND는 0~100 범위여야 합니다.")
    if min(
        TILT_INTERVAL_SECONDS,
        SAFETY_INTERVAL_SECONDS,
        RECONNECT_SECONDS,
        HTTP_TIMEOUT_SECONDS,
    ) <= 0:
        raise ValueError("시간 관련 설정은 0보다 커야 합니다.")
    if not 1 <= CAMERA_JPEG_QUALITY <= 100:
        raise ValueError("ROBODOG_CAMERA_JPEG_QUALITY는 1~100 범위여야 합니다.")
    if not 0 < DISTANCE_DANGER_CM <= DISTANCE_WARNING_CM:
        raise ValueError("거리 위험 기준은 경고 기준 이하여야 합니다.")
    if not 0 < TILT_WARNING_DEG <= TILT_DANGER_DEG:
        raise ValueError("기울기 경고 기준은 위험 기준 이하여야 합니다.")
    if DANGER_CONFIRM_COUNT < 1:
        raise ValueError("연속 위험 감지 횟수는 1 이상이어야 합니다.")
    if not 0 <= BARK_VOLUME <= 100 or BARK_COOLDOWN_SECONDS < 0:
        raise ValueError("경고음 설정 범위가 올바르지 않습니다.")


class Robot:
    """로보독 SDK 호출을 한곳에 모은다."""

    def __init__(self, dog):
        self.dog = dog
        self.lock = threading.RLock()
        self.last_bark_at = float("-inf")

    def stop(self):
        with self.lock:
            self.dog.move(STOP)
            self.dog.spin(STOP)
        update_runtime(robotState=STOPPED, speed=0)

    def move(self, direction, speed):
        with self.lock:
            self.dog.move(STOP)
            self.dog.spin(STOP)
            self.dog.move(direction, speed)
        update_runtime(robotState=MOVING, speed=speed)

    def spin(self, direction, speed):
        with self.lock:
            self.dog.move(STOP)
            self.dog.spin(STOP)
            self.dog.spin(direction, speed)
        update_runtime(robotState=TURNING, speed=speed)

    def rotate(self, direction, degrees, speed):
        # rotate()는 완료까지 기다리므로 외부 잠금을 오래 잡지 않는다.
        self.stop()
        update_runtime(robotState=TURNING, speed=speed)
        self.dog.rotate(direction, degrees, speed)
        update_runtime(robotState=STOPPED, speed=0)

    def bark(self):
        with self.lock:
            now = time.monotonic()
            if now - self.last_bark_at < BARK_COOLDOWN_SECONDS:
                return False
            self.dog.sound(BARK_THEME, BARK_TRACK, BARK_VOLUME)
            self.last_bark_at = now
            return True

    def tilt(self):
        with self.lock:
            value = self.dog.get_tilt()
        if value is None or len(value) < 2:
            return None
        roll, pitch = float(value[0]), float(value[1])
        if not (-180 <= roll <= 180 and -180 <= pitch <= 180):
            raise ValueError(
                "기울기 범위 오류: roll={}, pitch={}".format(roll, pitch)
            )
        return roll, pitch

    def distance(self):
        with self.lock:
            value = self.dog.get_distance()
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            if not value:
                return None
            value = value[0]
        distance = float(value)
        if distance < 0:
            raise ValueError("거리 값은 0 이상이어야 합니다: {}".format(distance))
        return distance

    def sensors(self):
        tilt = self.tilt()
        return {
            "distance": self.distance(),
            "roll": None if tilt is None else tilt[0],
            "pitch": None if tilt is None else tilt[1],
        }


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
                    queue_event("MOVEMENT_DURATION_ENDED")
                    stop_at = None
                continue

            try:
                changes_motion, seconds = self._execute(command)
                result = "OK"
                queue_event("COMMAND_COMPLETED", commandType=command["type"])
            except Exception as error:
                changes_motion, seconds = True, None
                self._safe_stop()
                result = describe(error)
                log("명령 실행 실패: {}".format(result))
                queue_event(
                    "COMMAND_FAILED",
                    commandType=command.get("type"),
                    error=result,
                )
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


class TelemetrySender(threading.Thread):
    """거리·기울기·동작·위험 상태를 일정 간격으로 서버에 전송한다."""

    def __init__(self, robot):
        super().__init__(daemon=True, name="telemetry-sender")
        self.robot = robot
        self.problem = None

    def run(self):
        while not stopping.wait(TILT_INTERVAL_SECONDS):
            self._report(self._send_once())

    def _send_once(self):
        try:
            sensors = self.robot.sensors()
        except Exception as error:
            return "센서 읽기 실패 ({})".format(describe(error))

        snapshot = runtime_snapshot()
        body = json.dumps(
            {
                "robotId": ROBOT_ID,
                "robotState": snapshot["robotState"],
                "speed": snapshot["speed"],
                "websocketConnected": snapshot["websocketConnected"],
                "sensor": sensors,
                "risk": {
                    "level": snapshot["riskLevel"],
                    "score": snapshot["riskScore"],
                    "reason": snapshot["riskReason"],
                },
                "recordedAt": datetime.now().astimezone().isoformat(),
            }
        ).encode("utf-8")
        headers = {"Content-Type": "application/json"}
        if DEVICE_TOKEN:
            headers["X-Device-Token"] = DEVICE_TOKEN
        request = urllib.request.Request(
            TELEMETRY_URL,
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
            return "서버가 텔레메트리를 받지 않았습니다 (HTTP {})".format(error.code)
        except OSError as error:
            return "텔레메트리 전송 실패 ({})".format(error)

    def _report(self, problem):
        if problem == self.problem:
            return
        log("텔레메트리 전송 정상" if problem is None else problem)
        self.problem = problem


class EventSender(threading.Thread):
    """제어·안전 이벤트를 큐에서 꺼내 서버로 전송한다."""

    def __init__(self):
        super().__init__(daemon=True, name="event-sender")
        self.problem = None

    def run(self):
        while not stopping.is_set() or not event_queue.empty():
            try:
                payload = event_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._send(payload)
                self._report(None)
            except urllib.error.HTTPError as error:
                self._report("이벤트 전송 거부 (HTTP {})".format(error.code))
            except OSError as error:
                self._report("이벤트 전송 실패 ({})".format(error))
            finally:
                event_queue.task_done()

    def _send(self, payload):
        if not EVENT_URL:
            return
        headers = {"Content-Type": "application/json"}
        if DEVICE_TOKEN:
            headers["X-Device-Token"] = DEVICE_TOKEN
        request = urllib.request.Request(
            EVENT_URL,
            data=json.dumps(payload).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(
            request, timeout=HTTP_TIMEOUT_SECONDS
        ) as response:
            response.read()

    def _report(self, problem):
        if problem == self.problem:
            return
        log("이벤트 전송 정상" if problem is None else problem)
        self.problem = problem


class SafetyMonitor(threading.Thread):
    """거리와 기울기를 감시하고 연속 위험이 확인되면 즉시 정지한다."""

    def __init__(self, robot, server_link):
        super().__init__(daemon=True, name="safety-monitor")
        self.robot = robot
        self.server_link = server_link
        self.danger_count = 0
        self.danger_reported = False
        self.problem = None

    def run(self):
        while not stopping.wait(SAFETY_INTERVAL_SECONDS):
            try:
                self._assess()
                self._report_problem(None)
            except Exception as error:
                self._report_problem("안전 센서 확인 실패 ({})".format(describe(error)))

    def _assess(self):
        sensors = self.robot.sensors()
        distance = sensors["distance"]
        tilt_values = [
            abs(value)
            for value in (sensors["roll"], sensors["pitch"])
            if value is not None
        ]
        maximum_tilt = max(tilt_values) if tilt_values else 0.0
        valid_distance = distance is not None and distance > 0

        danger_reasons = []
        if valid_distance and distance <= DISTANCE_DANGER_CM:
            danger_reasons.append("obstacle:{}cm".format(distance))
        if maximum_tilt >= TILT_DANGER_DEG:
            danger_reasons.append("tilt:{}deg".format(maximum_tilt))

        if danger_reasons:
            self.danger_count += 1
            reason = ",".join(danger_reasons)
            update_runtime(riskLevel="danger", riskScore=100, riskReason=reason)
            if self.danger_count >= DANGER_CONFIRM_COUNT and not self.danger_reported:
                self.danger_reported = True
                self._trigger_stop(reason, sensors)
            return

        self.danger_count = 0
        self.danger_reported = False
        warning_reasons = []
        if valid_distance and distance <= DISTANCE_WARNING_CM:
            warning_reasons.append("obstacle:{}cm".format(distance))
        if maximum_tilt >= TILT_WARNING_DEG:
            warning_reasons.append("tilt:{}deg".format(maximum_tilt))
        update_runtime(
            riskLevel="warning" if warning_reasons else "safe",
            riskScore=50 if warning_reasons else 0,
            riskReason=",".join(warning_reasons) if warning_reasons else "normal",
        )

    def _trigger_stop(self, reason, sensors):
        snapshot = runtime_snapshot()
        if snapshot["robotState"] not in {MOVING, TURNING}:
            return
        movement_locked.set()
        self.server_link.worker.clear()
        self.robot.stop()
        update_runtime(robotState=HAZARD)
        barked = self.robot.bark()
        log("센서 위험 감지, 비상 정지: {}".format(reason))
        queue_event(
            "SENSOR_HAZARD_DETECTED",
            reason=reason,
            sensors=sensors,
            barked=barked,
        )
        threading.Thread(
            target=create_hazard_report,
            args=(self.robot,),
            daemon=True,
            name="sensor-hazard-report",
        ).start()
        if CAMERA_UPLOAD_URL:
            threading.Thread(
                target=upload_snapshot,
                args=("sensor_hazard_detected",),
                daemon=True,
                name="sensor-snapshot",
            ).start()

    def _report_problem(self, problem):
        if problem == self.problem:
            return
        if problem is not None:
            log(problem)
        elif self.problem is not None:
            log("안전 센서 확인 정상")
        self.problem = problem



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


def create_hazard_report(robot, source="SENSOR"):
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
                "hazardType": "GENERAL_DANGER",
                "detectedAt": datetime.now().astimezone().isoformat(),
                "source": source,
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
            log("센서 위험 신고 후보 생성: {}".format(response_body))
        except (urllib.error.HTTPError, OSError, KeyError, ValueError) as error:
            log("위험 신고 전송 실패: {}".format(error))
        except Exception as error:
            log("위험 신고 처리 실패: {}".format(describe(error)))
        finally:
            camera.release()


def upload_snapshot(event_type):
    """별도 카메라 업로드 주소가 설정된 경우 이벤트 사진 한 장을 전송한다."""
    if not CAMERA_UPLOAD_URL:
        return
    if cv2 is None:
        log("사진 업로드 생략: OpenCV(cv2)가 설치되어 있지 않습니다.")
        return
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
            headers = {
                "Content-Type": "image/jpeg",
                "X-Event-Type": event_type,
                "X-Robot-ID": ROBOT_ID,
            }
            if DEVICE_TOKEN:
                headers["X-Device-Token"] = DEVICE_TOKEN
            request = urllib.request.Request(
                CAMERA_UPLOAD_URL,
                data=jpeg.tobytes(),
                headers=headers,
                method="POST",
            )
            with urllib.request.urlopen(
                request, timeout=HTTP_TIMEOUT_SECONDS
            ) as response:
                response.read()
            log("이벤트 사진 전송 완료: {}".format(event_type))
        except (urllib.error.HTTPError, OSError, ValueError) as error:
            log("이벤트 사진 전송 실패: {}".format(error))
        finally:
            camera.release()


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
        url = COMMAND_WS_URL
        headers = {"X-Device-Token": DEVICE_TOKEN} if DEVICE_TOKEN else None
        while not stopping.is_set():
            try:
                with connect(
                    url, additional_headers=headers, open_timeout=5
                ) as connection:
                    self.socket = connection
                    update_runtime(websocketConnected=True)
                    log("서버 연결: {}".format(url))
                    queue_event("SERVER_CONNECTED")
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
                was_connected = runtime_snapshot()["websocketConnected"]
                update_runtime(websocketConnected=False)
                self.worker.clear()
                self._safe_stop()
                if was_connected:
                    queue_event("SERVER_DISCONNECTED")
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
        """서버 명령을 검증하고 실행 대기열에 넣는다."""
        kind = str(command.get("type", "")).upper()
        command = dict(command)
        command["type"] = kind
        if kind not in SUPPORTED_COMMANDS:
            self.send_result(
                command.get("commandId"),
                "ValueError: 모르는 명령입니다: {}".format(kind),
            )
            return

        motion_kinds = {"FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT", "RESUME"}
        snapshot = runtime_snapshot()
        if kind in motion_kinds and snapshot["riskLevel"] == "danger":
            result = "RuntimeError: 센서 위험 상태에서는 이동할 수 없습니다."
            log(result)
            self.send_result(command.get("commandId"), result)
            queue_event("COMMAND_REJECTED", commandType=kind, reason="sensor_danger")
            return

        # 일반 정지는 현재 동작만 멈춘다. 비상 정지는 RESUME 전까지 이동을 막는다.
        if kind in {"STOP", "EMERGENCY_STOP"}:
            self.worker.clear()
            if kind == "EMERGENCY_STOP":
                movement_locked.set()
            try:
                self.robot.stop()
                if kind == "EMERGENCY_STOP":
                    update_runtime(robotState=HAZARD)
                result = "OK"
                log("명령 {} (즉시 실행)".format(kind))
                queue_event(kind)
            except Exception as error:
                result = describe(error)
            self.send_result(command.get("commandId"), result)
            return

        if kind == "RESUME":
            movement_locked.clear()
            command["type"] = "FORWARD"
            log("안전 잠금 해제 후 전진")
            queue_event("SAFETY_LOCK_RELEASED")

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
    log("인증 토큰 설정됨: {}".format(bool(DEVICE_TOKEN)))
    validate_settings()
    dog = RoboDog()
    background_threads = []
    opened = dog.Open() if ROBODOG_PORT is None else dog.Open(ROBODOG_PORT)
    if not opened:
        log("로보독 연결 실패: 동글과 전원을 확인하세요.")
        return

    log("로보독에 연결했습니다.")
    robot = Robot(dog)
    try:
        dog.leg_bend(ALL_LEG, LEG_BEND)
        server_link = ServerLink(robot)
        update_runtime(robotState=IDLE, speed=0)
        background_threads = [
            EventSender(),
            TelemetrySender(robot),
            SafetyMonitor(robot, server_link),
        ]
        for thread in background_threads:
            thread.start()
        queue_event("ROBOT_READY")
        server_link.run()
    except KeyboardInterrupt:
        log("종료합니다.")
    finally:
        stopping.set()
        try:
            robot.stop()
        finally:
            for thread in background_threads:
                thread.join(timeout=HTTP_TIMEOUT_SECONDS + 1.0)
            dog.Close()
            log("로보독 연결을 닫았습니다.")


if __name__ == "__main__":
    main()
