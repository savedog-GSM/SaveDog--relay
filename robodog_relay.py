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
DEVICE_TOKEN = os.getenv("DEVICE_API_TOKEN", "").strip()
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
CAMERA_WS_URL = os.getenv(
    "ROBODOG_CAMERA_WS_URL", "wss://{}/ws/camera/upload".format(SERVER)
).strip()
TELEMETRY_URL = os.getenv(
    "ROBODOG_TELEMETRY_URL", "https://{}/api/robot/telemetry".format(SERVER)
).strip()
COMMAND_WS_URL = os.getenv(
    "ROBODOG_WS_URL", "wss://{}/ws/robot".format(SERVER)
).strip()
ROBOT_ID = os.getenv("ROBODOG_ID", "robodog-01").strip()
CAMERA_INDEX = int(os.getenv("ROBODOG_CAMERA_INDEX", "0"))
CAMERA_JPEG_QUALITY = int(os.getenv("ROBODOG_CAMERA_JPEG_QUALITY", "70"))
CAMERA_FPS = float(os.getenv("ROBODOG_CAMERA_FPS", "10"))
CAMERA_WIDTH = int(os.getenv("ROBODOG_CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("ROBODOG_CAMERA_HEIGHT", "480"))
DISTANCE_WARNING_CM = float(os.getenv("ROBODOG_DISTANCE_WARNING_CM", "50"))
DISTANCE_DANGER_CM = float(os.getenv("ROBODOG_DISTANCE_DANGER_CM", "20"))
TILT_WARNING_DEG = float(os.getenv("ROBODOG_TILT_WARNING_DEG", "15"))
TILT_DANGER_DEG = float(os.getenv("ROBODOG_TILT_DANGER_DEG", "30"))
DANGER_CONFIRM_COUNT = int(os.getenv("ROBODOG_DANGER_CONFIRM_COUNT", "3"))
BARK_THEME = int(os.getenv("ROBODOG_BARK_THEME", "0"))
BARK_TRACK = int(os.getenv("ROBODOG_BARK_TRACK", "1"))
BARK_VOLUME = int(os.getenv("ROBODOG_BARK_VOLUME", "100"))
BARK_COOLDOWN_SECONDS = float(os.getenv("ROBODOG_BARK_COOLDOWN", "5.0"))
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
camera_frame_lock = threading.Lock()
runtime_lock = threading.RLock()
event_queue = queue.Queue()
latest_camera_frame = None
latest_camera_frame_at = 0.0

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


def store_camera_frame(jpeg):
    global latest_camera_frame, latest_camera_frame_at
    with camera_frame_lock:
        latest_camera_frame = bytes(jpeg)
        latest_camera_frame_at = time.monotonic()


def cached_camera_frame(max_age_seconds=5.0):
    with camera_frame_lock:
        if latest_camera_frame is None:
            return None
        if time.monotonic() - latest_camera_frame_at > max_age_seconds:
            return None
        return latest_camera_frame


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
    if not 0 < CAMERA_FPS <= 30:
        raise ValueError("ROBODOG_CAMERA_FPS는 0보다 크고 30 이하여야 합니다.")
    if CAMERA_WIDTH <= 0 or CAMERA_HEIGHT <= 0:
        raise ValueError("카메라 해상도는 0보다 커야 합니다.")
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


def _snapshot_jpeg_bytes():
    cached = cached_camera_frame()
    if cached is not None:
        return cached
    if cv2 is None:
        return None

    camera = cv2.VideoCapture(CAMERA_INDEX)
    try:
        if not camera.isOpened():
            return None
        ok, frame = camera.read()
        if not ok:
            return None
        ok, jpeg = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), CAMERA_JPEG_QUALITY],
        )
        return jpeg.tobytes() if ok else None
    finally:
        camera.release()


def create_hazard_report(robot, source="SENSOR"):
    """최신 카메라 프레임과 센서 값을 서버에 보내 검토 대기 건을 만든다."""
    if not HAZARD_URL:
        return

    with hazard_report_lock:
        image_bytes = _snapshot_jpeg_bytes()
        if image_bytes is None:
            log("위험 신고 생략: 카메라 프레임을 가져오지 못했습니다.")
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

        body, boundary = _multipart_body(fields, image_bytes)
        headers = {
            "Content-Type": "multipart/form-data; boundary={}".format(boundary)
        }
        if DEVICE_TOKEN:
            headers["X-Device-Token"] = DEVICE_TOKEN
        request = urllib.request.Request(
            HAZARD_URL, data=body, headers=headers, method="POST"
        )
        try:
            with urllib.request.urlopen(
                request, timeout=HTTP_TIMEOUT_SECONDS
            ) as response:
                response_body = response.read().decode("utf-8", errors="replace")
            log("센서 위험 신고 후보 생성: {}".format(response_body))
        except (urllib.error.HTTPError, OSError, KeyError, ValueError) as error:
            log("위험 신고 전송 실패: {}".format(error))
        except Exception as error:
            log("위험 신고 처리 실패: {}".format(describe(error)))


def upload_snapshot(event_type):
    """별도 업로드 주소가 설정된 경우 최신 카메라 프레임 한 장을 전송한다."""
    if not CAMERA_UPLOAD_URL:
        return
    with hazard_report_lock:
        image_bytes = _snapshot_jpeg_bytes()
        if image_bytes is None:
            log("이벤트 사진 업로드 생략: 카메라 프레임을 가져오지 못했습니다.")
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
            data=image_bytes,
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=HTTP_TIMEOUT_SECONDS
            ) as response:
                response.read()
            log("이벤트 사진 전송 완료: {}".format(event_type))
        except (urllib.error.HTTPError, OSError, ValueError) as error:
            log("이벤트 사진 전송 실패: {}".format(error))

class CameraStreamer(threading.Thread):
    """OpenCV 카메라의 JPEG 프레임을 WebSocket으로 실시간 전송한다."""

    def __init__(self):
        super().__init__(daemon=True, name="camera-streamer")
        self.problem = None

    def run(self):
        if not CAMERA_WS_URL:
            log("카메라 스트리밍 비활성화: ROBODOG_CAMERA_WS_URL이 비어 있습니다.")
            return
        if cv2 is None:
            log("카메라 스트리밍 비활성화: OpenCV(cv2)가 설치되어 있지 않습니다.")
            return

        while not stopping.is_set():
            camera = self._open_camera()
            if camera is None:
                self._report("카메라 {}을 열 수 없습니다.".format(CAMERA_INDEX))
                stopping.wait(RECONNECT_SECONDS)
                continue
            try:
                self._stream(camera)
            except Exception as error:
                self._report("카메라 스트리밍 실패 ({})".format(describe(error)))
            finally:
                camera.release()
            stopping.wait(RECONNECT_SECONDS)

    def _open_camera(self):
        if os.name == "nt" and hasattr(cv2, "CAP_DSHOW"):
            camera = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
            if not camera.isOpened():
                camera.release()
                camera = cv2.VideoCapture(CAMERA_INDEX)
        else:
            camera = cv2.VideoCapture(CAMERA_INDEX)
        if not camera.isOpened():
            camera.release()
            return None
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        log(
            "카메라 연결: index={} {}x{} {}fps".format(
                CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS
            )
        )
        return camera

    def _stream(self, camera):
        headers = {"X-Device-Token": DEVICE_TOKEN} if DEVICE_TOKEN else None
        interval = 1.0 / CAMERA_FPS
        with connect(
            CAMERA_WS_URL,
            additional_headers=headers,
            open_timeout=5,
            close_timeout=2,
            ping_interval=20,
            ping_timeout=20,
            max_size=None,
        ) as connection:
            self._report(None)
            next_frame_at = time.monotonic()
            while not stopping.is_set():
                ok, frame = camera.read()
                if not ok:
                    raise OSError("카메라 프레임을 읽지 못했습니다.")
                height, width = frame.shape[:2]
                if width != CAMERA_WIDTH or height != CAMERA_HEIGHT:
                    frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                ok, jpeg = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), CAMERA_JPEG_QUALITY],
                )
                if not ok:
                    raise OSError("카메라 프레임 JPEG 변환에 실패했습니다.")
                payload = jpeg.tobytes()
                store_camera_frame(payload)
                connection.send(payload)

                next_frame_at += interval
                delay = next_frame_at - time.monotonic()
                if delay > 0:
                    stopping.wait(delay)
                else:
                    next_frame_at = time.monotonic()

    def _report(self, problem):
        if problem == self.problem:
            return
        if problem is None:
            log("카메라 스트리밍 연결: {}".format(CAMERA_WS_URL))
        else:
            log(problem)
        self.problem = problem

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
            CameraStreamer(),
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
