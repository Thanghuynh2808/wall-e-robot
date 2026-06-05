#!/usr/bin/env python3
"""
robot_bridge.py — Tour System Real Robot Bridge (Phase 2)
=========================================================
Chạy trên Orange Pi (Ubuntu 22.04 ARM + ROS2 Humble).

Đây là single-script để điều hành toàn bộ robot:
  1. WebSocket → Backend: nhận lệnh navigate/stop, gửi heartbeat/feedback
  2. ROS2 Nav2: thực thi NavigateToPose
  3. LiveKit Speaker: nhận audio TTS từ Speech Controller → phát loa
  4. LiveKit Microphone + Echo Gate: thu âm khách, gửi lên LiveKit cho Agent
  5. Auto-reconnect cả WS lẫn LiveKit khi mất mạng

Cài đặt trên Orange Pi:
    pip install websockets livekit livekit-api sounddevice numpy python-dotenv

Chạy:
    python3 robot_bridge.py --backend ws://SERVER:PORT --robot-id walle_001
    # hoặc qua subpath:
    python3 robot_bridge.py --backend wss://SERVER --ws-prefix /walle --robot-id walle_001
"""

import argparse
import asyncio
import base64
import contextlib
import json
import logging
import math
import os
import random
import signal
import string
import threading
from datetime import timedelta
from pathlib import Path

import numpy as np
import sounddevice as sd
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose

import websockets
from livekit import rtc
from livekit.api import AccessToken, VideoGrants

# ── Load .env ──────────────────────────────────────────────────────────────────
load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("robot-bridge")

# ── Config từ env (overridable bằng CLI args) ──────────────────────────────────
LIVEKIT_URL        = os.getenv("LIVEKIT_URL", "")
LIVEKIT_API_KEY    = os.getenv("LIVEKIT_API_KEY", "")
LIVEKIT_API_SECRET = os.getenv("LIVEKIT_API_SECRET", "")
LIVEKIT_ROOM_NAME  = os.getenv("LIVEKIT_ROOM_NAME", os.getenv("ROOM_NAME", "robot-room"))
ROBOT_WS_TOKEN     = os.getenv("ROBOT_WS_TOKEN", "")

# ── Audio Config ───────────────────────────────────────────────────────────────
SAMPLE_RATE    = 48000
CHANNELS       = 1
# Blocksize cho loa: ~85ms/callback — dung sai jitter mạng tốt
SPEAKER_BLOCKSIZE  = 8192
# Pre-buffer: tích lũy trước khi phát để tránh underrun khi kết nối LiveKit
# Speech Controller thêm 600ms silence dẫn đầu nên 1000ms là đủ
PREBUF_SAMPLES = int(SAMPLE_RATE * 1.0)  # 1000ms = 48000 samples

# Chunk mic: 10ms/frame — chuẩn của LiveKit Agent
MIC_CHUNK_SAMPLES  = int(SAMPLE_RATE * 10 / 1000)  # 480 samples

# Echo Gate: khoảng thời gian (giây) mute mic SAU KHI agent ngừng nói
# Tăng lên nếu phòng vang/echo nhiều
POST_SPEECH_DELAY_S = 1.2

# Ngưỡng RMS để phát hiện agent đang nói (bên dưới = silence, không tính)
AGENT_SPEECH_RMS_THRESHOLD = 120

# Tắt echo gate để debug mic: set DISABLE_ECHO_GATE=1 trong env
DISABLE_ECHO_GATE = os.getenv("DISABLE_ECHO_GATE", "0") == "1"

# ── Shared state giữa ROS thread và asyncio event loop ────────────────────────
_pose         = {"x": 0.0, "y": 0.0, "theta": 0.0}
_nav_status   = "idle"
_voice_status = "idle"
_stop_event   = threading.Event()


# ═══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def _quat_to_yaw(ox, oy, oz, ow) -> float:
    """Convert quaternion → yaw (radians)."""
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)


def _make_pose_stamped(x: float, y: float, theta_rad: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(theta_rad / 2.0)
    pose.pose.orientation.w = math.cos(theta_rad / 2.0)
    return pose


def _make_initial_pose(x: float, y: float, theta_rad: float, *, stamp_msg) -> PoseWithCovarianceStamped:
    pose = PoseWithCovarianceStamped()
    pose.header.stamp = stamp_msg
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = float(x)
    pose.pose.pose.position.y = float(y)
    pose.pose.pose.position.z = 0.0
    pose.pose.pose.orientation.z = math.sin(theta_rad / 2.0)
    pose.pose.pose.orientation.w = math.cos(theta_rad / 2.0)
    pose.pose.covariance[0]  = 0.25
    pose.pose.covariance[7]  = 0.25
    pose.pose.covariance[35] = 0.068
    return pose


# ═══════════════════════════════════════════════════════════════════════════════
# ROS2 NODE
# ═══════════════════════════════════════════════════════════════════════════════

class RobotBridgeNode(Node):
    """
    ROS2 node chạy trong background thread.
    - Subscribe /amcl_pose → lấy vị trí robot thực tế.
    - ActionClient NavigateToPose → gửi goal Nav2.
    - Publisher /initialpose → set initial pose cho AMCL.
    - Publisher /cmd_vel → phát zero velocity khi emergency stop.
    """

    def __init__(self):
        super().__init__("robot_bridge_node")
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, "/diff_drive_controller/cmd_vel_unstamped", 10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_callback,
            10,
        )

        self._current_goal_handle = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws_send_queue: asyncio.Queue | None = None
        self._last_amcl_time: float = 0.0

        # TF Listener — fallback khi AMCL im lặng
        self._tf_buffer   = None
        self._tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer   = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.create_timer(0.2, self._tf_pose_callback)
            self.get_logger().info("[Bridge] TF Listener started")
        except Exception as e:
            self.get_logger().warning(f"[Bridge] tf2_ros unavailable: {e}")

    def set_async_context(self, loop: asyncio.AbstractEventLoop, ws_queue: asyncio.Queue):
        self._loop = loop
        self._ws_send_queue = ws_queue

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        import time
        global _pose
        p = msg.pose.pose
        _pose["x"]     = round(p.position.x, 3)
        _pose["y"]     = round(p.position.y, 3)
        _pose["theta"] = round(_quat_to_yaw(
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w,
        ), 4)
        self._last_amcl_time = time.monotonic()

    def _tf_pose_callback(self):
        """Dùng TF chỉ khi AMCL không publish trong 3 giây."""
        import time
        if not self._tf_buffer:
            return
        if time.monotonic() - self._last_amcl_time < 3.0:
            return
        for frame in ("base_link", "base_footprint"):
            try:
                trans = self._tf_buffer.lookup_transform("map", frame, rclpy.time.Time())
                global _pose
                _pose["x"]     = round(trans.transform.translation.x, 3)
                _pose["y"]     = round(trans.transform.translation.y, 3)
                _pose["theta"] = round(_quat_to_yaw(
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ), 4)
                return
            except Exception:
                continue

    async def navigate_to(
        self,
        command_id: str,
        poi_id: int,
        goal_x: float,
        goal_y: float,
        goal_theta: float,
        ws_queue: asyncio.Queue,
    ) -> bool:
        """
        Gửi NavigateToPose goal đến Nav2 và chờ kết quả.
        Gửi navigation_feedback định kỳ về backend.
        Trả về True nếu thành công.
        """
        global _nav_status

        logger.info("[Nav2] Waiting for action server...")
        while not self._nav_client.wait_for_server(timeout_sec=2.0):
            logger.warning("[Nav2] Action server not ready, retrying...")
            await asyncio.sleep(1.0)
            if _stop_event.is_set():
                return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = _make_pose_stamped(goal_x, goal_y, goal_theta)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        logger.info(
            f"[Nav2] Sending goal → POI {poi_id}: "
            f"({goal_x:.3f}, {goal_y:.3f}, θ={goal_theta:.3f})"
        )
        _nav_status = "navigating"

        send_future = self._nav_client.send_goal_async(goal_msg)
        while not send_future.done():
            await asyncio.sleep(0.05)

        goal_handle = send_future.result()
        self._current_goal_handle = goal_handle

        if not goal_handle.accepted:
            logger.error(f"[Nav2] Goal rejected for POI {poi_id}")
            _nav_status = "failed"
            return False

        logger.info("[Nav2] Goal accepted. Navigating...")
        result_future = goal_handle.get_result_async()

        timeout = 180.0
        elapsed = 0.0
        while not result_future.done():
            if elapsed > timeout:
                logger.error("[Nav2] Navigation timeout!")
                await goal_handle.cancel_goal_async()
                _nav_status = "failed"
                return False
            if _stop_event.is_set():
                logger.info("[Nav2] Stop requested — cancelling goal")
                await goal_handle.cancel_goal_async()
                _nav_status = "idle"
                return False

            await ws_queue.put({
                "type": "navigation_feedback",
                "command_id": command_id,
                "poi_id": poi_id,
                "x": _pose["x"],
                "y": _pose["y"],
                "theta": _pose["theta"],
            })
            await asyncio.sleep(1.0)
            elapsed += 1.0

        result = result_future.result()
        if result is None:
            _nav_status = "failed"
            return False

        success = result.status == 4  # GoalStatus.SUCCEEDED
        _nav_status = "arrived" if success else "failed"
        logger.info(
            f"[Nav2] Navigation {'SUCCESS' if success else 'FAILED'} "
            f"for POI {poi_id} (status={result.status})"
        )
        return success

    def cancel_current_goal(self):
        if self._current_goal_handle is not None:
            asyncio.create_task(self._current_goal_handle.cancel_goal_async())
            self._current_goal_handle = None

    def publish_initial_pose(self, x: float, y: float, theta_rad: float):
        stamp_msg = self.get_clock().now().to_msg()
        msg = _make_initial_pose(x, y, theta_rad, stamp_msg=stamp_msg)
        # Publish 2 lần để chắc AMCL nhận được
        self._initial_pose_pub.publish(msg)
        self._initial_pose_pub.publish(msg)

    def publish_zero_velocity(self):
        twist = Twist()
        self._cmd_vel_pub.publish(twist)


# ═══════════════════════════════════════════════════════════════════════════════
# AUDIO CLASSES
# ═══════════════════════════════════════════════════════════════════════════════

class EchoGate:
    """
    Mute microphone khi agent đang phát âm thanh để tránh feedback loop.

    Dùng asyncio timer thay vì đếm frame — đáng tin cậy hơn vì không bị
    ảnh hưởng bởi jitter sounddevice callback.

    State:
      OPEN  (_is_muted=False): mic hoạt động bình thường.
      MUTED (_is_muted=True) : mic gửi silence, timer chờ POST_SPEECH_DELAY_S
                               rồi tự mở lại.
    """

    def __init__(self, loop: asyncio.AbstractEventLoop):
        self._loop         = loop
        self._is_muted     = False
        self._reopen_task: asyncio.TimerHandle | None = None

    @property
    def is_muted(self) -> bool:
        if DISABLE_ECHO_GATE:
            return False
        return self._is_muted

    def agent_speaking(self):
        """
        Gọi từ sounddevice callback khi nhận frame audio có RMS > threshold.
        Thread-safe: dùng call_soon_threadsafe.
        """
        if not self._is_muted:
            self._is_muted = True
            self._loop.call_soon_threadsafe(self._log_closed)
        # Reset timer mỗi lần nhận thêm audio từ agent
        if self._reopen_task is not None:
            self._reopen_task.cancel()
        self._reopen_task = self._loop.call_later(POST_SPEECH_DELAY_S, self._open)

    def _open(self):
        if self._is_muted:
            self._is_muted = False
            logger.info("🎙️  Echo gate OPEN — mic hoạt động trở lại")

    def _log_closed(self):
        logger.info(
            f"🔇 Echo gate CLOSED — agent đang nói, "
            f"mic mute {POST_SPEECH_DELAY_S}s sau khi im lặng"
        )


# ═══════════════════════════════════════════════════════════════════════════════
# LIVEKIT — Full-duplex (Speaker + Microphone + Echo Gate)
# ═══════════════════════════════════════════════════════════════════════════════

async def _receive_speaker_track(
    track: rtc.AudioTrack,
    player: SpeakerPlayer,
    gate: EchoGate,
    my_gen: int,
    gen_ref: list[int],
):
    """
    Nhận audio TTS từ Speech Controller / Agent và:
      1. Đẩy vào SpeakerPlayer để phát loa.
      2. Kích hoạt EchoGate khi phát hiện âm thanh thực (RMS > threshold).
    """
    audio_stream = rtc.AudioStream(track, sample_rate=SAMPLE_RATE, num_channels=CHANNELS)
    logger.info("[Speaker] Audio stream started — buffering...")
    async for event in audio_stream:
        if isinstance(event, rtc.AudioFrameEvent):
            if my_gen != gen_ref[0]:
                # Track mới đến, task này đã lỗi thời
                break
            data = np.frombuffer(event.frame.data, dtype=np.int16)
            rms = int(np.sqrt(np.mean(data.astype(np.float32) ** 2)))
            if rms > AGENT_SPEECH_RMS_THRESHOLD:
                gate.agent_speaking()
            player.push(data)


async def _mic_capture_loop(
    audio_source: rtc.AudioSource,
    gate: EchoGate,
):
    """
    Vòng lặp thu âm microphone và gửi frame lên LiveKit.
    - Khi EchoGate đóng → gửi silence để Agent không nghe tiếng loa của chính nó.
    - Log RMS mic mỗi ~1 giây để debug.
    """
    mic_queue: asyncio.Queue[np.ndarray] = asyncio.Queue(maxsize=200)
    loop = asyncio.get_running_loop()
    frame_sent    = 0
    level_counter = 0

    def _mic_sd_callback(indata, frames, time_info, status):
        if gate.is_muted:
            silence = np.zeros(frames, dtype=np.int16)
            loop.call_soon_threadsafe(mic_queue.put_nowait, silence)
        else:
            loop.call_soon_threadsafe(
                mic_queue.put_nowait,
                indata.flatten().copy().astype(np.int16),
            )

    mic_stream = None
    try:
        mic_stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype="int16",
            blocksize=MIC_CHUNK_SAMPLES,
            callback=_mic_sd_callback,
        )
        mic_stream.start()
        logger.info("🎙️  Microphone started — khách có thể nói chuyện với robot")
    except Exception as e:
        logger.error(f"[Mic] Không thể khởi động microphone: {e}")
        return

    try:
        while not _stop_event.is_set():
            data = await mic_queue.get()
            flat = data.flatten().astype(np.int16)

            level_counter += 1
            if level_counter >= 100:  # log mỗi ~1 giây (100 * 10ms)
                level_counter = 0
                rms = int(np.sqrt(np.mean(flat.astype(np.float32) ** 2)))
                logger.info(
                    f"[Mic] RMS={rms:5d} | gate={'MUTED' if gate.is_muted else 'OPEN ':5s} "
                    f"| frames_sent={frame_sent}"
                )

            frame = rtc.AudioFrame(
                data=flat.tobytes(),
                sample_rate=SAMPLE_RATE,
                num_channels=CHANNELS,
                samples_per_channel=len(flat),
            )
            await audio_source.capture_frame(frame)
            frame_sent += 1

    except asyncio.CancelledError:
        pass
    finally:
        if mic_stream is not None:
            with contextlib.suppress(Exception):
                mic_stream.stop()
                mic_stream.close()
        logger.info("[Mic] Microphone stopped")


async def _livekit_connect(robot_id: str, player: SpeakerPlayer, gate: EchoGate):
    """
    Kết nối LiveKit room với full-duplex:
      - Subscribe audio track từ Agent/Speech Controller → loa (Speaker).
      - Publish mic track → Agent nghe được khách.

    Tự reconnect khi mất kết nối.
    """
    if not LIVEKIT_URL or not LIVEKIT_API_KEY or not LIVEKIT_API_SECRET:
        logger.warning("[LiveKit] Chưa cấu hình LIVEKIT_URL/KEY/SECRET — bỏ qua audio")
        return

    while not _stop_event.is_set():
        suffix   = "".join(random.choices(string.ascii_lowercase + string.digits, k=4))
        identity = f"{robot_id}-{suffix}"

        token = (
            AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
            .with_identity(identity)
            .with_name(identity)
            .with_grants(VideoGrants(room_join=True, room=LIVEKIT_ROOM_NAME))
            .with_ttl(timedelta(seconds=86400))
            .to_jwt()
        )

        room = rtc.Room()

        # Generation counter để quản lý khi có nhiều track đến liên tiếp
        _gen: list[int]                        = [0]
        _speaker_task: list[asyncio.Task | None] = [None]
        _mic_task: asyncio.Task | None           = None

        @room.on("track_subscribed")
        def on_track(track, pub, participant):
            if track.kind != rtc.TrackKind.KIND_AUDIO:
                return
            logger.info(f"[LiveKit] Audio track từ {participant.identity}")

            # Tạo AudioStream NGAY LẬP TỨC (sync) để không bỏ lỡ frame đầu
            audio_stream = rtc.AudioStream(track, sample_rate=SAMPLE_RATE, num_channels=CHANNELS)

            _gen[0] += 1
            my_gen = _gen[0]

            prev = _speaker_task[0]
            if prev and not prev.done():
                logger.info("[LiveKit] Huỷ speaker task cũ")
                prev.cancel()

            player.clear()

            async def _process():
                if my_gen != _gen[0]:
                    return
                logger.info("[LiveKit] Bắt đầu nhận audio, đang buffer...")
                async for event in audio_stream:
                    if isinstance(event, rtc.AudioFrameEvent):
                        if my_gen != _gen[0]:
                            break
                        data = np.frombuffer(event.frame.data, dtype=np.int16)
                        rms  = int(np.sqrt(np.mean(data.astype(np.float32) ** 2)))
                        if rms > AGENT_SPEECH_RMS_THRESHOLD:
                            gate.agent_speaking()
                        player.push(data)

            _speaker_task[0] = asyncio.ensure_future(_process())

        @room.on("track_unsubscribed")
        def on_track_unsub(track, pub, participant):
            if track.kind != rtc.TrackKind.KIND_AUDIO:
                return
            logger.info(f"[LiveKit] Track unsubscribed từ {participant.identity}")
            prev = _speaker_task[0]
            if prev and not prev.done():
                prev.cancel()
            _speaker_task[0] = None
            # Không clear buffer — để tiếng còn lại drain hết ra loa

        @room.on("disconnected")
        def on_disc(reason):
            logger.warning(f"[LiveKit] Disconnected: {reason}")

        try:
            await room.connect(LIVEKIT_URL, token)
            logger.info(
                f"[LiveKit] ✅ Connected | room={LIVEKIT_ROOM_NAME} | identity={identity}"
            )

            # Publish microphone track vào room để Agent nghe được
            audio_source = rtc.AudioSource(SAMPLE_RATE, CHANNELS)
            mic_track    = rtc.LocalAudioTrack.create_audio_track("robot-mic", audio_source)
            await room.local_participant.publish_track(
                mic_track,
                rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE),
            )
            logger.info("[LiveKit] 🎙️  Mic track published — Agent có thể nghe khách")

            # Chạy mic capture loop
            _mic_task = asyncio.create_task(_mic_capture_loop(audio_source, gate))

            # Giữ kết nối cho đến khi stop
            while not _stop_event.is_set():
                await asyncio.sleep(5.0)

        except asyncio.CancelledError:
            break
        except Exception as e:
            logger.error(f"[LiveKit] Connection error: {e}")
        finally:
            if _mic_task and not _mic_task.done():
                _mic_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await _mic_task
            prev = _speaker_task[0]
            if prev and not prev.done():
                prev.cancel()
            with contextlib.suppress(Exception):
                await room.disconnect()

        if not _stop_event.is_set():
            logger.info("[LiveKit] Reconnecting in 5s...")
            await asyncio.sleep(5.0)


# ═══════════════════════════════════════════════════════════════════════════════
# WEBSOCKET BRIDGE
# ═══════════════════════════════════════════════════════════════════════════════

async def _run_bridge(backend_ws_url: str, robot_id: str, node: RobotBridgeNode):
    """
    Main WebSocket bridge loop:
    - Kết nối WebSocket đến backend, auto-reconnect.
    - Gửi heartbeat mỗi 2 giây.
    - Nhận và dispatch lệnh từ backend.
    """
    global _nav_status, _voice_status

    send_queue: asyncio.Queue = asyncio.Queue()
    node.set_async_context(asyncio.get_event_loop(), send_queue)

    _current_nav_task: asyncio.Task | None = None
    active_runtime_mode = "idle"
    localization_status = "unknown"

    # ── Heartbeat ──────────────────────────────────────────────────────────────
    async def _heartbeat():
        while True:
            await send_queue.put({
                "type": "robot_state",
                "x": _pose["x"],
                "y": _pose["y"],
                "theta": _pose["theta"],
                "nav_status": _nav_status,
                "voice_status": _voice_status,
                "runtime_mode": (
                    "touring" if _nav_status == "navigating" else active_runtime_mode
                ),
                "localization_status": localization_status,
            })
            await asyncio.sleep(2.0)

    # ── Command handlers ───────────────────────────────────────────────────────

    async def _handle_navigate(msg: dict):
        nonlocal _current_nav_task, active_runtime_mode
        global _nav_status

        command_id  = msg.get("command_id", "")
        poi_id      = msg["poi_id"]
        goal_x      = float(msg["x"])
        goal_y      = float(msg["y"])
        goal_theta  = float(msg.get("theta", 0.0))

        # Huỷ navigation đang chạy (nếu có)
        if _current_nav_task and not _current_nav_task.done():
            _stop_event.set()
            _current_nav_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await _current_nav_task
        _stop_event.clear()

        async def _do_nav():
            nonlocal active_runtime_mode
            global _nav_status
            active_runtime_mode = "touring"
            success = await node.navigate_to(
                command_id, poi_id, goal_x, goal_y, goal_theta, send_queue
            )
            await send_queue.put({
                "type": "navigation_result",
                "command_id": command_id,
                "poi_id": poi_id,
                "success": success,
                "message": "Goal reached" if success else "Navigation failed",
            })
            if _nav_status not in ("idle",):
                await asyncio.sleep(1.5)
                _nav_status = "idle"
            active_runtime_mode = "idle"

        _current_nav_task = asyncio.create_task(_do_nav())

    async def _handle_stop(msg: dict):
        nonlocal _current_nav_task, active_runtime_mode
        global _nav_status
        logger.info("[Bridge] STOP command")
        _stop_event.set()
        if _current_nav_task and not _current_nav_task.done():
            _current_nav_task.cancel()
        node.cancel_current_goal()
        _nav_status         = "idle"
        active_runtime_mode = "idle"
        await send_queue.put({
            "type": "command_result",
            "command_id": msg.get("command_id", ""),
            "command_type": "stop",
            "success": True,
            "message": "Navigation stopped",
        })
        _stop_event.clear()

    async def _handle_set_initial_pose(msg: dict):
        nonlocal active_runtime_mode, localization_status
        x      = float(msg.get("x", 0.0))
        y      = float(msg.get("y", 0.0))
        theta  = float(msg.get("theta", 0.0))
        map_id = msg.get("map_id")

        active_runtime_mode = "localizing"
        localization_status = "setting"
        node.publish_initial_pose(x, y, theta)

        await send_queue.put({
            "type": "localization_status",
            "command_id": msg.get("command_id", ""),
            "status": "setting",
            "map_id": map_id,
            "x": x, "y": y, "theta": theta,
        })
        await asyncio.sleep(1.5)   # Chờ AMCL xử lý

        localization_status = "ready"
        active_runtime_mode = "idle"
        await send_queue.put({
            "type": "localization_status",
            "command_id": msg.get("command_id", ""),
            "status": "ready",
            "map_id": map_id,
            "x": _pose["x"] if (_pose["x"] or _pose["y"]) else x,
            "y": _pose["y"] if (_pose["x"] or _pose["y"]) else y,
            "theta": _pose["theta"] if _pose["theta"] else theta,
        })
        await send_queue.put({
            "type": "command_result",
            "command_id": msg.get("command_id", ""),
            "command_type": "set_initial_pose",
            "success": True,
            "message": "Initial pose applied",
        })

    async def _handle_emergency_stop(msg: dict):
        nonlocal _current_nav_task, active_runtime_mode
        global _nav_status
        logger.warning("[Bridge] ⚠️  EMERGENCY STOP")
        _stop_event.set()
        if _current_nav_task and not _current_nav_task.done():
            _current_nav_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await _current_nav_task
        node.cancel_current_goal()
        node.publish_zero_velocity()
        _nav_status         = "idle"
        active_runtime_mode = "emergency_stop"
        await send_queue.put({
            "type": "command_result",
            "command_id": msg.get("command_id", ""),
            "command_type": "emergency_stop",
            "success": True,
            "message": "Emergency stop executed",
        })
        _stop_event.clear()

    # ── WebSocket connect + receive loop ───────────────────────────────────────
    while True:
        try:
            logger.info(f"[Bridge] Connecting to {backend_ws_url} ...")
            async with websockets.connect(
                backend_ws_url,
                ping_interval=20,
                ping_timeout=10,
                max_size=10 * 1024 * 1024,
            ) as ws:
                logger.info(f"[Bridge] ✅ Connected as robot_id={robot_id}")
                _nav_status = "idle"

                hb_task = asyncio.create_task(_heartbeat())

                async def _sender():
                    while True:
                        msg = await send_queue.get()
                        msg_type = msg.get("type", "unknown")
                        try:
                            await ws.send(json.dumps(msg))
                            logger.debug(f"[Bridge] → SENT: {msg_type}")
                        except Exception as e:
                            logger.error(f"[Bridge] → SEND FAILED ({msg_type}): {e}")

                send_task = asyncio.create_task(_sender())

                try:
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                        except Exception:
                            continue

                        msg_type = msg.get("type", "")
                        logger.info(f"[Bridge] ← CMD: {msg_type}")

                        if msg_type == "navigate":
                            await _handle_navigate(msg)

                        elif msg_type == "stop":
                            await _handle_stop(msg)

                        elif msg_type == "set_initial_pose":
                            await _handle_set_initial_pose(msg)

                        elif msg_type == "emergency_stop":
                            await _handle_emergency_stop(msg)

                        elif msg_type == "speak":
                            # Speech Controller sẽ tự push audio vào LiveKit room.
                            # Bridge chỉ cần log — audio player sẽ tự nhận qua track_subscribed.
                            text = msg.get("text", "")
                            logger.info(f"[Bridge] SPEAK queued: {text[:80]}...")

                        elif msg_type == "ack":
                            logger.info(f"[Bridge] ACK: {msg}")

                        else:
                            logger.warning(f"[Bridge] Unknown message type: {msg_type!r}")

                except websockets.exceptions.ConnectionClosed as e:
                    logger.warning(f"[Bridge] Connection closed: {e.code}")
                finally:
                    hb_task.cancel()
                    send_task.cancel()
                    if _current_nav_task and not _current_nav_task.done():
                        _current_nav_task.cancel()

        except (ConnectionRefusedError, OSError) as e:
            logger.error(f"[Bridge] Cannot connect: {e}")
        except websockets.exceptions.WebSocketException as e:
            logger.error(f"[Bridge] WebSocket error: {e}")
        except asyncio.CancelledError:
            logger.info("[Bridge] Shutting down")
            return
        except Exception as e:
            logger.exception(f"[Bridge] Unexpected error: {e}")

        logger.info("[Bridge] Reconnecting in 5s...")
        await asyncio.sleep(5.0)


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

async def main(args):
    # ── Build WebSocket URL ────────────────────────────────────────────────────
    ws_prefix      = args.ws_prefix.rstrip("/")
    backend_ws_url = f"{args.backend.rstrip('/')}{ws_prefix}/ws/robots/{args.robot_id}"
    if args.token:
        sep = "&" if "?" in backend_ws_url else "?"
        backend_ws_url = f"{backend_ws_url}{sep}token={args.token}"

    logger.info("=" * 60)
    logger.info(f"  WebSocket URL : {backend_ws_url}")
    logger.info(f"  LiveKit Room  : {LIVEKIT_ROOM_NAME}")
    logger.info(f"  Robot ID      : {args.robot_id}")
    logger.info(f"  Echo Gate     : {'DISABLED (debug)' if DISABLE_ECHO_GATE else 'ENABLED'}")
    logger.info("=" * 60)

    # ── Khởi tạo ROS2 ─────────────────────────────────────────────────────────
    rclpy.init()
    node = RobotBridgeNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    logger.info("[ROS2] Spin thread started")

    # ── Khởi tạo Loa (sounddevice output) ─────────────────────────────────────
    player  = SpeakerPlayer()
    speaker = None
    try:
        speaker = sd.OutputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype="int16",
            blocksize=SPEAKER_BLOCKSIZE,   # ~85ms/callback — dung sai jitter mạng tốt
            callback=player.callback,
        )
        speaker.start()
        logger.info("🔊 Speaker started")
    except Exception as e:
        logger.error(f"[Speaker] Không thể khởi động loa: {e} — chạy không có âm thanh")

    # ── EchoGate: dùng chung giữa livekit task và mic ─────────────────────────
    loop = asyncio.get_running_loop()
    gate = EchoGate(loop)

    # ── Khởi chạy các task song song ─────────────────────────────────────────
    bridge_task   = asyncio.create_task(
        _run_bridge(backend_ws_url, args.robot_id, node)
    )
    livekit_task  = asyncio.create_task(
        _livekit_connect(args.robot_id, player, gate)
    )

    # ── SIGINT / SIGTERM handler ───────────────────────────────────────────────
    def _on_signal():
        logger.info("Signal received — shutting down...")
        _stop_event.set()
        bridge_task.cancel()
        livekit_task.cancel()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _on_signal)

    try:
        await asyncio.gather(bridge_task, livekit_task, return_exceptions=True)
    finally:
        logger.info("Shutting down ROS2 and audio...")
        if speaker:
            with contextlib.suppress(Exception):
                speaker.stop()
                speaker.close()
        node.destroy_node()
        rclpy.shutdown()
        logger.info("✅ Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Wall-E Real Robot Bridge — Phase 2 (Nav2 + Full-duplex LiveKit Q&A)"
    )
    parser.add_argument(
        "--backend",
        default=os.getenv("BACKEND_WS_URL", "ws://mai.emobiz.vn:51102"),
        help="Backend WebSocket base URL (default: ws://mai.emobiz.vn:51102)",
    )
    parser.add_argument(
        "--ws-prefix",
        default=os.getenv("WS_PREFIX", ""),
        help="Path prefix nếu dùng subpath routing, ví dụ: /walle (mặc định: rỗng)",
    )
    parser.add_argument(
        "--robot-id",
        default=os.getenv("ROBOT_ID", "walle_001"),
        help="Robot ID (phải khớp với backend, mặc định: walle_001)",
    )
    parser.add_argument(
        "--token",
        default=ROBOT_WS_TOKEN,
        help="Robot WebSocket token để xác thực với backend",
    )
    parsed = parser.parse_args()

    try:
        asyncio.run(main(parsed))
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
