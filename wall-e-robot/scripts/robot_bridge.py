#!/usr/bin/env python3
"""
robot_bridge.py — Tour System Real Robot Bridge
================================================
Chạy trên Orange Pi (Ubuntu 22.04 ARM + ROS2 Humble).

Thay thế tour_runner.py cũ. Script này:
  1. Kết nối WebSocket → Backend Tour System (nhận lệnh navigate/stop/speak)
  2. Bridge lệnh navigate → ROS2 Nav2 NavigateToPose
  3. Subscribe /amcl_pose → gửi robot_state heartbeat lên Backend
  4. Kết nối LiveKit → nhận audio TTS từ Speech Controller → phát loa
  5. Tự động reconnect khi mất mạng

Cài đặt trên Orange Pi:
    pip install websockets livekit livekit-api sounddevice numpy python-dotenv

Chạy:
    python3 robot_bridge.py --backend ws://mai.emobiz.vn:51102 --robot-id walle_001
    # hoặc qua subpath:
    python3 robot_bridge.py --backend wss://mai.emobiz.vn --ws-prefix /walle --robot-id walle_001
"""

import argparse
import asyncio
import json
import logging
import math
import os
import queue
import random
import signal
import string
import sys
import threading
from datetime import timedelta

import numpy as np
import sounddevice as sd
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

import websockets
from livekit import rtc
from livekit.api import AccessToken, VideoGrants

# ── Load .env ─────────────────────────────────────────────────────────────────
load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("robot-bridge")

# ── Config từ env (overridable bằng CLI args) ─────────────────────────────────
LIVEKIT_URL        = os.getenv("LIVEKIT_URL", "")
LIVEKIT_API_KEY    = os.getenv("LIVEKIT_API_KEY", "")
LIVEKIT_API_SECRET = os.getenv("LIVEKIT_API_SECRET", "")
LIVEKIT_ROOM_NAME  = os.getenv("LIVEKIT_ROOM_NAME", os.getenv("ROOM_NAME", "robot-room"))

SAMPLE_RATE    = 48000
CHANNELS       = 1
# Blocksize 4096 = ~85ms per callback — tolerates network jitter up to 85ms
# (vs 10ms before which caused constant underruns over internet)
BLOCKSIZE      = 8192
# Pre-buffer: accumulate before playback to absorb LiveKit connect + jitter delay.
# 150ms is enough since wall-clock pacing on speech-ctrl makes arrival predictable.
# Speech-ctrl also adds 600ms lead-in silence, so total startup delay is well-covered.
PREBUF_SAMPLES = int(SAMPLE_RATE * 1.0)  # 1000ms = 48000 samples

# Shared state giữa ROS thread và asyncio event loop
_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
_nav_status   = "idle"
_voice_status = "idle"
_stop_event   = threading.Event()


# ── Helpers ───────────────────────────────────────────────────────────────────

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


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class RobotBridgeNode(Node):
    """
    ROS2 node chạy trong background thread riêng.
    - Subscribe /amcl_pose để lấy vị trí thực tế của robot.
    - ActionClient NavigateToPose để gửi goal nav2.
    """

    def __init__(self):
        super().__init__("robot_bridge_node")
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Subscribe vị trí AMCL realtime
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_callback,
            10,
        )

        self._current_goal_handle = None
        self._loop: asyncio.AbstractEventLoop | None = None  # set sau khi asyncio loop start
        self._ws_send_queue: asyncio.Queue | None = None

    def set_async_context(self, loop: asyncio.AbstractEventLoop, ws_queue: asyncio.Queue):
        self._loop = loop
        self._ws_send_queue = ws_queue

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Cập nhật pose global khi AMCL publish."""
        global _pose
        p = msg.pose.pose
        _pose["x"] = round(p.position.x, 3)
        _pose["y"] = round(p.position.y, 3)
        _pose["theta"] = round(_quat_to_yaw(
            p.orientation.x, p.orientation.y,
            p.orientation.z, p.orientation.w
        ), 4)

    async def navigate_to(
        self,
        poi_id: int,
        goal_x: float,
        goal_y: float,
        goal_theta: float,
        ws_queue: asyncio.Queue,
    ) -> bool:
        """
        Gửi NavigateToPose goal đến Nav2 và chờ kết quả.
        Trong khi chờ, gửi navigation_feedback định kỳ lên Backend.
        Trả về True nếu thành công.
        """
        global _nav_status

        logger.info(f"[Nav2] Waiting for action server...")
        while not self._nav_client.wait_for_server(timeout_sec=2.0):
            logger.warning("[Nav2] Action server not ready, retrying...")
            await asyncio.sleep(1.0)
            if _stop_event.is_set():
                return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = _make_pose_stamped(goal_x, goal_y, goal_theta)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        logger.info(f"[Nav2] Sending goal → POI {poi_id}: ({goal_x:.3f}, {goal_y:.3f}, θ={goal_theta:.3f})")
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

        logger.info(f"[Nav2] Goal accepted. Robot navigating...")
        result_future = goal_handle.get_result_async()

        # Poll kết quả, gửi feedback định kỳ
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

            # Gửi feedback với pose hiện tại
            await ws_queue.put({
                "type": "navigation_feedback",
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

        # Nav2 GoalStatus: SUCCEEDED=4
        success = result.status == 4
        _nav_status = "arrived" if success else "failed"
        logger.info(f"[Nav2] Navigation {'SUCCESS' if success else 'FAILED'} for POI {poi_id} (status={result.status})")
        return success

    def cancel_current_goal(self):
        """Gửi cancel đến Nav2 goal hiện tại (gọi từ asyncio)."""
        if self._current_goal_handle is not None:
            asyncio.create_task(self._current_goal_handle.cancel_goal_async())
            self._current_goal_handle = None


# ── Audio (LiveKit → Loa) ─────────────────────────────────────────────────────

class AudioPlayer:
    """
    Thread-safe audio player with one-time pre-buffering.

    State machine:
      WAITING  (_streaming=False): accumulating PREBUF_SAMPLES before starting.
                                   Output silence during this phase.
      PLAYING  (_streaming=True):  playing audio. Once in this state, NEVER
                                   go back to WAITING during the same stream.
                                   Momentary buffer empty = tiny silence gap,
                                   NOT a full 300ms re-buffer wait.
      RESET:   clear() called from track_subscribed when new TTS starts.
    """
    def __init__(self):
        self._lock      = threading.Lock()
        self._buf       = np.array([], dtype=np.int16)
        self._streaming = False   # Set True once; only cleared by clear()

    def push(self, data: np.ndarray):
        with self._lock:
            self._buf = np.concatenate((self._buf, data))
            if not self._streaming and len(self._buf) >= PREBUF_SAMPLES:
                self._streaming = True
                logger.info(f"[Audio] Pre-buffer ready ({len(self._buf)} samples) — starting playback")

    def clear(self):
        """Reset for a new TTS stream. Called when a new track subscription arrives."""
        with self._lock:
            self._buf       = np.array([], dtype=np.int16)
            self._streaming = False

    def callback(self, outdata, frames, time_info, status):
        with self._lock:
            if not self._streaming:
                # Pre-buffer phase: output silence and wait
                outdata[:] = 0
                return

            if len(self._buf) >= frames:
                # Normal case: enough data in buffer
                outdata[:] = self._buf[:frames].reshape(-1, 1)
                self._buf  = self._buf[frames:]
            else:
                # Buffer momentarily empty (network jitter).
                # Output silence for this one callback period (~85ms).
                # DO NOT reset _streaming — more data will arrive shortly.
                # Resetting here was the bug causing 300ms forced silence gaps.
                n = len(self._buf)
                if n > 0:
                    outdata[:n] = self._buf.reshape(-1, 1)
                    outdata[n:] = 0
                    self._buf   = np.array([], dtype=np.int16)
                else:
                    outdata[:] = 0


async def _receive_livekit_audio(track: rtc.AudioTrack, player: AudioPlayer):
    audio_stream = rtc.AudioStream(track, sample_rate=SAMPLE_RATE, num_channels=CHANNELS)
    first = True
    async for event in audio_stream:
        if isinstance(event, rtc.AudioFrameEvent):
            if first:
                logger.info("[LiveKit] Audio stream started — buffering...")
                first = False
            data = np.frombuffer(event.frame.data, dtype=np.int16)
            player.push(data)


async def _livekit_connect(robot_id: str, player: AudioPlayer):
    """
    Kết nối vào LiveKit room để nhận audio TTS từ Speech Controller.
    Speech Controller publish audio track vào room này.
    """
    if not LIVEKIT_URL or not LIVEKIT_API_KEY:
        logger.warning("[LiveKit] Không có cấu hình — bỏ qua audio stream")
        return

    suffix = "".join(random.choices(string.ascii_lowercase + string.digits, k=4))
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
    # Generation counter: each new track_subscribed increments this.
    # Each receive task checks its own generation to abort if superseded.
    # This handles LiveKit reconnect events that fire track_subscribed multiple times.
    _gen: list[int]               = [0]
    _audio_task: list[asyncio.Task | None] = [None]

    @room.on("track_subscribed")
    def on_track(track, pub, participant):
        if track.kind != rtc.TrackKind.KIND_AUDIO:
            return
        logger.info(f"[LiveKit] Audio track from {participant.identity} — receiving")

        # ── Create AudioStream IMMEDIATELY (sync) to start buffering frames ──
        # Any delay here (e.g. inside a coroutine) causes early frames to be
        # missed, cutting off the first characters of speech.
        audio_stream = rtc.AudioStream(track, sample_rate=SAMPLE_RATE, num_channels=CHANNELS)

        _gen[0] += 1
        my_gen = _gen[0]

        prev = _audio_task[0]
        if prev and not prev.done():
            logger.info("[LiveKit] Cancelling previous audio receive task")
            prev.cancel()

        player.clear()

        async def _process_stream():
            if my_gen != _gen[0]:
                return
            logger.info("[LiveKit] Audio stream started — buffering...")
            async for event in audio_stream:
                if isinstance(event, rtc.AudioFrameEvent):
                    if my_gen != _gen[0]:
                        break
                    player.push(np.frombuffer(event.frame.data, dtype=np.int16))

        _audio_task[0] = asyncio.ensure_future(_process_stream())

    @room.on("track_unsubscribed")
    def on_track_unsub(track, pub, participant):
        if track.kind != rtc.TrackKind.KIND_AUDIO:
            return
        logger.info(f"[LiveKit] Track unsubscribed from {participant.identity}")
        prev = _audio_task[0]
        if prev and not prev.done():
            prev.cancel()
        _audio_task[0] = None
        # Don't clear buffer: let remaining audio drain to speaker naturally

    @room.on("disconnected")
    def on_disc(reason):
        logger.warning(f"[LiveKit] Disconnected: {reason}")

    try:
        await room.connect(LIVEKIT_URL, token)
        logger.info(f"[LiveKit] Connected | room={LIVEKIT_ROOM_NAME} identity={identity}")
        while not _stop_event.is_set():
            await asyncio.sleep(5.0)
    except asyncio.CancelledError:
        pass
    except Exception as e:
        logger.error(f"[LiveKit] Connection error: {e}")
    finally:
        prev = _audio_task[0]
        if prev and not prev.done():
            prev.cancel()
        await room.disconnect()


# ── WebSocket Bridge ──────────────────────────────────────────────────────────

async def _run_bridge(backend_ws_url: str, robot_id: str, node: RobotBridgeNode):
    """
    Main loop: kết nối WebSocket với backend, handle commands, auto-reconnect.
    """
    global _nav_status, _voice_status

    # Queue để gửi messages về backend (từ nav feedback, heartbeat, etc.)
    send_queue: asyncio.Queue = asyncio.Queue()
    node.set_async_context(asyncio.get_event_loop(), send_queue)

    _current_nav_task: asyncio.Task | None = None

    async def _heartbeat():
        while True:
            await send_queue.put({
                "type": "robot_state",
                "x": _pose["x"],
                "y": _pose["y"],
                "theta": _pose["theta"],
                "nav_status": _nav_status,
                "voice_status": _voice_status,
            })
            await asyncio.sleep(2.0)

    async def _handle_navigate(ws, msg: dict):
        nonlocal _current_nav_task
        global _nav_status

        poi_id    = msg["poi_id"]
        goal_x    = float(msg["x"])
        goal_y    = float(msg["y"])
        goal_theta = float(msg.get("theta", 0.0))

        # Huỷ navigation đang chạy (nếu có)
        if _current_nav_task and not _current_nav_task.done():
            _stop_event.set()
            _current_nav_task.cancel()
            try:
                await _current_nav_task
            except asyncio.CancelledError:
                pass
        _stop_event.clear()

        async def _do_nav():
            global _nav_status
            success = await node.navigate_to(poi_id, goal_x, goal_y, goal_theta, send_queue)

            # Gửi kết quả navigation về backend
            await send_queue.put({
                "type": "navigation_result",
                "poi_id": poi_id,
                "success": success,
                "message": "Goal reached" if success else "Navigation failed",
            })

            # Reset về idle
            if _nav_status not in ("idle",):
                await asyncio.sleep(1.5)
                _nav_status = "idle"

        _current_nav_task = asyncio.create_task(_do_nav())

    async def _handle_stop():
        nonlocal _current_nav_task
        global _nav_status
        logger.info("[Bridge] STOP command received")
        _stop_event.set()
        if _current_nav_task and not _current_nav_task.done():
            _current_nav_task.cancel()
        node.cancel_current_goal()
        _nav_status = "idle"
        _stop_event.clear()

    # ── WebSocket connect + loop ──────────────────────────────────────────────
    while True:
        try:
            logger.info(f"[Bridge] Connecting to {backend_ws_url} ...")
            async with websockets.connect(
                backend_ws_url,
                ping_interval=20,
                ping_timeout=10,
                max_size=10 * 1024 * 1024,  # 10MB cho map uploads
            ) as ws:
                logger.info(f"[Bridge] Connected as robot_id={robot_id}")
                _nav_status = "idle"

                hb_task = asyncio.create_task(_heartbeat())

                async def _sender():
                    """Drains send_queue và gửi lên WebSocket."""
                    while True:
                        msg = await send_queue.get()
                        try:
                            await ws.send(json.dumps(msg))
                        except Exception:
                            # Nếu WS đóng, msg sẽ bị mất — reconnect sẽ xử lý
                            pass

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
                            await _handle_navigate(ws, msg)

                        elif msg_type == "stop":
                            await _handle_stop()

                        elif msg_type == "speak":
                            # Speech Controller sẽ phát audio qua LiveKit.
                            # Bridge chỉ cần log và cập nhật voice_status.
                            # Khi backend gọi trigger_speech(), speech controller
                            # tự publish audio vào LiveKit room.
                            text = msg.get("text", "")
                            logger.info(f"[Bridge] SPEAK: {text[:60]}...")
                            # voice_status được backend update qua voice-done callback.

                        elif msg_type == "ack":
                            logger.info(f"[Bridge] ACK: {msg}")

                        else:
                            logger.warning(f"[Bridge] Unknown msg type: {msg_type}")

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


# ── Main ──────────────────────────────────────────────────────────────────────

async def main(args):
    # Build WebSocket URL
    ws_prefix = args.ws_prefix.rstrip("/")
    backend_ws_url = f"{args.backend.rstrip('/')}{ws_prefix}/ws/robots/{args.robot_id}"
    logger.info(f"WebSocket URL: {backend_ws_url}")
    logger.info(f"LiveKit Room:  {LIVEKIT_ROOM_NAME}")
    logger.info(f"Robot ID:      {args.robot_id}")

    # Khởi tạo ROS2
    rclpy.init()
    node = RobotBridgeNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    logger.info("[ROS2] Spin thread started")

    # Khởi tạo loa (sounddevice)
    player = AudioPlayer()
    speaker = None
    try:
        speaker = sd.OutputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype="int16",
            blocksize=BLOCKSIZE,   # 85ms per callback — jitter tolerant
            callback=player.callback,
        )
        speaker.start()
        logger.info("[Audio] Speaker started")
    except Exception as e:
        logger.error(f"[Audio] Speaker init failed: {e} — audio disabled")

    # Các task chạy song song
    bridge_task   = asyncio.create_task(_run_bridge(backend_ws_url, args.robot_id, node))
    livekit_task  = asyncio.create_task(_livekit_connect(args.robot_id, player))

    # Xử lý SIGINT / SIGTERM
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: (bridge_task.cancel(), livekit_task.cancel()))

    try:
        await asyncio.gather(bridge_task, livekit_task, return_exceptions=True)
    finally:
        logger.info("Shutting down ROS2 and audio...")
        if speaker:
            speaker.stop()
            speaker.close()
        node.destroy_node()
        rclpy.shutdown()
        logger.info("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Wall-E Real Robot Bridge for Tour System")
    parser.add_argument(
        "--backend",
        default=os.getenv("BACKEND_WS_URL", "ws://mai.emobiz.vn:51102"),
        help="Backend WebSocket base URL (default: ws://mai.emobiz.vn:51102)",
    )
    parser.add_argument(
        "--ws-prefix",
        default=os.getenv("WS_PREFIX", ""),
        help="Path prefix if using subpath routing, e.g. /walle (default: empty)",
    )
    parser.add_argument(
        "--robot-id",
        default=os.getenv("ROBOT_ID", "walle_001"),
        help="Robot ID (must match backend config, default: walle_001)",
    )
    args = parser.parse_args()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
