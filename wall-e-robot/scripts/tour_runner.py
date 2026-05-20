#!/usr/bin/env python3
"""
tour_runner.py
--------------
Script chạy trên robot (Ubuntu).
Đọc kịch bản JSON (từ tour_editor Web UI) -> Navigate bằng Nav2 ->
Khi đến nơi thì lấy đoạn Text, gọi API Kokoro TTS (OpenAI compatible) ->
Phát ra loa đồng thời truyền phát lên LiveKit Room.

Dựa trên (based on) robot_client.py để kết nối LiveKit Cloud và quản lý âm thanh.
"""

import argparse
import json
import time
import sys
import os
import io
import math
import requests
import queue
import random
import string
import asyncio
import threading
import logging
from datetime import timedelta
import numpy as np
import sounddevice as sd
import soundfile as sf
# Hàm đọc file .env thủ công để không bị phụ thuộc vào thư viện python-dotenv
def manual_load_dotenv(env_path=None):
    if env_path is None:
        env_path = ".env"
    abs_path = os.path.abspath(env_path)
    print(f"--> [TOUR_RUNNER] Đang kiểm tra file .env tại: {abs_path}", flush=True)
    if not os.path.exists(env_path):
        print(f"--> [TOUR_RUNNER] Không tìm thấy file .env tại: {abs_path}", flush=True)
        return
    try:
        with open(env_path, 'r', encoding='utf-8') as f:
            count = 0
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if '=' in line:
                    key, val = line.split('=', 1)
                    key = key.strip()
                    val = val.strip().strip('"').strip("'")
                    os.environ[key] = val
                    count += 1
            print(f"--> [TOUR_RUNNER] Đã nạp thành công {count} biến môi trường từ: {abs_path}", flush=True)
    except Exception as e:
        print(f"--> [TOUR_RUNNER] Lỗi đọc file .env: {e}", flush=True)

from livekit import rtc
from livekit.api import AccessToken, VideoGrants

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# ── Cấu hình Log ─────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger("tour-runner")

# ── Tải file cấu hình .env từ các thư mục dự án ──────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
print(f"--> [TOUR_RUNNER] Thư mục script: {SCRIPT_DIR}", flush=True)
manual_load_dotenv(os.path.join(SCRIPT_DIR, ".env"))
manual_load_dotenv(os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "livekit", ".env")))
manual_load_dotenv(os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "livekit-agent", ".env")))

LIVEKIT_URL        = os.getenv("LIVEKIT_URL")
LIVEKIT_API_KEY    = os.getenv("LIVEKIT_API_KEY")
LIVEKIT_API_SECRET = os.getenv("LIVEKIT_API_SECRET")
ROOM_NAME          = os.getenv("ROOM_NAME", "robot-room")
IDENTITY_BASE      = os.getenv("IDENTITY", "robot-01")

# Tránh xung đột Identity khi reconnect nhanh
random_suffix = "".join(random.choices(string.ascii_lowercase + string.digits, k=4))
IDENTITY = f"{IDENTITY_BASE}-tour-{random_suffix}"

SAMPLE_RATE    = 48000
CHANNELS       = 1
CHUNK_SAMPLES  = int(SAMPLE_RATE * 10 / 1000)   # 10ms = 480 samples/frame

# Trạng thái đang phát âm thanh để tắt tiếng mic
is_speaking = False


# ── Generate token LiveKit ───────────────────────────────────────────────────
def generate_token() -> str:
    token = (
        AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
        .with_identity(IDENTITY)
        .with_name(IDENTITY)
        .with_grants(VideoGrants(room_join=True, room=ROOM_NAME))
        .with_ttl(timedelta(seconds=3600))
        .to_jwt()
    )
    logger.info(f"Token generated for identity={IDENTITY}, room={ROOM_NAME}")
    return token


# ── Playback Buffer cho Loa ──────────────────────────────────────────────────

class AudioPlayer:
    def __init__(self):
        self.queue = queue.Queue()
        self.buffer = np.array([], dtype=np.int16)
        
    def add_data(self, data: np.ndarray):
        try:
            self.queue.put_nowait(data)
        except queue.Full:
            pass

    def callback(self, outdata, frames, time_info, status):
        while len(self.buffer) < frames:
            try:
                chunk = self.queue.get_nowait()
                self.buffer = np.concatenate((self.buffer, chunk))
            except queue.Empty:
                break
        
        if len(self.buffer) >= frames:
            outdata[:] = self.buffer[:frames].reshape(-1, 1)
            self.buffer = self.buffer[frames:]
        else:
            outdata[:len(self.buffer)] = self.buffer.reshape(-1, 1)
            outdata[len(self.buffer):] = 0
            self.buffer = np.array([], dtype=np.int16)

audio_player = AudioPlayer()


async def receive_agent_audio(track: rtc.AudioTrack):
    audio_stream = rtc.AudioStream(track, sample_rate=SAMPLE_RATE, num_channels=CHANNELS)
    first_frame = True
    async for event in audio_stream:
        if isinstance(event, rtc.AudioFrameEvent):
            if first_frame:
                logger.info("[Audio] → Đã bắt đầu nhận dữ liệu âm thanh thực tế từ Agent! Đang đẩy vào loa...")
                first_frame = False
            data = np.frombuffer(event.frame.data, dtype=np.int16)
            audio_player.add_data(data)


# ── ROS 2 Navigation Helpers & Node ──────────────────────────────────────────

def deg_to_quat(yaw_deg: float):
    yaw_rad = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))

def make_pose_stamped(x: float, y: float, yaw_deg: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = deg_to_quat(yaw_deg)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class TourRunner(Node):
    def __init__(self):
        super().__init__('tour_runner_node')
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    async def navigate_to(self, x: float, y: float, yaw_deg: float, label: str) -> bool:
        logger.info(f'Đang chờ Nav2 action server...')
        while not self._nav_client.wait_for_server(timeout_sec=1.0):
            logger.info('Đang đợi Nav2 action server...')
            await asyncio.sleep(1.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw_deg)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        logger.info(f'→ Gửi lệnh di chuyển đến {label}: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}°')

        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        
        # Chờ goal được chấp nhận một cách phi tuần tự (non-blocking)
        while not send_goal_future.done():
            await asyncio.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            logger.error(f'Vị trí {label} bị từ chối bởi Nav2!')
            return False

        logger.info(f'Vị trí {label} hợp lệ. Robot đang di chuyển...')

        result_future = goal_handle.get_result_async()
        
        # Chờ kết quả điều hướng (Timeout 120 giây)
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > 120.0:
                logger.error(f'Quá thời gian điều hướng đến {label} (Timeout 120s)!')
                return False
            await asyncio.sleep(0.5)

        result = result_future.result()
        if result is None:
            logger.error(f'Không nhận được kết quả di chuyển đến {label}!')
            return False

        if result.status == 4: # SUCCEEDED
            logger.info(f' Robot ĐÃ ĐẾN {label}!')
            return True
        else:
            logger.error(f' Không thể đến {label}! Lỗi status={result.status}')
            return False


# ── TTS & Audio Streaming ────────────────────────────────────────────────────

async def play_tts(text: str, room: rtc.Room):
    """Gửi text tới LiveKit room để Server Agent nhận diện và tự chuyển sang âm thanh."""
    logger.info(f"[TTS] Đang gửi text thuyết trình lên LiveKit: '{text}'...")
    try:
        # Cách 1: Sử dụng send_text (Phương thức chuẩn mới nhất của LiveKit SDK cho Text Stream)
        if hasattr(room.local_participant, "send_text"):
            await room.local_participant.send_text(text, topic="lk.chat")
            logger.info("[TTS] Đã gửi text thành công qua send_text (Text Stream).")
        else:
            # Cách 2: Fallback sử dụng publish_data (Data Packet) nếu SDK bản cũ
            import uuid
            # Gửi cả dạng JSON ChatMessage chuẩn của LiveKit Client để các Agent cũ/mới đều nhận diện được
            chat_payload = json.dumps({
                "id": str(uuid.uuid4()),
                "message": text,
                "timestamp": int(time.time() * 1000)
            })
            try:
                await room.local_participant.publish_data(chat_payload, topic="lk.chat", reliable=True)
            except TypeError:
                await room.local_participant.publish_data(chat_payload.encode('utf-8'), topic="lk.chat", reliable=True)
            logger.info("[TTS] Đã gửi text thành công qua publish_data (ChatMessage JSON).")
        
        # Ước lượng thời gian nói dựa trên số từ (khoảng 0.45s / từ tiếng Việt/Anh + 1.5s buffer)
        words = len(text.split())
        duration_sec = max(2.5, words * 0.45 + 1.5)
        logger.info(f"[TTS] Ước lượng thời gian thuyết trình của Agent: {duration_sec:.1f} giây. Robot tạm dừng chờ...")
        await asyncio.sleep(duration_sec)
        logger.info("[TTS] Đã hoàn thành thời gian chờ thuyết trình.")
    except Exception as e:
        logger.error(f"[LỖI TTS] Lỗi gửi text lên LiveKit: {e}")


# ── Loops ────────────────────────────────────────────────────────────────────

async def mic_capture_loop(mic_queue: asyncio.Queue, audio_source: rtc.AudioSource):
    """Loop đọc mic liên tục và gửi lên LiveKit, tắt tiếng khi đang tự thuyết trình."""
    global is_speaking
    try:
        while True:
            data = await mic_queue.get()
            if is_speaking:
                data = np.zeros_like(data)  # Gửi silence để tránh echo
            frame = rtc.AudioFrame(
                data=data.tobytes(),
                sample_rate=SAMPLE_RATE,
                num_channels=CHANNELS,
                samples_per_channel=len(data),
            )
            await audio_source.capture_frame(frame)
    except asyncio.CancelledError:
        pass


async def run_tour(runner: TourRunner, waypoints: list, delay_between: float, room: rtc.Room):
    """Loop di chuyển tuần tự qua các điểm dừng."""
    global is_speaking
    total = len(waypoints)
    try:
        for i, wp in enumerate(waypoints):
            label = wp.get('label', str(i+1))
            x, y, yaw = wp['x'], wp['y'], wp.get('yaw', 0.0)
            script = wp.get('script', '').strip()

            logger.info(f'\n[{i+1}/{total}] Chuẩn bị di chuyển đến Điểm {label} ...')
            
            # 1. Gọi Nav2 để di chuyển (async)
            success = await runner.navigate_to(x, y, yaw, label=f'Điểm {label}')

            if success:
                # 2. Khi đến nơi -> Thuyết trình TTS
                if script:
                    is_speaking = True
                    await play_tts(script, room)
                    is_speaking = False
                else:
                    logger.info(f"  (Không có kịch bản Text-to-Speech cho điểm {label})")

                # 3. Đợi delay_between trước khi qua điểm tiếp theo
                if i < total - 1:
                    logger.info(f'  ⏳ Tạm dừng {delay_between} giây trước khi qua điểm tiếp theo...')
                    await asyncio.sleep(delay_between)
            else:
                logger.error(f'  ✗ Bỏ qua điểm {label} do lỗi di chuyển...')
                await asyncio.sleep(delay_between)

        logger.info('\n' + '='*60)
        logger.info('  🏁 Đã kết thúc toàn bộ hành trình Hướng dẫn viên!')
        logger.info('='*60 + '\n')
    except Exception as e:
        logger.error(f"Lỗi trong quá trình chạy Tour: {e}")


def ros_spin(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        logger.error(f"ROS Spin interrupted: {e}")


# ── Main ─────────────────────────────────────────────────────────────────────
async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tour', type=str, required=True, help='Đường dẫn tới file tour_script.json')
    args = parser.parse_args()

    if not os.path.exists(args.tour):
        logger.error(f"Không tìm thấy file kịch bản JSON: {args.tour}")
        sys.exit(1)

    with open(args.tour, 'r', encoding='utf-8') as f:
        tour_data = json.load(f)

    waypoints = tour_data.get('waypoints', [])
    delay_between = tour_data.get('delay_between', 3.0)

    if not waypoints:
        logger.error("Kịch bản trống! Không có điểm dừng nào.")
        sys.exit(1)

    # Khởi tạo ROS 2 Node và chạy trong Thread riêng biệt
    rclpy.init()
    runner = TourRunner()
    ros_thread = threading.Thread(target=ros_spin, args=(runner,), daemon=True)
    ros_thread.start()

    # Khởi tạo LiveKit Room
    token = generate_token()
    room  = rtc.Room()
    loop  = asyncio.get_running_loop()

    @room.on("track_published")
    def on_track_published(publication, participant):
        logger.info(f"[LiveKit Event] Track được publish bởi {participant.identity}: id={publication.sid}, name={publication.name}, kind={publication.kind}")

    @room.on("track_subscribed")
    def on_track_subscribed(track, pub, participant):
        logger.info(f"[LiveKit Event] Track được subscribe thành công từ {participant.identity}: id={pub.sid}, kind={track.kind}")
        if track.kind == rtc.TrackKind.KIND_AUDIO:
            logger.info(f"Agent audio track subscribed from {participant.identity}")
            asyncio.ensure_future(receive_agent_audio(track))

    @room.on("track_subscription_failed")
    def on_track_subscription_failed(participant, track_sid, error):
        logger.error(f"[LiveKit Event] Lỗi subscribe track {track_sid} từ {participant.identity}: {error}")

    @room.on("participant_connected")
    def on_participant(p):
        logger.info(f"Participant joined: {p.identity}")

    @room.on("disconnected")
    def on_disconnected(reason):
        logger.warning(f"Room disconnected: {reason}")

    # Kết nối LiveKit Cloud
    await room.connect(LIVEKIT_URL, token)
    logger.info(f"Connected to LiveKit Cloud | room={room.name}")

    # In ra danh sách các participant đang có trong phòng và track của họ để kiểm tra
    logger.info(f"--- Danh sách participants hiện có trong phòng ({len(room.remote_participants)}): ---")
    for identity, p in room.remote_participants.items():
        logger.info(f"Participant: {identity}")
        for pub_id, pub in p.track_publications.items():
            logger.info(f"  - Track: id={pub.sid}, name={pub.name}, kind={pub.kind}, subscribed={pub.subscribed}")

    # Publish mic track
    audio_source = rtc.AudioSource(SAMPLE_RATE, CHANNELS)
    mic_track = rtc.LocalAudioTrack.create_audio_track("robot-mic", audio_source)
    await room.local_participant.publish_track(
        mic_track,
        rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_MICROPHONE),
    )
    logger.info("Microphone published to LiveKit!")

    # Start speaker output stream
    playback_stream = None
    try:
        playback_stream = sd.OutputStream(
            samplerate=SAMPLE_RATE, channels=CHANNELS,
            dtype="int16", blocksize=CHUNK_SAMPLES,
            callback=audio_player.callback,
        )
        playback_stream.start()
        logger.info("Speaker started — LiveKit audio will play here")
    except Exception as e:
        logger.error(f"❌ KHÔNG THỂ KHỞI ĐỘNG LOA (SoundDevice): {e}")
        await room.disconnect()
        runner.destroy_node()
        rclpy.shutdown()
        return

    # Capture mic thông qua async queue
    mic_queue = asyncio.Queue()

    def mic_callback(indata, frames, time_info, status):
        loop.call_soon_threadsafe(mic_queue.put_nowait, indata.copy())

    mic_stream = None
    try:
        mic_stream = sd.InputStream(
            samplerate=SAMPLE_RATE, channels=CHANNELS,
            dtype="int16", blocksize=CHUNK_SAMPLES,
            callback=mic_callback
        )
        mic_stream.start()
        logger.info("Microphone started successfully")
    except Exception as e:
        logger.error(f"❌ KHÔNG THỂ KHỞI ĐỘNG MICROPHONE (SoundDevice): {e}")
        if playback_stream:
            playback_stream.stop()
            playback_stream.close()
        await room.disconnect()
        runner.destroy_node()
        rclpy.shutdown()
        return

    logger.info("Listening & Ready! Bắt đầu chạy Tour...")

    # Chạy đồng thời 2 task: stream mic và chạy tour
    mic_task = asyncio.create_task(mic_capture_loop(mic_queue, audio_source))
    tour_task = asyncio.create_task(run_tour(runner, waypoints, delay_between, room))

    # Đợi tour chạy xong
    try:
        await tour_task
    except KeyboardInterrupt:
        logger.info("Tour interrupted by user.")
    finally:
        mic_task.cancel()
        try:
            await mic_task
        except asyncio.CancelledError:
            pass

        # Dọn dẹp thiết bị âm thanh và kết nối
        if mic_stream is not None:
            try:
                mic_stream.stop()
                mic_stream.close()
            except Exception:
                pass
        if playback_stream is not None:
            try:
                playback_stream.stop()
                playback_stream.close()
            except Exception:
                pass

        await room.disconnect()
        runner.destroy_node()
        rclpy.shutdown()
        logger.info("Disconnected and shutdown completed.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
