"""
tour_editor/server.py
FastAPI server chạy trên Orange Pi / Ubuntu robot.
- Phục vụ Web UI
- Nhận tour_script.json từ trình duyệt
- Start / Stop tour_runner

Chạy: uvicorn server:app --host 0.0.0.0 --port 5000 --reload
"""
import json
import subprocess
import sys
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

BASE_DIR   = Path(__file__).parent
SCRIPT_DIR = BASE_DIR
TOUR_FILE  = BASE_DIR / "tour_script.json"

app = FastAPI(title="Tour Editor API")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)

# Serve static files (index.html, assets)
app.mount("/static", StaticFiles(directory=str(BASE_DIR)), name="static")

_runner_proc: subprocess.Popen | None = None


# ── Models ──────────────────────────────────────────────────────────────────

class TourScript(BaseModel):
    waypoints: list[dict]   # [{label, x, y, yaw, script}]
    delay_between: float = 3.0
    map_resolution: float = 0.05
    map_origin: list[float] = [0.0, 0.0, 0.0]


# ── Routes ───────────────────────────────────────────────────────────────────

@app.get("/")
def index():
    return FileResponse(str(BASE_DIR / "web.html"))


@app.post("/api/upload_tour")
def upload_tour(tour: TourScript):
    """Nhận tour script từ Web UI và lưu vào file."""
    data = tour.model_dump()
    TOUR_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return {"status": "ok", "waypoints": len(tour.waypoints)}


@app.get("/api/get_tour")
def get_tour():
    if not TOUR_FILE.exists():
        return JSONResponse({"waypoints": [], "delay_between": 3.0})
    return JSONResponse(json.loads(TOUR_FILE.read_text(encoding="utf-8")))


@app.post("/api/start_tour")
def start_tour():
    global _runner_proc
    if _runner_proc and _runner_proc.poll() is None:
        return {"status": "already_running"}
    if not TOUR_FILE.exists():
        raise HTTPException(400, "Chưa có tour script. Hãy upload trước.")

    runner = SCRIPT_DIR / "tour_runner.py"
    
    # Tìm kiếm Python binary trong virtual environment
    python_bin = "python3"
    venv_python = BASE_DIR.parent.parent / "livekit" / ".venv" / "bin" / "python3"
    if venv_python.exists():
        python_bin = str(venv_python.resolve())
    else:
        venv_agent_python = BASE_DIR.parent.parent / "livekit-agent" / ".venv" / "bin" / "python3"
        if venv_agent_python.exists():
            python_bin = str(venv_agent_python.resolve())

    print(f"--> [SERVER] Kích hoạt chạy: {python_bin} {runner} --tour {TOUR_FILE}")
    _runner_proc = subprocess.Popen(
        [python_bin, str(runner), "--tour", str(TOUR_FILE)]
    )
    return {"status": "started", "pid": _runner_proc.pid}


@app.post("/api/stop_tour")
def stop_tour():
    global _runner_proc
    if _runner_proc and _runner_proc.poll() is None:
        _runner_proc.terminate()
        return {"status": "stopped"}
    return {"status": "not_running"}


@app.get("/api/tour_status")
def tour_status():
    if _runner_proc is None:
        return {"running": False}
    return {"running": _runner_proc.poll() is None, "pid": _runner_proc.pid}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("server:app", host="0.0.0.0", port=5000, reload=True)
