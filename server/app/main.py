from __future__ import annotations

import json
import os
import sqlite3
import time
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .collector import CollectorConfig, SensorCollector
from .database import Database, row_to_dict


BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = Path(os.getenv("DATA_DIR", "/data"))
DB_PATH = Path(os.getenv("DATABASE_PATH", str(DATA_DIR / "sensor.sqlite3")))
SENSOR_BASE_URL = os.getenv("SENSOR_BASE_URL", "http://co2-sensor.local").rstrip("/")
POLL_INTERVAL_SEC = int(os.getenv("POLL_INTERVAL_SEC", "60"))
REQUEST_TIMEOUT_SEC = float(os.getenv("REQUEST_TIMEOUT_SEC", "30"))

db = Database(DB_PATH)
collector = SensorCollector(
    db,
    CollectorConfig(
        sensor_base_url=SENSOR_BASE_URL,
        poll_interval_sec=POLL_INTERVAL_SEC,
        request_timeout_sec=REQUEST_TIMEOUT_SEC,
    ),
)

app = FastAPI(title="CO2 Sensor Analysis Server")
app.mount("/static", StaticFiles(directory=BASE_DIR / "static"), name="static")


@app.on_event("startup")
async def startup() -> None:
    collector.start()


@app.on_event("shutdown")
async def shutdown() -> None:
    await collector.stop()


@app.get("/")
def index() -> FileResponse:
    return FileResponse(BASE_DIR / "static" / "index.html")


@app.post("/api/sync")
async def sync_now(full: bool = Query(False)) -> dict[str, Any]:
    try:
        return await collector.sync_now(full=full)
    except Exception as exc:
        raise HTTPException(status_code=502, detail=str(exc)) from exc


class ConfigUpdate(BaseModel):
    sensor_base_url: str


@app.get("/api/config")
def get_config() -> dict[str, Any]:
    return {
        "sensor_base_url": collector.config.sensor_base_url,
        "poll_interval_sec": collector.config.poll_interval_sec,
    }


@app.put("/api/config")
async def update_config(body: ConfigUpdate) -> dict[str, Any]:
    new_url = body.sensor_base_url.strip().rstrip("/")
    if not new_url:
        raise HTTPException(status_code=400, detail="sensor_base_url must not be empty")
    await collector.reconfigure(new_url)
    return {"sensor_base_url": new_url, "ok": True}


@app.get("/api/status")
def status() -> dict[str, Any]:
    device_url = collector.config.sensor_base_url
    with db.connect() as conn:
        state = row_to_dict(
            conn.execute(
                "SELECT * FROM sync_state WHERE device_url = ?",
                (device_url,),
            ).fetchone()
        )
        latest = row_to_dict(
            conn.execute(
                """
                SELECT * FROM readings
                WHERE device_url = ?
                ORDER BY unix_time DESC, id DESC
                LIMIT 1
                """,
                (device_url,),
            ).fetchone()
        )
        totals = row_to_dict(
            conn.execute(
                """
                SELECT
                    COUNT(*) AS reading_count,
                    MIN(unix_time) AS first_unix_time,
                    MAX(unix_time) AS latest_unix_time
                FROM readings
                WHERE device_url = ?
                """,
                (device_url,),
            ).fetchone()
        )

    if state and state.get("last_status_json"):
        state["device_status"] = json.loads(state["last_status_json"])
        del state["last_status_json"]

    return {
        "sensor_base_url": device_url,
        "poll_interval_sec": collector.config.poll_interval_sec,
        "server_time_unix": int(time.time()),
        "state": state,
        "latest": latest,
        "totals": totals,
    }


@app.get("/api/readings")
def readings(
    start: int | None = Query(None),
    end: int | None = Query(None),
    metric: str = Query("all"),
    limit: int = Query(5000, ge=1, le=50000),
) -> dict[str, Any]:
    clauses = ["device_url = ?"]
    params: list[Any] = [collector.config.sensor_base_url]
    if start is not None:
        clauses.append("unix_time >= ?")
        params.append(start)
    if end is not None:
        clauses.append("unix_time <= ?")
        params.append(end)

    metric_columns = {
        "temperature_c",
        "humidity_pct",
        "aqi",
        "tvoc_ppb",
        "eco2_ppm",
        "ens_validity",
    }
    if metric != "all":
        if metric not in metric_columns:
            raise HTTPException(status_code=400, detail="Unknown metric")
        clauses.append(f"{metric} IS NOT NULL")

    params.append(limit)
    where = " AND ".join(clauses)

    with db.connect() as conn:
        rows = conn.execute(
            f"""
            SELECT id, unix_time, temperature_c, humidity_pct, aqi, tvoc_ppb,
                   eco2_ppm, ens_validity, has_aht, has_ens, source
            FROM readings
            WHERE {where}
            ORDER BY unix_time ASC, id ASC
            LIMIT ?
            """,
            params,
        ).fetchall()
        total = conn.execute(
            f"SELECT COUNT(*) AS count FROM readings WHERE {where}",
            params[:-1],
        ).fetchone()["count"]

    return {
        "count": len(rows),
        "total_matching": total,
        "truncated": total > len(rows),
        "readings": [row_to_dict(row) for row in rows],
    }


@app.get("/api/summary")
def summary(
    start: int | None = Query(None),
    end: int | None = Query(None),
) -> dict[str, Any]:
    clauses = ["device_url = ?"]
    params: list[Any] = [collector.config.sensor_base_url]
    if start is not None:
        clauses.append("unix_time >= ?")
        params.append(start)
    if end is not None:
        clauses.append("unix_time <= ?")
        params.append(end)
    where = " AND ".join(clauses)

    metrics = ["temperature_c", "humidity_pct", "aqi", "tvoc_ppb", "eco2_ppm", "ens_validity"]
    with db.connect() as conn:
        result: dict[str, Any] = {}
        for metric in metrics:
            row = conn.execute(
                f"""
                SELECT
                    COUNT({metric}) AS count,
                    MIN({metric}) AS min,
                    AVG({metric}) AS avg,
                    MAX({metric}) AS max
                FROM readings
                WHERE {where}
                """,
                params,
            ).fetchone()
            result[metric] = row_to_dict(row)

        row = conn.execute(
            f"""
            SELECT COUNT(*) AS count, MIN(unix_time) AS start, MAX(unix_time) AS end
            FROM readings
            WHERE {where}
            """,
            params,
        ).fetchone()

    return {"range": row_to_dict(row), "metrics": result}


@app.get("/api/rollup")
def rollup(
    bucket: str = Query("hour", pattern="^(minute|hour|day)$"),
    start: int | None = Query(None),
    end: int | None = Query(None),
) -> dict[str, Any]:
    bucket_seconds = {"minute": 60, "hour": 3600, "day": 86400}[bucket]
    clauses = ["device_url = ?"]
    params: list[Any] = [collector.config.sensor_base_url]
    if start is not None:
        clauses.append("unix_time >= ?")
        params.append(start)
    if end is not None:
        clauses.append("unix_time <= ?")
        params.append(end)
    where = " AND ".join(clauses)

    with db.connect() as conn:
        rows = conn.execute(
            f"""
            SELECT
                (unix_time / ?) * ? AS bucket_start,
                COUNT(*) AS count,
                AVG(temperature_c) AS temperature_c,
                AVG(humidity_pct) AS humidity_pct,
                AVG(aqi) AS aqi,
                AVG(tvoc_ppb) AS tvoc_ppb,
                AVG(eco2_ppm) AS eco2_ppm,
                AVG(ens_validity) AS ens_validity
            FROM readings
            WHERE {where}
            GROUP BY bucket_start
            ORDER BY bucket_start ASC
            """,
            [bucket_seconds, bucket_seconds, *params],
        ).fetchall()

    return {
        "bucket": bucket,
        "bucket_seconds": bucket_seconds,
        "points": [row_to_dict(row) for row in rows],
    }


@app.get("/api/export.csv")
def export_csv(
    start: int | None = Query(None),
    end: int | None = Query(None),
) -> StreamingResponse:
    clauses = ["device_url = ?"]
    params: list[Any] = [collector.config.sensor_base_url]
    if start is not None:
        clauses.append("unix_time >= ?")
        params.append(start)
    if end is not None:
        clauses.append("unix_time <= ?")
        params.append(end)
    where = " AND ".join(clauses)

    def iter_csv():
        yield "unix_time,temperature_c,humidity_pct,aqi,tvoc_ppb,eco2_ppm,ens_validity,has_aht,has_ens,source\n"
        with db.connect() as conn:
            for row in conn.execute(
                f"""
                SELECT unix_time, temperature_c, humidity_pct, aqi, tvoc_ppb,
                       eco2_ppm, ens_validity, has_aht, has_ens, source
                FROM readings
                WHERE {where}
                ORDER BY unix_time ASC, id ASC
                """,
                params,
            ):
                yield ",".join("" if value is None else str(value) for value in row) + "\n"

    return StreamingResponse(
        iter_csv(),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=sensor-readings.csv"},
    )


@app.exception_handler(sqlite3.Error)
def sqlite_error_handler(_, exc: sqlite3.Error):
    return JSONResponse(status_code=500, content={"detail": str(exc)})
