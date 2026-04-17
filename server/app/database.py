from __future__ import annotations

import sqlite3
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator


SCHEMA = """
PRAGMA journal_mode = WAL;
PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS readings (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    device_url TEXT NOT NULL,
    source TEXT,
    esp_index INTEGER,
    unix_time INTEGER NOT NULL,
    temperature_c REAL,
    humidity_pct REAL,
    aqi INTEGER,
    tvoc_ppb INTEGER,
    eco2_ppm INTEGER,
    ens_validity INTEGER,
    has_aht INTEGER NOT NULL DEFAULT 0,
    has_ens INTEGER NOT NULL DEFAULT 0,
    ingested_at INTEGER NOT NULL DEFAULT (unixepoch()),
    UNIQUE (
        device_url,
        unix_time,
        temperature_c,
        humidity_pct,
        aqi,
        tvoc_ppb,
        eco2_ppm,
        ens_validity,
        has_aht,
        has_ens
    )
);

CREATE INDEX IF NOT EXISTS idx_readings_time ON readings (unix_time);
CREATE INDEX IF NOT EXISTS idx_readings_device_time ON readings (device_url, unix_time);
CREATE UNIQUE INDEX IF NOT EXISTS idx_readings_dedupe ON readings (
    device_url,
    unix_time,
    COALESCE(temperature_c, -9999.0),
    COALESCE(humidity_pct, -9999.0),
    COALESCE(aqi, -1),
    COALESCE(tvoc_ppb, -1),
    COALESCE(eco2_ppm, -1),
    COALESCE(ens_validity, -1),
    has_aht,
    has_ens
);

CREATE TABLE IF NOT EXISTS sync_state (
    device_url TEXT PRIMARY KEY,
    next_since INTEGER NOT NULL DEFAULT 0,
    last_full_sync_at INTEGER,
    last_poll_at INTEGER,
    last_success_at INTEGER,
    last_error_at INTEGER,
    last_error TEXT,
    last_status_json TEXT,
    connected INTEGER NOT NULL DEFAULT 0,
    updated_at INTEGER NOT NULL DEFAULT (unixepoch())
);
"""


class Database:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._init_schema()

    @contextmanager
    def connect(self) -> Iterator[sqlite3.Connection]:
        conn = sqlite3.connect(self.path, timeout=30)
        conn.row_factory = sqlite3.Row
        try:
            conn.execute("PRAGMA foreign_keys = ON")
            yield conn
            conn.commit()
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

    def _init_schema(self) -> None:
        with sqlite3.connect(self.path, timeout=30) as conn:
            conn.executescript(SCHEMA)


def row_to_dict(row: sqlite3.Row | None) -> dict | None:
    if row is None:
        return None
    return {key: row[key] for key in row.keys()}
