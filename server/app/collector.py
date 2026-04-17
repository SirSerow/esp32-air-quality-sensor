from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass
from typing import Any
from urllib.parse import urljoin

import httpx

from .database import Database


LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class CollectorConfig:
    sensor_base_url: str
    poll_interval_sec: int = 60
    request_timeout_sec: float = 30.0
    sync_limit: int = 240


class SensorCollector:
    def __init__(self, db: Database, config: CollectorConfig) -> None:
        self.db = db
        self.config = config
        self._task: asyncio.Task[None] | None = None
        self._stop_event = asyncio.Event()

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._run(), name="sensor-collector")

    async def stop(self) -> None:
        self._stop_event.set()
        if self._task is not None:
            try:
                await asyncio.wait_for(self._task, timeout=10.0)
            except asyncio.TimeoutError:
                self._task.cancel()
        self._task = None

    async def reconfigure(self, new_url: str) -> None:
        await self.stop()
        self.config = CollectorConfig(
            sensor_base_url=new_url,
            poll_interval_sec=self.config.poll_interval_sec,
            request_timeout_sec=self.config.request_timeout_sec,
            sync_limit=self.config.sync_limit,
        )
        self._stop_event = asyncio.Event()
        self.start()

    async def sync_now(self, full: bool = False) -> dict[str, Any]:
        async with httpx.AsyncClient(timeout=self.config.request_timeout_sec) as client:
            try:
                return await self._sync_once(client, full=full)
            except Exception as exc:
                self._mark_error(self._format_error(exc))
                raise

    async def _run(self) -> None:
        async with httpx.AsyncClient(timeout=self.config.request_timeout_sec) as client:
            while not self._stop_event.is_set():
                try:
                    await self._sync_once(client, full=False)
                except Exception as exc:
                    message = self._format_error(exc)
                    LOGGER.warning("Sensor sync failed: %s", message)
                    self._mark_error(message)

                try:
                    await asyncio.wait_for(
                        self._stop_event.wait(),
                        timeout=max(5, self.config.poll_interval_sec),
                    )
                except asyncio.TimeoutError:
                    pass

    async def _sync_once(self, client: httpx.AsyncClient, full: bool) -> dict[str, Any]:
        status = await self._fetch_status(client)
        self._mark_status(status)

        database_was_empty = self._is_database_empty()
        since = 0 if full or database_was_empty else self._get_next_since()
        total_inserted = 0
        total_seen = 0
        batches = 0

        while True:
            payload = await self._fetch_sync_batch(client, since)
            entries = payload.get("entries", [])
            source = payload.get("source")
            inserted = self._insert_entries(entries, source)
            total_inserted += inserted
            total_seen += len(entries)
            batches += 1

            next_since = int(payload.get("next_since") or since)
            if next_since > since:
                since = next_since
                self._set_next_since(since)

            if not payload.get("has_more"):
                break

            if next_since <= since and not entries:
                raise RuntimeError("ESP32 sync cursor did not advance")

            await asyncio.sleep(0)

        now = int(time.time())
        with self.db.connect() as conn:
            conn.execute(
                """
                INSERT INTO sync_state (
                    device_url, next_since, last_full_sync_at, last_poll_at,
                    last_success_at, connected, last_error, updated_at
                )
                VALUES (?, ?, ?, ?, ?, 1, NULL, ?)
                ON CONFLICT(device_url) DO UPDATE SET
                    next_since = excluded.next_since,
                    last_full_sync_at = COALESCE(excluded.last_full_sync_at, sync_state.last_full_sync_at),
                    last_poll_at = excluded.last_poll_at,
                    last_success_at = excluded.last_success_at,
                    connected = 1,
                    last_error = NULL,
                    updated_at = excluded.updated_at
                """,
                (
                    self.config.sensor_base_url,
                    since,
                    now if full or database_was_empty else None,
                    now,
                    now,
                    now,
                ),
            )

        return {
            "connected": True,
            "batches": batches,
            "seen": total_seen,
            "inserted": total_inserted,
            "next_since": since,
        }

    async def _fetch_status(self, client: httpx.AsyncClient) -> dict[str, Any]:
        response = await client.get(urljoin(self.config.sensor_base_url.rstrip("/") + "/", "api/v1/status"))
        response.raise_for_status()
        return response.json()

    @staticmethod
    def _format_error(exc: Exception) -> str:
        message = str(exc).strip()
        if message:
            return message
        return exc.__class__.__name__

    async def _fetch_sync_batch(self, client: httpx.AsyncClient, since: int) -> dict[str, Any]:
        response = await client.get(
            urljoin(self.config.sensor_base_url.rstrip("/") + "/", "api/v1/sync"),
            params={"since": since, "limit": self.config.sync_limit},
        )
        response.raise_for_status()
        return response.json()

    def _insert_entries(self, entries: list[dict[str, Any]], source: str | None) -> int:
        if not entries:
            return 0

        rows = []
        for item in entries:
            unix_time = int(item.get("unix_time") or 0)
            if unix_time <= 0:
                continue
            has_aht = 1 if item.get("has_aht") else 0
            has_ens = 1 if item.get("has_ens") else 0
            rows.append(
                (
                    self.config.sensor_base_url,
                    source,
                    item.get("index"),
                    unix_time,
                    item.get("temperature_c") if has_aht else None,
                    item.get("humidity_pct") if has_aht else None,
                    item.get("aqi") if has_ens else None,
                    item.get("tvoc_ppb") if has_ens else None,
                    item.get("eco2_ppm") if has_ens else None,
                    item.get("ens_validity"),
                    has_aht,
                    has_ens,
                    int(time.time()),
                )
            )

        with self.db.connect() as conn:
            before = conn.total_changes
            conn.executemany(
                """
                INSERT OR IGNORE INTO readings (
                    device_url, source, esp_index, unix_time, temperature_c,
                    humidity_pct, aqi, tvoc_ppb, eco2_ppm, ens_validity,
                    has_aht, has_ens, ingested_at
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                rows,
            )
            return conn.total_changes - before

    def _is_database_empty(self) -> bool:
        with self.db.connect() as conn:
            row = conn.execute(
                "SELECT COUNT(*) AS count FROM readings WHERE device_url = ?",
                (self.config.sensor_base_url,),
            ).fetchone()
            return int(row["count"]) == 0

    def _get_next_since(self) -> int:
        with self.db.connect() as conn:
            row = conn.execute(
                "SELECT next_since FROM sync_state WHERE device_url = ?",
                (self.config.sensor_base_url,),
            ).fetchone()
            if row is not None:
                return int(row["next_since"])
            row = conn.execute(
                "SELECT COALESCE(MAX(unix_time), 0) AS next_since FROM readings WHERE device_url = ?",
                (self.config.sensor_base_url,),
            ).fetchone()
            return int(row["next_since"] or 0)

    def _set_next_since(self, next_since: int) -> None:
        now = int(time.time())
        with self.db.connect() as conn:
            conn.execute(
                """
                INSERT INTO sync_state (device_url, next_since, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(device_url) DO UPDATE SET
                    next_since = excluded.next_since,
                    updated_at = excluded.updated_at
                """,
                (self.config.sensor_base_url, next_since, now),
            )

    def _mark_status(self, status: dict[str, Any]) -> None:
        now = int(time.time())
        with self.db.connect() as conn:
            conn.execute(
                """
                INSERT INTO sync_state (
                    device_url, last_status_json, last_poll_at,
                    connected, last_error, updated_at
                )
                VALUES (?, ?, ?, 1, NULL, ?)
                ON CONFLICT(device_url) DO UPDATE SET
                    last_status_json = excluded.last_status_json,
                    last_poll_at = excluded.last_poll_at,
                    connected = 1,
                    last_error = NULL,
                    updated_at = excluded.updated_at
                """,
                (self.config.sensor_base_url, json.dumps(status), now, now),
            )

    def _mark_error(self, message: str) -> None:
        now = int(time.time())
        with self.db.connect() as conn:
            conn.execute(
                """
                INSERT INTO sync_state (
                    device_url, last_poll_at, last_error_at, last_error,
                    connected, updated_at
                )
                VALUES (?, ?, ?, ?, 0, ?)
                ON CONFLICT(device_url) DO UPDATE SET
                    last_poll_at = excluded.last_poll_at,
                    last_error_at = excluded.last_error_at,
                    last_error = excluded.last_error,
                    connected = 0,
                    updated_at = excluded.updated_at
                """,
                (self.config.sensor_base_url, now, now, message[:500], now),
            )
