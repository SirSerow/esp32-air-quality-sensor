# CO2 Sensor Analysis Server

Dockerized web server for collecting ESP32 sensor history, storing it in SQLite,
and serving an analysis dashboard.

## Run with Docker Compose

From the repository root:

```bash
SENSOR_BASE_URL=http://192.168.1.50 docker compose up --build -d
```

Then open:

```text
http://localhost:8000/
```

Useful environment variables:

- `SENSOR_BASE_URL`: ESP32 base URL, for example `http://192.168.1.50`.
- `SERVER_PORT`: host port for the dashboard, default `8000`.
- `POLL_INTERVAL_SEC`: background sync interval, default `60`.
- `REQUEST_TIMEOUT_SEC`: ESP32 request timeout, default `30`.

SQLite data is stored in the Docker volume `co2_sensor_data`.

Use the ESP32 STA IP address from the serial boot log when possible. The
`co2-sensor.local` mDNS name can work on the host machine but often does not
resolve from inside a Linux container unless the host is specifically set up for
container mDNS.

## Sync Behavior

The collector uses the firmware API under `/api/v1`:

1. Reads `/api/v1/status` to update connection and device state.
2. If the local database is empty, imports from `/api/v1/sync?since=0`.
3. Continues requesting sync batches while `has_more` is true.
4. Stores the returned `next_since` cursor.
5. Polls incrementally on the configured interval.

Use the dashboard `Full import` button or:

```bash
curl -X POST 'http://localhost:8000/api/sync?full=true'
```

to force a fresh ESP32 history scan. Duplicate rows are ignored by SQLite.

## HTTP API

- `GET /api/status`: collector state, connection status, latest reading, and database totals.
- `POST /api/sync?full=false`: trigger an immediate sync.
- `GET /api/readings?start=<unix>&end=<unix>&limit=5000`: raw readings.
- `GET /api/summary?start=<unix>&end=<unix>`: min, average, max per metric.
- `GET /api/rollup?bucket=minute|hour|day`: averaged chart points.
- `GET /api/export.csv`: CSV export for the selected time range.
