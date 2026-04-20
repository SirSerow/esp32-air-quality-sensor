#!/usr/bin/env sh
set -eu

SERVICE_NAME="${SERVICE_NAME:-co2-analysis-server}"
CONTAINER_NAME="${CONTAINER_NAME:-co2-analysis-server}"
ARCHIVE_PATH="${1:-}"

if [ -z "$ARCHIVE_PATH" ]; then
    echo "Usage: $0 <co2-sensor-data-backup.tgz>" >&2
    exit 1
fi

if [ ! -f "$ARCHIVE_PATH" ]; then
    echo "ERROR: backup archive not found: $ARCHIVE_PATH" >&2
    exit 1
fi

ARCHIVE_DIR="$(cd "$(dirname "$ARCHIVE_PATH")" && pwd)"
ARCHIVE_FILE="$(basename "$ARCHIVE_PATH")"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker is not installed or not in PATH." >&2
    exit 1
fi

echo "Preparing Compose container and data volume..."
docker compose up --no-start --build "$SERVICE_NAME" >/dev/null

VOLUME_NAME="$(
    docker inspect "$CONTAINER_NAME" \
        --format '{{ range .Mounts }}{{ if eq .Destination "/data" }}{{ .Name }}{{ end }}{{ end }}'
)"

if [ -z "$VOLUME_NAME" ]; then
    echo "ERROR: could not find the Docker volume mounted at /data." >&2
    exit 1
fi

echo "Stopping $SERVICE_NAME before restore..."
docker compose stop "$SERVICE_NAME" >/dev/null

echo "Restoring '$ARCHIVE_PATH' into Docker volume '$VOLUME_NAME'..."
docker run --rm \
    -v "$VOLUME_NAME:/data" \
    -v "$ARCHIVE_DIR:/backup:ro" \
    -e "ARCHIVE_FILE=$ARCHIVE_FILE" \
    alpine:3.20 \
    sh -eu -c 'find /data -mindepth 1 -maxdepth 1 -exec rm -rf {} + && tar xzf "/backup/${ARCHIVE_FILE}" -C /data'

echo "Starting $SERVICE_NAME..."
docker compose up -d "$SERVICE_NAME" >/dev/null

echo "Restore complete."
echo "Verify with:"
echo "  curl http://localhost:${SERVER_PORT:-8000}/api/status"
