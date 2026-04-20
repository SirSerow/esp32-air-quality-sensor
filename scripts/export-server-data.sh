#!/usr/bin/env sh
set -eu

SERVICE_NAME="${SERVICE_NAME:-co2-analysis-server}"
CONTAINER_NAME="${CONTAINER_NAME:-co2-analysis-server}"
BACKUP_DIR="${BACKUP_DIR:-./backups}"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ARCHIVE_PATH="${1:-${BACKUP_DIR}/co2-sensor-data-${TIMESTAMP}.tgz}"

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker is not installed or not in PATH." >&2
    exit 1
fi

mkdir -p "$(dirname "$ARCHIVE_PATH")"
ARCHIVE_DIR="$(cd "$(dirname "$ARCHIVE_PATH")" && pwd)"
ARCHIVE_FILE="$(basename "$ARCHIVE_PATH")"

if ! docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
    echo "ERROR: container '$CONTAINER_NAME' does not exist." >&2
    echo "Start the server once with: docker compose up -d --build" >&2
    exit 1
fi

VOLUME_NAME="$(
    docker inspect "$CONTAINER_NAME" \
        --format '{{ range .Mounts }}{{ if eq .Destination "/data" }}{{ .Name }}{{ end }}{{ end }}'
)"

if [ -z "$VOLUME_NAME" ]; then
    echo "ERROR: could not find the Docker volume mounted at /data." >&2
    exit 1
fi

WAS_RUNNING="$(
    docker inspect "$CONTAINER_NAME" \
        --format '{{ if .State.Running }}yes{{ else }}no{{ end }}'
)"

if [ "$WAS_RUNNING" = "yes" ]; then
    echo "Stopping $SERVICE_NAME for a consistent SQLite backup..."
    docker compose stop "$SERVICE_NAME" >/dev/null
fi

cleanup() {
    if [ "$WAS_RUNNING" = "yes" ]; then
        echo "Restarting $SERVICE_NAME..."
        docker compose up -d "$SERVICE_NAME" >/dev/null
    fi
}
trap cleanup EXIT INT TERM

echo "Creating backup from Docker volume '$VOLUME_NAME'..."
docker run --rm \
    -v "$VOLUME_NAME:/data:ro" \
    -v "$ARCHIVE_DIR:/backup" \
    alpine:3.20 \
    tar czf "/backup/${ARCHIVE_FILE}" -C /data .

echo "Backup created: $ARCHIVE_PATH"
echo "Transfer it to the Raspberry Pi, for example:"
echo "  scp '$ARCHIVE_PATH' pi@raspberrypi.local:/home/pi/co2-sensor/"
