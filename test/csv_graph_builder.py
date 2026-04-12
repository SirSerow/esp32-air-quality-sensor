"""Generate sensor graphs from a CO2 sensor CSV log file.

This script loads a CSV exported by the firmware endpoint (`/csv`) and builds
one PNG graph for each reading value.
"""

from __future__ import annotations

import argparse
import csv
import importlib
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


@dataclass(slots=True)
class SensorReading:
    """Represents one sensor row parsed from the firmware CSV format."""

    unix_time: int
    temp_c: float
    humidity_pct: float
    eco2_ppm: int
    tvoc_ppb: int
    aqi: int
    ens_validity: int
    has_aht: bool
    has_ens: bool


class SensorCsvGraphBuilder:
    """Loads a sensor CSV file and creates per-metric line charts."""

    def __init__(self, output_dir: Path) -> None:
        """Initialize the graph builder.

        Args:
            output_dir: Directory where generated graph files are saved.
        """
        self.output_dir = output_dir
        self.readings: list[SensorReading] = []
        self.filtered_out_of_year_count = 0

    @staticmethod
    def _load_pyplot() -> Any:
        """Load ``matplotlib.pyplot`` lazily at runtime.

        Returns:
            The imported ``matplotlib.pyplot`` module object.

        Raises:
            RuntimeError: If matplotlib is not installed in the active environment.
        """
        try:
            return importlib.import_module("matplotlib.pyplot")
        except ModuleNotFoundError as exc:
            msg = (
                "matplotlib is required for graph generation. "
                "Install it with: pip install matplotlib"
            )
            raise RuntimeError(msg) from exc

    def upload_csv_file(self, csv_path: Path) -> None:
        """Upload (ingest) a CSV file into the graph builder state.

        Args:
            csv_path: Path to the CSV file exported by the firmware.

        Raises:
            FileNotFoundError: If the file does not exist.
            ValueError: If no valid rows are found.
        """
        if not csv_path.exists():
            msg = f"CSV file not found: {csv_path}"
            raise FileNotFoundError(msg)

        parsed: list[SensorReading] = []
        with csv_path.open("r", newline="", encoding="utf-8") as file_handle:
            reader = csv.DictReader(file_handle)
            for row in reader:
                reading = self._parse_row(row)
                if reading is not None:
                    parsed.append(reading)

        if not parsed:
            msg = "No valid sensor rows were found in the CSV file."
            raise ValueError(msg)

        cleaned = self._filter_current_year_readings(parsed)
        self.filtered_out_of_year_count = len(parsed) - len(cleaned)

        if not cleaned:
            msg = "No sensor rows remain after filtering to the current year."
            raise ValueError(msg)

        self.readings = cleaned

    @staticmethod
    def _filter_current_year_readings(
        readings: list[SensorReading],
    ) -> list[SensorReading]:
        """Keep only readings that belong to the current local year.

        Args:
            readings: Parsed sensor rows.

        Returns:
            A new list containing rows whose ``unix_time`` is in the current year.
        """
        current_year = datetime.now().year
        return [
            reading
            for reading in readings
            if datetime.fromtimestamp(reading.unix_time).year == current_year
        ]

    def download_csv_file(self, csv_url: str, destination: Path) -> Path:
        """Download a CSV file from an HTTP endpoint.

        Args:
            csv_url: URL of the CSV endpoint (for example, ``http://192.168.4.1/csv``).
            destination: Local path where the downloaded CSV is stored.

        Returns:
            The path to the downloaded CSV file.

        Raises:
            RuntimeError: If the URL cannot be downloaded successfully.
        """
        destination.parent.mkdir(parents=True, exist_ok=True)

        try:
            with urlopen(csv_url, timeout=15) as response:
                payload = response.read()
        except (HTTPError, URLError, TimeoutError) as exc:
            msg = f"Failed to download CSV from {csv_url}: {exc}"
            raise RuntimeError(msg) from exc

        destination.write_bytes(payload)
        return destination

    def filter_readings_by_period(self, period: str) -> int:
        """Filter `self.readings` to the requested recent period.

        Args:
            period: One of 'day', 'week', 'month', 'year', 'all'.

        Returns:
            The number of rows removed by the period filter.
        """
        if period == "all":
            return 0

        now = datetime.now()
        if period == "day":
            cutoff = now - timedelta(days=1)
        elif period == "week":
            cutoff = now - timedelta(days=7)
        elif period == "month":
            cutoff = now - timedelta(days=30)
        elif period == "year":
            cutoff = now - timedelta(days=365)
        else:
            raise ValueError(f"Unknown period: {period}")

        before = len(self.readings)
        self.readings = [
            r for r in self.readings if datetime.fromtimestamp(r.unix_time) >= cutoff
        ]
        return before - len(self.readings)

    def build_graphs(self) -> list[Path]:
        """Build one graph image for each reading value.

        Returns:
            A list of file paths to the generated PNG charts.

        Raises:
            ValueError: If no CSV data has been uploaded first.
        """
        if not self.readings:
            msg = "No sensor data loaded. Call upload_csv_file first."
            raise ValueError(msg)

        self.output_dir.mkdir(parents=True, exist_ok=True)

        generated_files = [
            self._plot_metric(
                filename="temperature_c.png",
                title="Temperature (°C)",
                y_label="Temperature (°C)",
                extractor=lambda reading: reading.temp_c if reading.has_aht else None,
                color="#d14",
            ),
            self._plot_metric(
                filename="humidity_pct.png",
                title="Humidity (%)",
                y_label="Humidity (%)",
                extractor=lambda reading: (
                    reading.humidity_pct if reading.has_aht else None
                ),
                color="#06c",
            ),
            self._plot_metric(
                filename="aqi.png",
                title="AQI",
                y_label="AQI",
                extractor=lambda reading: (
                    float(reading.aqi) if reading.has_ens else None
                ),
                color="#1a7f37",
            ),
            self._plot_metric(
                filename="tvoc_ppb.png",
                title="TVOC (ppb)",
                y_label="TVOC (ppb)",
                extractor=lambda reading: (
                    float(reading.tvoc_ppb) if reading.has_ens else None
                ),
                color="#a36a00",
            ),
            self._plot_metric(
                filename="eco2_ppm.png",
                title="eCO2 (ppm)",
                y_label="eCO2 (ppm)",
                extractor=lambda reading: (
                    float(reading.eco2_ppm) if reading.has_ens else None
                ),
                color="#6f42c1",
            ),
            self._plot_metric(
                filename="ens_validity.png",
                title="ENS Validity",
                y_label="Validity",
                extractor=lambda reading: float(reading.ens_validity),
                color="#444",
            ),
        ]

        return generated_files

    def _plot_metric(
        self,
        *,
        filename: str,
        title: str,
        y_label: str,
        extractor: Callable[[SensorReading], float | None],
        color: str,
    ) -> Path:
        """Render a single metric chart from loaded readings.

        Args:
            filename: Output PNG file name.
            title: Chart title.
            y_label: Y-axis label.
            extractor: Function that extracts a metric value from a reading.
            color: Line color for the chart.

        Returns:
            The output file path of the generated chart image.
        """
        x_values = [self._as_datetime(reading.unix_time) for reading in self.readings]
        y_values = [extractor(reading) for reading in self.readings]

        plt = self._load_pyplot()

        figure, axis = plt.subplots(figsize=(12, 4))
        axis.plot(x_values, y_values, color=color, linewidth=2)
        axis.set_title(title)
        axis.set_xlabel("Time")
        axis.set_ylabel(y_label)
        axis.grid(True, linestyle="--", alpha=0.3)
        figure.autofmt_xdate()

        output_path = self.output_dir / filename
        figure.tight_layout()
        figure.savefig(str(output_path), dpi=150)
        plt.close(figure)
        return output_path

    @staticmethod
    def _as_datetime(unix_time: int) -> datetime:
        """Convert Unix epoch seconds into a local datetime instance.

        Args:
            unix_time: Unix epoch seconds from CSV.

        Returns:
            A local datetime object used for X-axis plotting.
        """
        return datetime.fromtimestamp(unix_time)

    @staticmethod
    def _parse_row(row: dict[str, str]) -> SensorReading | None:
        """Parse one CSV dictionary row into a typed sensor reading.

        Args:
            row: Raw CSV row provided by ``csv.DictReader``.

        Returns:
            A parsed ``SensorReading`` object, or ``None`` when row parsing fails.
        """
        try:
            unix_time = int(row["unix_time"])
            temp_x100 = int(row["temp_x100"])
            humidity_x100 = int(row["humidity_x100"])
            eco2_ppm = int(row["eco2_ppm"])
            tvoc_ppb = int(row["tvoc_ppb"])
            aqi = int(row["aqi"])
            ens_validity = int(row["ens_validity"])
            has_aht = bool(int(row["has_aht"]))
            has_ens = bool(int(row["has_ens"]))
        except (KeyError, TypeError, ValueError):
            return None

        return SensorReading(
            unix_time=unix_time,
            temp_c=temp_x100 / 100.0,
            humidity_pct=humidity_x100 / 100.0,
            eco2_ppm=eco2_ppm,
            tvoc_ppb=tvoc_ppb,
            aqi=aqi,
            ens_validity=ens_validity,
            has_aht=has_aht,
            has_ens=has_ens,
        )


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for CSV graph generation."""
    parser = argparse.ArgumentParser(
        description=(
            "Upload a local sensor CSV file or download one via HTTP, "
            "then build graphs for each reading value."
        ),
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        type=Path,
        help="Path to a local source CSV file",
    )
    parser.add_argument(
        "--url",
        type=str,
        help="HTTP URL to download CSV from ESP32 (example: http://192.168.4.1/csv)",
    )
    parser.add_argument(
        "--download-path",
        type=Path,
        default=Path("test/downloaded_logs.csv"),
        help="Local path used when downloading CSV via --url",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("test/graphs"),
        help="Directory where PNG graphs are saved (default: test/graphs)",
    )
    parser.add_argument(
        "--period",
        type=str,
        choices=["day", "week", "month", "year", "all"],
        default=None,
        help=(
            "Optional graphing period to restrict plotted data to the last "
            "day/week/month/year or 'all' for no period filtering. If not set, "
            "no period filtering is applied."
        ),
    )
    args = parser.parse_args()

    if args.csv_file is None and not args.url:
        parser.error("Provide either a local csv_file or --url.")
    if args.csv_file is not None and args.url:
        parser.error("Use either csv_file or --url, not both.")

    return args


def main() -> None:
    """Run the CSV upload + graph generation workflow from CLI."""
    args = parse_args()

    builder = SensorCsvGraphBuilder(output_dir=args.output_dir)
    if args.url:
        downloaded_path = builder.download_csv_file(args.url, args.download_path)
        print(f"Downloaded CSV: {downloaded_path}")
        builder.upload_csv_file(downloaded_path)
    else:
        builder.upload_csv_file(args.csv_file)

    if builder.filtered_out_of_year_count > 0:
        print(
            "Filtered out "
            f"{builder.filtered_out_of_year_count} row(s) outside the current year."
        )

    # Apply optional period filtering (day/week/month/year/all)
    if args.period:
        filtered_by_period = builder.filter_readings_by_period(args.period)
        if filtered_by_period > 0:
            print(
                "Filtered out "
                f"{filtered_by_period} row(s) outside the selected period ({args.period})."
            )

    generated = builder.build_graphs()

    print("Generated graph files:")
    for graph_path in generated:
        print(f"- {graph_path}")


if __name__ == "__main__":
    """
    Example usage:
    - Upload from local file:
      python csv_graph_builder.py test/sample_log.csv
    - Download from ESP32 and generate:
      python csv_graph_builder.py --url http://192.168.4.1/csv --download-path test/downloaded_logs.csv
    """
    main()
