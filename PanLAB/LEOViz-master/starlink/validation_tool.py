import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pandas as pd


PING_RTT_RE = re.compile(r"time[=<](?P<rtt>[0-9.]+)\s*ms")
PING_LOSS_RE = re.compile(
    r"(?P<transmitted>\d+)\s+packets transmitted,\s+"
    r"(?P<received>\d+)\s+(?:packets )?received(?:,.*?,)?\s+"
    r"(?P<loss>[0-9.]+)%\s+packet loss"
)


@dataclass
class ValidationDataset:
    run_id: str
    base_dir: Path
    date_dir: str

    @property
    def grpc_dir(self) -> Path:
        return self.base_dir / "grpc" / self.date_dir

    @property
    def latency_dir(self) -> Path:
        return self.base_dir / "latency" / self.date_dir

    @property
    def tle_dir(self) -> Path:
        return self.base_dir / "TLE" / self.date_dir

    def find_latency_file(self) -> Path | None:
        matches = sorted(self.latency_dir.glob(f"ping-*-{self.run_id}.txt"))
        return matches[0] if matches else None

    def find_files(self) -> dict[str, Path | None]:
        return {
            "obstruction_csv": self.base_dir / f"obstruction-data-{self.run_id}.csv",
            "processed_csv": self.base_dir / f"processed_obstruction-data-{self.run_id}.csv",
            "serving_csv": self.base_dir / f"serving_satellite_data-{self.run_id}.csv",
            "video_mp4": self.base_dir / f"starlink-{self.run_id}.mp4",
            "grpc_status_csv": self.grpc_dir / f"GRPC_STATUS-{self.run_id}.csv",
            "grpc_location_csv": self.grpc_dir / f"GRPC_LOCATION-{self.run_id}.csv",
            "obstruction_parquet": self.grpc_dir / f"obstruction_map-{self.run_id}.parquet",
            "tle_txt": self.tle_dir / f"starlink-tle-{self.run_id}.txt",
            "latency_txt": self.find_latency_file(),
        }


def auto_detect_run_id(base_dir: Path) -> str:
    serving_files = sorted(base_dir.glob("serving_satellite_data-*.csv"))
    if not serving_files:
        raise FileNotFoundError(
            f"在 {base_dir} 下找不到 serving_satellite_data-*.csv"
        )
    latest = max(serving_files, key=lambda path: path.stat().st_mtime)
    return latest.stem.removeprefix("serving_satellite_data-")


def parse_ping_file(path: Path | None) -> dict[str, Any]:
    if path is None or not path.exists():
        return {
            "file_present": False,
            "samples": 0,
            "avg_rtt_ms": None,
            "min_rtt_ms": None,
            "max_rtt_ms": None,
            "packet_loss_percent": None,
            "transmitted": None,
            "received": None,
        }

    rtts: list[float] = []
    transmitted = received = None
    packet_loss = None

    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        rtt_match = PING_RTT_RE.search(line)
        if rtt_match:
            rtts.append(float(rtt_match.group("rtt")))
        loss_match = PING_LOSS_RE.search(line)
        if loss_match:
            transmitted = int(loss_match.group("transmitted"))
            received = int(loss_match.group("received"))
            packet_loss = float(loss_match.group("loss"))

    return {
        "file_present": True,
        "samples": len(rtts),
        "avg_rtt_ms": round(sum(rtts) / len(rtts), 3) if rtts else None,
        "min_rtt_ms": round(min(rtts), 3) if rtts else None,
        "max_rtt_ms": round(max(rtts), 3) if rtts else None,
        "packet_loss_percent": packet_loss,
        "transmitted": transmitted,
        "received": received,
    }


def summarize_dataframe(df: pd.DataFrame, timestamp_column: str) -> dict[str, Any]:
    if df.empty:
        return {
            "rows": 0,
            "start": None,
            "end": None,
            "duration_seconds": 0,
        }

    ts = pd.to_datetime(df[timestamp_column], utc=True, errors="coerce").dropna()
    if ts.empty:
        return {
            "rows": len(df),
            "start": None,
            "end": None,
            "duration_seconds": 0,
        }

    start = ts.min()
    end = ts.max()
    return {
        "rows": int(len(df)),
        "start": start.isoformat(),
        "end": end.isoformat(),
        "duration_seconds": int((end - start).total_seconds()),
    }


def within_tolerance(
    outer_start: str | None,
    outer_end: str | None,
    inner_start: str | None,
    inner_end: str | None,
    tolerance_seconds: int = 1,
) -> bool:
    if None in (outer_start, outer_end, inner_start, inner_end):
        return False

    tolerance = pd.Timedelta(seconds=tolerance_seconds)
    outer_start_ts = pd.Timestamp(outer_start)
    outer_end_ts = pd.Timestamp(outer_end)
    inner_start_ts = pd.Timestamp(inner_start)
    inner_end_ts = pd.Timestamp(inner_end)
    return (
        outer_start_ts - tolerance <= inner_start_ts
        and outer_end_ts + tolerance >= inner_end_ts
    )


def build_report(base_dir: Path, run_id: str) -> dict[str, Any]:
    dataset = ValidationDataset(run_id=run_id, base_dir=base_dir, date_dir=run_id[:10])
    files = dataset.find_files()

    required = {
        "obstruction_csv",
        "processed_csv",
        "serving_csv",
        "grpc_status_csv",
        "obstruction_parquet",
        "tle_txt",
        "latency_txt",
    }
    file_status = {
        name: bool(path and path.exists())
        for name, path in files.items()
    }

    missing_required = sorted(
        name for name in required if not file_status.get(name, False)
    )

    processed_df = pd.read_csv(files["processed_csv"]) if file_status["processed_csv"] else pd.DataFrame()
    serving_df = pd.read_csv(files["serving_csv"]) if file_status["serving_csv"] else pd.DataFrame()
    grpc_status_df = pd.read_csv(files["grpc_status_csv"]) if file_status["grpc_status_csv"] else pd.DataFrame()

    serving_summary = summarize_dataframe(serving_df, "Timestamp") if not serving_df.empty else {
        "rows": 0,
        "start": None,
        "end": None,
        "duration_seconds": 0,
    }
    processed_summary = summarize_dataframe(processed_df, "timestamp") if not processed_df.empty else {
        "rows": 0,
        "start": None,
        "end": None,
        "duration_seconds": 0,
    }
    grpc_summary = summarize_dataframe(grpc_status_df, "timestamp") if not grpc_status_df.empty else {
        "rows": 0,
        "start": None,
        "end": None,
        "duration_seconds": 0,
    }

    switch_count = 0
    unique_satellites: list[str] = []
    avg_distance_km = None
    if not serving_df.empty and "Connected_Satellite" in serving_df.columns:
        sat_series = serving_df["Connected_Satellite"].fillna("")
        unique_satellites = [value for value in sat_series.drop_duplicates().tolist() if value]
        switch_count = int((sat_series != sat_series.shift(1)).sum() - 1)
        if "Distance" in serving_df.columns:
            avg_distance_km = round(pd.to_numeric(serving_df["Distance"], errors="coerce").dropna().mean(), 3)

    grpc_metrics = {}
    if not grpc_status_df.empty:
        for column in ("popPingLatencyMs", "downlinkThroughputBps", "uplinkThroughputBps", "sinr"):
            if column in grpc_status_df.columns:
                series = pd.to_numeric(grpc_status_df[column], errors="coerce").dropna()
                grpc_metrics[column] = {
                    "mean": round(series.mean(), 3) if not series.empty else None,
                    "min": round(series.min(), 3) if not series.empty else None,
                    "max": round(series.max(), 3) if not series.empty else None,
                }

    checks = {
        "required_files_present": not missing_required,
        "processed_rows_present": processed_summary["rows"] > 0,
        "serving_rows_present": serving_summary["rows"] > 0,
        "grpc_rows_present": grpc_summary["rows"] > 0,
        "video_present": file_status.get("video_mp4", False),
        "timestamp_alignment": (
            within_tolerance(
                processed_summary["start"],
                processed_summary["end"],
                serving_summary["start"],
                serving_summary["end"],
                tolerance_seconds=1,
            )
        ),
    }

    report = {
        "run_id": run_id,
        "base_dir": str(base_dir),
        "files": {name: (str(path) if path else None) for name, path in files.items()},
        "file_status": file_status,
        "missing_required_files": missing_required,
        "checks": checks,
        "processed_summary": processed_summary,
        "serving_summary": {
            **serving_summary,
            "unique_satellites": unique_satellites,
            "switch_count": switch_count,
            "avg_distance_km": avg_distance_km,
        },
        "grpc_summary": grpc_summary,
        "grpc_metrics": grpc_metrics,
        "latency_summary": parse_ping_file(files["latency_txt"]),
    }
    report["overall_pass"] = all(checks.values()) and not missing_required
    return report


def report_to_markdown(report: dict[str, Any]) -> str:
    lines = [
        f"# LEOViz 驗證報告",
        "",
        f"- 執行批次 ID: `{report['run_id']}`",
        f"- 資料目錄: `{report['base_dir']}`",
        f"- 整體是否通過: `{report['overall_pass']}`",
        "",
        "## 檢查項目",
    ]

    for name, value in report["checks"].items():
        lines.append(f"- `{name}`: `{value}`")

    if report["missing_required_files"]:
        lines.extend(["", "## 缺少的必要檔案"])
        for item in report["missing_required_files"]:
            lines.append(f"- `{item}`")

    lines.extend(
        [
            "",
            "## 資料摘要",
            f"- Processed 資料筆數: `{report['processed_summary']['rows']}`",
            f"- Serving satellite 資料筆數: `{report['serving_summary']['rows']}`",
            f"- gRPC 狀態資料筆數: `{report['grpc_summary']['rows']}`",
            f"- 唯一 serving satellites 數量: `{len(report['serving_summary']['unique_satellites'])}`",
            f"- 衛星切換次數: `{report['serving_summary']['switch_count']}`",
            f"- 平均 serving 距離（km）: `{report['serving_summary']['avg_distance_km']}`",
        ]
    )

    latency = report["latency_summary"]
    lines.extend(
        [
            "",
            "## 延遲摘要",
            f"- Ping 檔案存在: `{latency['file_present']}`",
            f"- RTT 樣本數: `{latency['samples']}`",
            f"- 平均 RTT（ms）: `{latency['avg_rtt_ms']}`",
            f"- 最小 RTT（ms）: `{latency['min_rtt_ms']}`",
            f"- 最大 RTT（ms）: `{latency['max_rtt_ms']}`",
            f"- 封包遺失率（%）: `{latency['packet_loss_percent']}`",
        ]
    )

    if report["grpc_metrics"]:
        lines.extend(["", "## gRPC 指標"])
        for metric, values in report["grpc_metrics"].items():
            lines.append(
                f"- `{metric}`: 平均值=`{values['mean']}`，最小值=`{values['min']}`，最大值=`{values['max']}`"
            )

    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="為 LEOViz 單次執行結果產生可重現的驗證報告。"
    )
    parser.add_argument(
        "--data-dir",
        default="tests/sample-data",
        help="LEOViz 輸出資料所在目錄。",
    )
    parser.add_argument(
        "--id",
        dest="run_id",
        help="執行批次 ID，例如 2025-05-29-23-20-13；若省略則自動偵測最新一筆。",
    )
    parser.add_argument(
        "--output",
        help="可選：輸出 Markdown 報告的路徑。",
    )
    parser.add_argument(
        "--json-output",
        help="可選：輸出原始 JSON 報告的路徑。",
    )
    args = parser.parse_args()

    base_dir = Path(args.data_dir).resolve()
    run_id = args.run_id or auto_detect_run_id(base_dir)
    report = build_report(base_dir, run_id)

    markdown = report_to_markdown(report)
    print(markdown)

    if args.output:
        output_path = Path(args.output)
        output_path.write_text(markdown, encoding="utf-8")

    if args.json_output:
        json_path = Path(args.json_output)
        json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
