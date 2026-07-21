#!/usr/bin/env python3
"""将 bags/ + jobs.jsonl + insert_extract 一键打成交付目录（可选 tar.gz）。

用法:
  ./pack_delivery.py
  ./pack_delivery.py --no-tar --out delivery/my_pack
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tarfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent


def _parse_iso(ts: Any) -> Optional[datetime]:
    if ts is None:
        return None
    text = str(ts).strip()
    if not text or text.startswith("<"):
        return None
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    try:
        return datetime.fromisoformat(text)
    except ValueError:
        return None


def _bag_time_window_s(bag_dir: Path) -> Optional[Tuple[float, float]]:
    meta_path = bag_dir / "metadata.yaml"
    if not meta_path.is_file():
        return None

    start_ns: Optional[int] = None
    duration_ns: Optional[int] = None

    try:
        import yaml  # type: ignore
    except ImportError:
        yaml = None

    if yaml is not None:
        with meta_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        info = (data or {}).get("rosbag2_bagfile_information") or {}
        raw_start = (info.get("starting_time") or {}).get("nanoseconds_since_epoch")
        raw_dur = (info.get("duration") or {}).get("nanoseconds")
        if raw_start is not None:
            start_ns = int(raw_start)
        if raw_dur is not None:
            duration_ns = int(raw_dur)
    else:
        in_start = False
        in_duration = False
        for line in meta_path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if stripped.startswith("starting_time:"):
                in_start = True
                in_duration = False
                continue
            if stripped.startswith("duration:"):
                in_duration = True
                in_start = False
                continue
            if in_start and stripped.startswith("nanoseconds_since_epoch:"):
                try:
                    start_ns = int(stripped.split(":", 1)[1].strip())
                except ValueError:
                    pass
                in_start = False
            elif in_duration and stripped.startswith("nanoseconds:"):
                try:
                    duration_ns = int(stripped.split(":", 1)[1].strip())
                except ValueError:
                    pass
                in_duration = False

    if start_ns is None or duration_ns is None:
        return None
    start_s = float(start_ns) * 1.0e-9
    end_s = start_s + float(duration_ns) * 1.0e-9
    return start_s, end_s


def _row_time_window_s(row: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    start_dt = _parse_iso(row.get("started_timestamp"))
    end_dt = _parse_iso(row.get("finished_timestamp"))
    if start_dt is None and end_dt is None:
        return None
    if start_dt is None:
        start_dt = end_dt
    if end_dt is None:
        end_dt = start_dt
    assert start_dt is not None and end_dt is not None
    if start_dt.tzinfo is None:
        start_dt = start_dt.replace(tzinfo=timezone.utc)
    if end_dt.tzinfo is None:
        end_dt = end_dt.replace(tzinfo=timezone.utc)
    return start_dt.timestamp(), end_dt.timestamp()


def _overlaps(a: Tuple[float, float], b: Tuple[float, float], slack_s: float = 5.0) -> bool:
    return a[0] <= b[1] + slack_s and b[0] <= a[1] + slack_s


def _is_placeholder_row(row: Dict[str, Any]) -> bool:
    for key in ("job_id", "started_timestamp", "finished_timestamp"):
        val = row.get(key)
        if isinstance(val, str) and ("<" in val or "job_event" in val):
            return True
    return False


def load_jobs_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.is_file():
        return []
    rows: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                obj = json.loads(text)
            except json.JSONDecodeError as exc:
                print(
                    "[pack_delivery] WARN: jobs.jsonl L%d 解析失败: %s" % (line_no, exc),
                    file=sys.stderr,
                )
                continue
            if not isinstance(obj, dict):
                continue
            if _is_placeholder_row(obj):
                continue
            rows.append(obj)
    return rows


def filter_jobs_for_episode(
    rows: Sequence[Dict[str, Any]],
    episode_id: str,
    bag_window: Optional[Tuple[float, float]],
) -> Tuple[List[Dict[str, Any]], str]:
    """返回 (过滤后的行, 选择策略说明)。"""
    ep_rows = [r for r in rows if str(r.get("episode_id", "")) == episode_id]
    if not ep_rows:
        return [], "no_jobs"

    if bag_window is not None:
        matched: List[Dict[str, Any]] = []
        for row in ep_rows:
            row_win = _row_time_window_s(row)
            if row_win is None:
                continue
            if _overlaps(row_win, bag_window):
                matched.append(row)
        if matched:
            return matched, "bag_time_overlap"

    # 回退：保留该 episode 最后一次“完整尝试”
    # 从尾部找：若末行是 INSERT，连同其前一条同次 PICK；否则只取末行
    last = ep_rows[-1]
    selected = [last]
    task = str(last.get("task_name", ""))
    if task == "TARGET_INSERT" and len(ep_rows) >= 2:
        prev = ep_rows[-2]
        if str(prev.get("task_name", "")) == "PICK" and str(prev.get("episode_id", "")) == episode_id:
            selected = [prev, last]
    elif task == "PICK":
        # 若后面还有 INSERT 属于同一次（已是末行则没有）
        pass
    return selected, "fallback_last_attempt"


def list_episode_bags(bags_root: Path) -> List[Path]:
    if not bags_root.is_dir():
        return []
    out: List[Path] = []
    for child in sorted(bags_root.iterdir()):
        if not child.is_dir():
            continue
        if child.name.startswith("."):
            continue
        if (child / "metadata.yaml").is_file() or any(child.glob("*.db3")) or any(child.glob("*.mcap")):
            out.append(child)
    return out


def write_jsonl(path: Path, rows: Sequence[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def write_delivery_readme(path: Path, stamp: str, episodes: Sequence[str], warnings: Sequence[str]) -> None:
    lines = [
        "# arm_bag 交付包",
        "",
        "生成时间: %s" % stamp,
        "",
        "## 内容",
        "",
        "- 每个 episode 子目录含 `bag/`、过滤后的 `jobs.jsonl`、可选 `insert_extract/`",
        "- 根目录 `jobs.jsonl` 为全部 episode 过滤后的合集",
        "- `topic_list.txt`：录制话题说明",
        "",
        "## Episodes",
        "",
    ]
    for eid in episodes:
        lines.append("- `%s`" % eid)
    lines.append("")
    if warnings:
        lines.append("## Warnings")
        lines.append("")
        for w in warnings:
            lines.append("- %s" % w)
        lines.append("")
    lines.extend(
        [
            "## 插孔训练数据",
            "",
            "若存在 `insert_extract/transitions.npz`：",
            "",
            "```python",
            "import numpy as np",
            "d = np.load('ep_xxx/insert_extract/transitions.npz', allow_pickle=True)",
            "print(d['s'].shape, d['a'].shape, d['s_next'].shape)",
            "```",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def make_tarball(src_dir: Path, tar_path: Path) -> None:
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(src_dir, arcname=src_dir.name)


def pack_delivery(
    script_dir: Path,
    out_dir: Path,
    jobs_path: Path,
    bags_root: Path,
    extract_root: Path,
    make_tar: bool,
) -> int:
    bag_dirs = list_episode_bags(bags_root)
    if not bag_dirs:
        print("[pack_delivery] 错误: %s 下无可用 bag 目录" % bags_root, file=sys.stderr)
        return 1

    all_jobs = load_jobs_jsonl(jobs_path)
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
    aggregated: List[Dict[str, Any]] = []
    episodes: List[str] = []
    warnings: List[str] = []

    topic_src = script_dir / "topic_list.txt"
    if topic_src.is_file():
        shutil.copy2(topic_src, out_dir / "topic_list.txt")
    else:
        warnings.append("缺少 topic_list.txt")

    for bag_dir in bag_dirs:
        episode_id = bag_dir.name
        episodes.append(episode_id)
        ep_out = out_dir / episode_id
        ep_out.mkdir(parents=True, exist_ok=True)

        bag_dst = ep_out / "bag"
        shutil.copytree(bag_dir, bag_dst)

        bag_window = _bag_time_window_s(bag_dir)
        if bag_window is None:
            warnings.append("%s: 无法从 metadata.yaml 解析时间窗" % episode_id)

        selected, strategy = filter_jobs_for_episode(all_jobs, episode_id, bag_window)
        if not selected:
            warnings.append("%s: jobs.jsonl 无匹配行" % episode_id)
        else:
            if strategy == "fallback_last_attempt":
                warnings.append(
                    "%s: jobs 与 bag 时间窗无重叠，回退为最后一次记录 (%d 行)"
                    % (episode_id, len(selected))
                )
            write_jsonl(ep_out / "jobs.jsonl", selected)
            aggregated.extend(selected)

        extract_src = extract_root / episode_id
        if extract_src.is_dir():
            shutil.copytree(extract_src, ep_out / "insert_extract")
        else:
            warnings.append("%s: 无 insert_extract（可先跑 extract_insert_trajectories）" % episode_id)

        print(
            "[pack_delivery] %s: jobs=%d strategy=%s extract=%s"
            % (
                episode_id,
                len(selected),
                strategy,
                "yes" if extract_src.is_dir() else "no",
            )
        )

    write_jsonl(out_dir / "jobs.jsonl", aggregated)
    write_delivery_readme(out_dir / "README_DELIVERY.md", stamp, episodes, warnings)

    summary = {
        "generated_at": stamp,
        "episode_count": len(episodes),
        "episodes": episodes,
        "job_rows": len(aggregated),
        "warnings": list(warnings),
        "source_bags": str(bags_root),
        "source_jobs": str(jobs_path),
        "source_extract": str(extract_root),
    }
    (out_dir / "manifest.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    if make_tar:
        tar_path = out_dir.with_suffix(".tar.gz")
        if tar_path.exists():
            tar_path.unlink()
        # 命名: delivery/arm_bag_delivery_xxx.tar.gz 与目录同级
        make_tarball(out_dir, tar_path)
        print("[pack_delivery] tar: %s" % tar_path)

    print("[pack_delivery] 完成: %s (%d episodes)" % (out_dir, len(episodes)))
    if warnings:
        print("[pack_delivery] warnings=%d（见 README_DELIVERY.md）" % len(warnings))
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="打包 arm_bag_tools 交付目录")
    parser.add_argument(
        "--out",
        type=str,
        default="",
        help="输出目录（默认 delivery/arm_bag_delivery_YYYYMMDD_HHMMSS）",
    )
    parser.add_argument(
        "--jobs-jsonl",
        type=str,
        default=str(SCRIPT_DIR / "jobs.jsonl"),
        help="源 jobs.jsonl 路径",
    )
    parser.add_argument(
        "--bags-root",
        type=str,
        default=str(SCRIPT_DIR / "bags"),
        help="bags 根目录",
    )
    parser.add_argument(
        "--extract-root",
        type=str,
        default=str(SCRIPT_DIR / "analysis" / "insert_extract"),
        help="insert_extract 根目录",
    )
    parser.add_argument(
        "--no-tar",
        action="store_true",
        help="不生成 .tar.gz",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.out:
        out_dir = Path(args.out).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = (SCRIPT_DIR / "delivery" / ("arm_bag_delivery_%s" % stamp)).resolve()

    return pack_delivery(
        script_dir=SCRIPT_DIR,
        out_dir=out_dir,
        jobs_path=Path(args.jobs_jsonl).expanduser().resolve(),
        bags_root=Path(args.bags_root).expanduser().resolve(),
        extract_root=Path(args.extract_root).expanduser().resolve(),
        make_tar=not args.no_tar,
    )


if __name__ == "__main__":
    sys.exit(main())
