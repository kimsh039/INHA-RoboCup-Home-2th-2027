"""Create a separate six-class dataset, preserving coordinates and video splits."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "head_detection_ws" /
                       "src" / "robocup_head_detection"))
from robocup_head_detection.class_schema import CLASSES, LEGACY_CLASSES, LEGACY_TO_CURRENT


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n")


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def renamed(name):
    target = LEGACY_TO_CURRENT[LEGACY_CLASSES.index(name)]
    return None if target is None else CLASSES[target]


def package_dataset(target, archive):
    """Reproducible upload ZIP; exclude test videos and package self-checksum."""
    archive.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as package:
        for path in sorted(target.rglob("*")):
            relative = path.relative_to(target)
            if (path.is_file() and relative.parts[0] != "test_videos" and
                    path.name not in (".gitignore", ".gitattributes", "README.md", "colab_package_manifest.json")):
                entry = zipfile.ZipInfo(relative.as_posix(), date_time=(2026, 10, 12, 0, 0, 0))
                entry.compress_type = zipfile.ZIP_DEFLATED
                package.writestr(entry, path.read_bytes())
    manifest = json.loads((target / "dataset_manifest.json").read_text())
    package_manifest = {"zip": archive.name, "bytes": archive.stat().st_size,
                        "sha256": digest(archive), "classes": list(CLASSES),
                        "train": manifest["train_images"], "val": manifest["val_images"],
                        "boxes": manifest["label_boxes"], "test_included": False}
    write_json(target / "colab_package_manifest.json", package_manifest)
    return package_manifest


def migrate(source, target, archive):
    if tuple((source / "classes.txt").read_text().splitlines()) != LEGACY_CLASSES:
        raise ValueError("Source is not the original seven-class Head dataset")
    if target.exists() or archive.exists():
        raise FileExistsError("Destination dataset and ZIP must be new paths")
    manifest = json.loads((source / "dataset_manifest.json").read_text())
    # Check the input before copying anything; do not migrate modified or corrupt files.
    for name, expected in manifest["files_sha256"].items():
        if digest(source / name) != expected:
            raise ValueError(f"Source checksum mismatch: {name}")
    target.mkdir(parents=True)
    counts = {}
    removed = 0
    for folder in ("images", "test_videos"):
        shutil.copytree(source / folder, target / folder)
    for split in ("train", "val"):
        counts[split] = Counter({name: 0 for name in CLASSES})
        (target / "labels" / split).mkdir(parents=True)
        for label in sorted((source / "labels" / split).glob("*.txt")):
            lines = []
            for line in label.read_text().splitlines():
                old, coordinates = line.split(maxsplit=1)
                values = coordinates.split()
                if len(values) != 4 or any(not 0 <= float(v) <= 1 for v in values):
                    raise ValueError(f"Invalid box: {label}: {line}")
                current = LEGACY_TO_CURRENT[int(old)]
                if current is None:
                    removed += 1
                    continue
                counts[split][CLASSES[current]] += 1
                lines.append(f"{current} {coordinates}")
            (target / "labels" / split / label.name).write_text(
                "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8", newline="\n")
    (target / "classes.txt").write_text("\n".join(CLASSES) + "\n", encoding="utf-8", newline="\n")
    (target / "data.yaml").write_text("train: images/train\nval: images/val\nnames:\n" +
        "".join(f"  {i}: {name}\n" for i, name in enumerate(CLASSES)), encoding="utf-8", newline="\n")
    for name in ("frames.csv", "split_manifest.json", ".gitignore"):
        (target / name).write_text((source / name).read_text(encoding="utf-8-sig"),
                                  encoding="utf-8", newline="\n")
    with (source / "object_table_assignments.csv").open(newline="", encoding="utf-8") as stream:
        rows = csv.DictReader(stream)
        with (target / "object_table_assignments.csv").open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, rows.fieldnames, lineterminator="\n")
            writer.writeheader()
            for row in rows:
                current = LEGACY_TO_CURRENT[int(row["class_id"])]
                if current is not None:
                    row.update(class_id=current, class_name=CLASSES[current])
                    writer.writerow(row)
    inventory = json.loads((source / "video_table_inventory.json").read_text())
    for tables in inventory["videos"].values():
        for table, names in tables.items():
            tables[table] = [renamed(name) for name in names if renamed(name) is not None]
    write_json(target / "video_table_inventory.json", inventory)
    review = json.loads((source / "review_state.json").read_text())
    for frame in review.values():
        boxes = []
        for box in frame["boxes"]:
            current = LEGACY_TO_CURRENT[box["class_id"]]
            if current is not None:
                boxes.append(dict(box, class_id=current))
        frame["boxes"] = boxes
    write_json(target / "review_state.json", review)
    audit = json.loads((source / "audit_report.json").read_text())
    audit["class_instance_counts"] = counts
    audit["total_boxes"] = sum(sum(c.values()) for c in counts.values())
    audit["issues"] = {renamed(key.split(":")[0]) + ":" + key.split(":", 1)[1]: value
                       for key, value in audit["issues"].items() if not key.startswith("plate:")}
    # Flagged-frame union cannot be derived from aggregate issue counts.
    audit["source_flagged_frames"] = audit.pop("flagged_frames")
    audit["flagged_frames"] = None
    audit["flagged_frames_note"] = "Requires a new per-frame review after excluding plate"
    write_json(target / "audit_report.json", audit)
    hashes = {name: digest(target / name) for name in manifest["files_sha256"]}
    manifest.update(label_boxes=audit["total_boxes"], files_sha256=hashes,
                    classes=list(CLASSES), source_dataset=source.name,
                    legacy_id_to_current=LEGACY_TO_CURRENT, removed_plate_boxes=removed,
                    correction="apple was tomato; mug renamed cup; plate excluded; no bbox changes")
    write_json(target / "dataset_manifest.json", manifest)
    # Upload package includes train/val and metadata, never the held-out videos.
    package_manifest = package_dataset(target, archive)
    print(json.dumps(package_manifest, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--archive", type=Path, required=True)
    args = parser.parse_args()
    migrate(args.source, args.target, args.archive)
