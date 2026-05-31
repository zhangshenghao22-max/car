from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

START_MARKER = "<rtt_start>"
END_MARKER = "<rtt_end>"

REPLACEMENTS = {
    "moveForward": "<rtt_0>",
    "moveBackward": "<rtt_1>",
    "turnLeft": "<rtt_2>",
    "turnRight": "<rtt_3>",
}


def replace_functions_in_segment(segment: str) -> str:
    for name, token in REPLACEMENTS.items():
        segment = re.sub(rf"\b{name}\b\s*\(", f"{token}(", segment)
    return segment


def replace_between_markers(text: str) -> tuple[str, bool]:
    cursor = 0
    changed = False
    parts: list[str] = []

    while True:
        start_idx = text.find(START_MARKER, cursor)
        if start_idx == -1:
            parts.append(text[cursor:])
            break
        end_idx = text.find(END_MARKER, start_idx + len(START_MARKER))
        if end_idx == -1:
            parts.append(text[cursor:])
            break

        parts.append(text[cursor : start_idx + len(START_MARKER)])
        between = text[start_idx + len(START_MARKER) : end_idx]
        new_between = replace_functions_in_segment(between)
        if new_between != between:
            changed = True
        parts.append(new_between)
        parts.append(text[end_idx : end_idx + len(END_MARKER)])
        cursor = end_idx + len(END_MARKER)

    return "".join(parts), changed


def load_records(path: Path) -> list[dict]:
    if not path.exists():
        raise FileNotFoundError(f"Input file not found: {path}")

    if path.suffix == ".jsonl":
        records: list[dict] = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                records.append(json.loads(line))
        return records

    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError(f"Expected a JSON array in {path}")
    return data


def save_records(records: list[dict], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = output_path.with_suffix(output_path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as file:
        if output_path.suffix == ".jsonl":
            for record in records:
                file.write(json.dumps(record, ensure_ascii=False))
                file.write("\n")
        else:
            json.dump(records, file, ensure_ascii=False, indent=2)
            file.write("\n")
    tmp_path.replace(output_path)


def parse_args() -> argparse.Namespace:
    base_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Replace function names with <rtt_*> tokens between markers."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=base_dir / "function_call_dataset.json",
        help="Input JSON or JSONL file",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output file (defaults to in-place)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = args.input
    output_path = args.output or input_path

    records = load_records(input_path)
    changed_count = 0

    for record in records:
        if not isinstance(record, dict):
            continue
        output = record.get("output")
        if not isinstance(output, str):
            continue
        new_output, changed = replace_between_markers(output)
        if changed:
            record["output"] = new_output
            changed_count += 1

    save_records(records, output_path)
    print(f"Updated records: {changed_count}")
    print(f"Saved to: {output_path}")


if __name__ == "__main__":
    main()
