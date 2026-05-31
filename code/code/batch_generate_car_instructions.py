from __future__ import annotations

import argparse
import time
from pathlib import Path

from generate_car_instructions import (
    get_default_output_path,
    stream_instructions,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Bulk-generate car instructions and save to a file"
    )
    parser.add_argument(
        "-n",
        "--count",
        type=int,
        default=5,
        help="Number of batches to generate",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output file path (defaults to the project file)",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=0.0,
        help="Seconds to sleep between requests",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.count <= 0:
        raise SystemExit("count must be greater than 0")

    output_path = args.output or get_default_output_path()
    start_time = time.perf_counter()

    for index in range(args.count):
        stream_instructions(output_path, print_stream=True)
        if args.sleep > 0 and index < args.count - 1:
            time.sleep(args.sleep)

    elapsed = time.perf_counter() - start_time
    print(f"\nBatch complete: {args.count} runs")
    print(f"Saved to: {output_path}")
    print(f"Total runtime: {elapsed:.2f} seconds")


if __name__ == "__main__":
    main()
