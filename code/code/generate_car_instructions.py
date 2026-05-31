from __future__ import annotations

import os
import time
from pathlib import Path
from typing import TextIO

from openai import OpenAI

MODEL_NAME = "deepseek-ai/DeepSeek-V3.2"
OPENAI_API_KEY = "ytl2gA0DpJgveobtA9Ca4aF2-3CeD-4844-8E17-123b80B6"
OPENAI_BASE_URL = "https://api.modelverse.cn/v1/"


def load_car_functions() -> str:
    functions_path = Path.cwd() / "car_functions.txt"
    if not functions_path.exists():
        raise FileNotFoundError(f"car_functions.txt not found in {Path.cwd()}")
    return functions_path.read_text(encoding="utf-8")


def build_prompt(car_functions: str) -> str:
    return f"""你是一个人工智能助手，负责生成与ai小车相关的指令。请严格遵循以下要求：
1. 解析下面的 car functions。
2. 根据函数生成50个唯一指令。
3. 其中5个为简单查询（单步动作）；45个为超长指令（包含4-10个操作步骤）。
4. 避免在输出中使用任何XML标记或类似标签（如 <...>）。
5. 确保每条指令都能仅通过 car functions 中的能力实现（只包含向前/向后移动与左/右转，带距离或角度）。
6. 只输出指令本身，不要输出如何执行，也不要出现函数名（如 moveForward）。

输出格式要求：
- 每行一条指令，行首使用“-”。
- 每条指令为单行输出，不要换行。
- 不要添加标题、编号或额外说明。

car functions:
{car_functions}
"""


def get_default_output_path() -> Path:
    return Path.cwd() / "car_instructions.txt"


def generate_instructions() -> str:
    car_functions = load_car_functions()
    prompt = build_prompt(car_functions)

    client = OpenAI(api_key=OPENAI_API_KEY, base_url=OPENAI_BASE_URL)
    response = client.chat.completions.create(
        model=MODEL_NAME,
        messages=[{"role": "user", "content": prompt}],
        max_tokens=8192,
        temperature=0.9,
    )

    return response.choices[0].message.content


def append_instructions(instructions: str, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    text = instructions.strip()
    if not text:
        return
    needs_newline = False
    if output_path.exists() and output_path.stat().st_size > 0:
        with output_path.open("rb") as file:
            file.seek(-1, os.SEEK_END)
            needs_newline = file.read(1) != b"\n"
    with output_path.open("a", encoding="utf-8") as file:
        if needs_newline:
            file.write("\n")
        file.write(text)
        file.write("\n")


def open_output_stream(output_path: Path) -> TextIO:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    needs_newline = False
    if output_path.exists() and output_path.stat().st_size > 0:
        with output_path.open("rb") as file:
            file.seek(-1, os.SEEK_END)
            needs_newline = file.read(1) != b"\n"
    file = output_path.open("a", encoding="utf-8")
    if needs_newline:
        file.write("\n")
        file.flush()
    return file


def stream_instructions(output_path: Path, print_stream: bool = True) -> str:
    car_functions = load_car_functions()
    prompt = build_prompt(car_functions)

    client = OpenAI(api_key=OPENAI_API_KEY, base_url=OPENAI_BASE_URL)
    stream = client.chat.completions.create(
        model=MODEL_NAME,
        messages=[{"role": "user", "content": prompt}],
        max_tokens=8192,
        temperature=0.9,
        stream=True,
    )

    chunks: list[str] = []
    last_char = ""
    with open_output_stream(output_path) as file:
        for chunk in stream:
            delta = chunk.choices[0].delta
            content = delta.content if delta else None
            if not content:
                continue
            chunks.append(content)
            file.write(content)
            file.flush()
            if print_stream:
                print(content, end="", flush=True)
            last_char = content[-1]
        if last_char != "\n":
            file.write("\n")
            file.flush()
            if print_stream:
                print()

    return "".join(chunks).strip()


def main() -> None:
    start_time = time.perf_counter()
    output_path = get_default_output_path()
    stream_instructions(output_path, print_stream=True)
    elapsed = time.perf_counter() - start_time
    print(f"\n运行时间: {elapsed:.2f} 秒")


if __name__ == "__main__":
    main()
