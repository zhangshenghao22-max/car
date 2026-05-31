from __future__ import annotations

import time
from pathlib import Path
import json

from openai import OpenAI
from tqdm import tqdm

MODEL_NAME = "deepseek-ai/DeepSeek-V3.2"
OPENAI_API_KEY = "ytl2gA0DpJgveobtA9Ca4aF2-3CeD-4844-8E17-123b80B6"
OPENAI_BASE_URL = "https://api.modelverse.cn/v1/"

PROMPT_TEMPLATE = """下面是用户的查询，请选择正确的函数并生成参数来调用函数，如果小车执行不了查询，只需要输出:小车无法完成任务。

查询:{query}

可调用的函数:
{car_functions}

生成的数据格式如下(只允许输出这一种格式，禁止输出JSON、Markdown代码块或任何多余说明):
<rtt_start>
函数名(参数)
函数名(参数)
<rtt_end>
函数描述:
调用函数的完整描述

注意:
1) 函数调用后，必须以 <rtt_start> 开始，<rtt_end> 结尾。
2) 不要输出 ```json 或其他代码块标记。
3) 不要输出JSON结构，只输出函数调用行与函数描述文本。

示例:
查询: 右转90度，向前移动3米，再右转45度
<rtt_start>
turnRight(90)
moveForward(3)
turnLeft(45)
<rtt_end>
函数描述:

def turnRight(degree):
    控制小车向右转弯指定角度。

    Parameters:
    - degree (int): 小车向右转弯角度，单位为度。

    Returns:
    - bool: 如果转弯成功，则为True，否则为False。

def moveForward(distance):
    控制小车向前移动指定距离。

    Parameters:
    - distance (int): 小车运行距离，单位为米

    Returns:
    - bool: 如果移动成功，则为True，否则为False。
"""


def read_text(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
    return path.read_text(encoding="utf-8")


def load_queries(path: Path) -> list[str]:
    lines = read_text(path).splitlines()
    return [line.strip() for line in lines if line.strip()]


def load_records(path: Path) -> list[dict]:
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError(f"Expected a JSON array in {path}")
    return data


def save_records(records: list[dict], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = output_path.with_suffix(output_path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as file:
        json.dump(records, file, ensure_ascii=False, indent=2)
        file.write("\n")
    tmp_path.replace(output_path)


def build_prompt(car_functions: str, query: str) -> str:
    return PROMPT_TEMPLATE.format(car_functions=car_functions, query=query)


def main() -> None:
    start_time = time.perf_counter()
    base_dir = Path(__file__).resolve().parent

    car_functions = read_text(base_dir / "car_functions.txt")
    queries = load_queries(base_dir / "car_instructions.txt")
    output_path = base_dir / "function_call_dataset.json"
    records = load_records(output_path)

    client = OpenAI(api_key=OPENAI_API_KEY, base_url=OPENAI_BASE_URL)

    for query in tqdm(queries):
        prompt = build_prompt(car_functions, query)
        response = client.chat.completions.create(
            model=MODEL_NAME,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=1024,
            temperature=0.2,
        )
        output = response.choices[0].message.content
        record = {
            "instruction": prompt,
            "input": "",
            "output": output.strip(),
        }
        records.append(record)
        save_records(records, output_path)

    elapsed = time.perf_counter() - start_time
    print(f"生成完成: {len(queries)} 条")
    print(f"保存到: {output_path}")
    print(f"总运行时间: {elapsed:.2f} 秒")


if __name__ == "__main__":
    main()
