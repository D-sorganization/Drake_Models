import os
import re


def fix_file(filepath):
    with open(filepath) as f:
        lines = f.readlines()

    out_lines = []
    for i, line in enumerate(lines):
        if line.lstrip().startswith("def test_") and "->" not in line:
            if "self" in line and len(line.split(",")) == 1:
                # def test_something(self):
                line = line.replace("):", ") -> None:")
            elif "self" not in line and line.rstrip().endswith("():"):
                line = line.replace("():", "() -> None:")
            else:
                # Add Any to args
                match = re.search(r"def test_\w+\((.*)\):", line)
                if match:
                    args_str = match.group(1)
                    new_args_str = ", ".join(
                        [
                            arg if ":" in arg or arg == "self" else f"{arg}: Any"
                            for arg in args_str.split(", ")
                        ]
                    )
                    line = line.replace(f"({args_str}):", f"({new_args_str}) -> None:")

        # fix benchmark
        if "benchmark: object" in line:
            line = line.replace("benchmark: object", "benchmark: Any")
        elif "builder: object" in line:
            line = line.replace("builder: object", "builder: Any")

        if "import pytest" in line and i < 15:
            if "from typing import Any" not in "".join(lines[:20]):
                line = line + "from typing import Any\n"
        out_lines.append(line)

    with open(filepath, "w") as f:
        f.writelines(out_lines)


for root, _, files in os.walk("tests"):
    for f in files:
        if f.endswith(".py"):
            fix_file(os.path.join(root, f))
