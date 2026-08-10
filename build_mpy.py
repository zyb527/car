"""Build the deployable main-car MicroPython artifacts.

Run from this directory:
    python build_mpy.py

Use --mpy-cross to specify a compiler path when it is not available on PATH.
"""

from __future__ import print_function

import argparse
import os
import shutil
import subprocess
import sys


SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
DIST_DIR = os.path.join(SOURCE_DIR, "mpy_dist")

# Keep this list aligned with the files copied to the main-car controller.
# motor1.py is a development/reference module and is intentionally excluded.
MODULES = (
    "approach.py",
    "control.py",
    "garage.py",
    "main.py",
    "main_config.py",
    "motor.py",
    "navigation.py",
    "odometry.py",
    "orbit.py",
    "push.py",
    "tof.py",
    "vision.py",
    "wireless_feedforward.py",
)


def find_mpy_cross(configured_path):
    """Return a usable mpy-cross executable, or raise FileNotFoundError."""
    candidates = [configured_path, os.environ.get("MPY_CROSS"), "mpy-cross"]
    if os.name == "nt":
        candidates.extend((r"D:\\Scripts\\mpy-cross.exe", "mpy-cross.exe"))

    for candidate in candidates:
        if not candidate:
            continue
        if os.path.isfile(candidate) or shutil.which(candidate):
            return candidate
    raise FileNotFoundError("未找到 mpy-cross；请安装它或通过 --mpy-cross 指定路径。")


def build(mpy_cross):
    os.makedirs(DIST_DIR, exist_ok=True)

    for module in MODULES:
        source = os.path.join(SOURCE_DIR, module)
        output = os.path.join(DIST_DIR, os.path.splitext(module)[0] + ".mpy")
        print("编译 {} -> {}".format(module, os.path.basename(output)))
        subprocess.run([mpy_cross, "-o", output, source], check=True)

    # main.py is retained as the boot entry point; all imported modules use .mpy.
    shutil.copy2(os.path.join(SOURCE_DIR, "main.py"), os.path.join(DIST_DIR, "main.py"))
    print("已同步 mpy_dist/main.py")


def main():
    parser = argparse.ArgumentParser(description="编译主车部署用的 MPY 文件")
    parser.add_argument("--mpy-cross", help="mpy-cross 可执行文件路径")
    args = parser.parse_args()

    try:
        build(find_mpy_cross(args.mpy_cross))
    except (FileNotFoundError, subprocess.CalledProcessError) as error:
        print("编译失败：{}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
