# 跟随摄像头可靠双红外点识别 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `car/跟随摄像头/` 新建可抗反光、可滤波、可标定且保持六字段 UART 协议的 OpenART 双红外点识别程序。

**Architecture:** `main.py` 只负责 OpenART 硬件采图、UART 和绘图；`ir_tracker.py` 负责候选配对、连续性和滤波；`config.py` 集中现场参数。纯算法通过桌面 `unittest` 测试，硬件脚本只做语法检查。

**Tech Stack:** OpenART MicroPython、Python 标准库 `math`、桌面 `unittest`

---

## 文件结构

- Create: `car/跟随摄像头/config.py` — 所有现场参数。
- Create: `car/跟随摄像头/ir_tracker.py` — 无硬件依赖的纯算法。
- Create: `car/跟随摄像头/main.py` — OpenART 运行入口。
- Create: `car/跟随摄像头/README.md` — 烧录、验证和标定手册。
- Create: `car/tests/test_follow_camera_tracker.py` — 桌面行为测试。

### Task 1: 候选过滤和双点配对

**Files:**
- Create: `car/tests/test_follow_camera_tracker.py`
- Create: `car/跟随摄像头/config.py`
- Create: `car/跟随摄像头/ir_tracker.py`

- [x] **Step 1: 写候选配对失败测试**

测试使用普通字典候选，覆盖几何目标优先于大反光、距离不合法和大小比例不合法：

```python
def blob(cx, cy, pixels=40, w=8, h=8):
    return {"cx": cx, "cy": cy, "pixels": pixels, "w": w, "h": h}

def test_select_pair_prefers_reference_geometry_over_largest_reflection(self):
    blobs = [blob(140, 100), blob(180, 140), blob(40, 40, 300, 20, 20)]
    pair = tracker.select_best_pair(blobs, make_config())
    self.assertEqual(pair["mid_x"], 160.0)
    self.assertEqual(pair["mid_y"], 120.0)

def test_select_pair_rejects_invalid_distance_and_size_ratio(self):
    self.assertIsNone(tracker.select_best_pair([blob(10, 10), blob(15, 10)], make_config()))
    self.assertIsNone(
        tracker.select_best_pair([blob(140, 100, 80), blob(180, 140, 5)], make_config())
    )
```

- [x] **Step 2: 运行测试确认 RED**

Run:

```powershell
python -B -m unittest car.tests.test_follow_camera_tracker -v
```

Expected: FAIL，因为 `ir_tracker` 和 `select_best_pair` 尚不存在。

- [x] **Step 3: 最小实现配置和配对**

`config.py` 定义 QVGA、ROI、blob、配对、滤波和发送参数。`ir_tracker.py` 提供：

```python
def normalize_line_angle_deg(angle):
    while angle > 90.0:
        angle -= 180.0
    while angle <= -90.0:
        angle += 180.0
    return float(angle)

def is_valid_candidate(candidate, config):
    pixels = float(candidate.get("pixels", 0))
    width = float(candidate.get("w", 0))
    height = float(candidate.get("h", 0))
    if pixels < config["blob_pixels_min"] or width * height < config["blob_area_min"]:
        return False
    if width <= 0 or height <= 0:
        return False
    aspect = width / height
    return config["blob_aspect_min"] <= aspect <= config["blob_aspect_max"]

def select_best_pair(candidates, config, previous=None):
    # 过滤候选，枚举所有二元组合，执行距离/面积比例/历史跳变硬约束，
    # 按参考灯距、参考中点、面积对称性和历史连续性加权评分，
    # 返回最低分组合的测量字典；无合法组合返回 None。
```

返回测量字典固定包含：

```python
{
    "found": True,
    "x1": int, "y1": int, "x2": int, "y2": int,
    "mid_x": float, "mid_y": float,
    "line_angle_deg": float,
    "distance_px": float,
    "quality": int,
}
```

- [x] **Step 4: 运行测试确认 GREEN**

Run: `python -B -m unittest car.tests.test_follow_camera_tracker -v`

Expected: 新增配对测试 PASS。

- [x] **Step 5: 检查点**

当前工作区不是有效 Git 仓库，因此不执行提交；在 `progress.md` 记录 RED/GREEN 结果。

### Task 2: 历史连续性、确认状态和滤波

**Files:**
- Modify: `car/tests/test_follow_camera_tracker.py`
- Modify: `car/跟随摄像头/ir_tracker.py`

- [x] **Step 1: 写状态和滤波失败测试**

```python
def test_tracker_requires_two_frames_and_loses_immediately(self):
    state = tracker.IRTracker(make_config())
    self.assertFalse(state.update(valid_blobs())["found"])
    self.assertTrue(state.update(valid_blobs())["found"])
    self.assertFalse(state.update([])["found"])

def test_tracker_filters_midpoint_and_distance(self):
    state = tracker.IRTracker(make_config(ema_alpha=0.5, acquire_confirm_frames=1))
    first = state.update([blob(140, 100), blob(180, 140)])
    second = state.update([blob(150, 100), blob(190, 140)])
    self.assertAlmostEqual(second["mid_x"], (first["mid_x"] + 170.0) / 2.0)

def test_line_angle_filter_uses_180_degree_period(self):
    filtered = tracker.ema_line_angle_deg(89.0, -89.0, 0.5)
    self.assertLess(abs(abs(filtered) - 90.0), 0.001)
```

- [x] **Step 2: 运行测试确认 RED**

Run: `python -B -m unittest car.tests.test_follow_camera_tracker -v`

Expected: FAIL，因为 `IRTracker` 和 `ema_line_angle_deg` 尚不存在。

- [x] **Step 3: 最小实现跟踪器**

```python
def ema(current, sample, alpha):
    return current + alpha * (sample - current)

def ema_line_angle_deg(current, sample, alpha):
    delta = normalize_line_angle_deg(sample - current)
    return normalize_line_angle_deg(current + alpha * delta)

class IRTracker:
    def __init__(self, config):
        self.config = config
        self.previous = None
        self.filtered = None
        self.confirm_count = 0
        self.confirmed = False

    def update(self, candidates):
        measurement = select_best_pair(candidates, self.config, self.previous)
        if measurement is None:
            self.confirm_count = 0
            self.confirmed = False
            return {"found": False}
        # 更新连续确认计数；确认后对 mid_x/mid_y/distance/angle 做 EMA；
        # 保存原始端点供 main.py 绘图，并返回固定测量字段。
```

- [x] **Step 4: 运行测试确认 GREEN**

Run: `python -B -m unittest car.tests.test_follow_camera_tracker -v`

Expected: 所有跟踪和滤波测试 PASS。

- [x] **Step 5: 检查点**

记录测试数量和结果；因无有效 Git 仓库不提交。

### Task 3: 六字段协议和发送时机

**Files:**
- Modify: `car/tests/test_follow_camera_tracker.py`
- Modify: `car/跟随摄像头/ir_tracker.py`

- [x] **Step 1: 写协议失败测试**

```python
def test_format_measurement_line_preserves_six_fields(self):
    line = tracker.format_measurement_line({
        "found": True, "mid_x": 160.2, "mid_y": 119.8,
        "line_angle_deg": 45.25, "distance_px": 80.04, "quality": 84,
    })
    self.assertEqual(line, "1,160,120,45.25,80.0,84\n")
    self.assertEqual(tracker.format_measurement_line({"found": False}), "0,0,0,0,0,0\n")
```

- [x] **Step 2: 运行测试确认 RED**

Run: `python -B -m unittest car.tests.test_follow_camera_tracker -v`

Expected: FAIL，因为 `format_measurement_line` 尚不存在。

- [x] **Step 3: 实现格式化**

```python
def format_measurement_line(measurement):
    if not measurement or not measurement.get("found", False):
        return "0,0,0,0,0,0\n"
    quality = max(0, min(999, int(measurement.get("quality", 0))))
    return "1,%d,%d,%.2f,%.1f,%d\n" % (
        int(round(measurement["mid_x"])),
        int(round(measurement["mid_y"])),
        measurement["line_angle_deg"],
        measurement["distance_px"],
        quality,
    )
```

- [x] **Step 4: 运行测试确认 GREEN**

Run: `python -B -m unittest car.tests.test_follow_camera_tracker -v`

Expected: 全部 PASS。

### Task 4: OpenART 主循环

**Files:**
- Create: `car/跟随摄像头/main.py`

- [x] **Step 1: 创建硬件入口**

主循环必须：

```python
import sensor, image, time, machine, pyb
import config
import ir_tracker

# 配置灰度 QVGA、镜像/翻转、关闭自动增益、固定曝光。
# find_blobs(..., roi=config.ROI, merge=False)。
# 把 OpenART blob 转成 cx/cy/pixels/w/h 字典。
# 调用 IRTracker.update()。
# found=1 按 TX_MIN_INTERVAL_MS 发送。
# found 从 1 变 0 时立即发送；持续 lost 按 LOST_HEARTBEAT_MS 发送。
# 单帧异常发送 lost，下一帧继续运行。
# DEBUG_DRAW 为 True 时才绘制 ROI、端点、中点、连线和测量值。
```

- [x] **Step 2: 运行语法检查**

Run:

```powershell
python -B -m py_compile "car/跟随摄像头/config.py" "car/跟随摄像头/ir_tracker.py" "car/跟随摄像头/main.py"
```

Expected: exit code 0。桌面不导入 `main.py`，因为没有 OpenART `sensor` 模块。

### Task 5: 参数测量文档

**Files:**
- Create: `car/跟随摄像头/README.md`

- [x] **Step 1: 写部署和标定说明**

README 必须列出并给出测量方法：

1. `H_MIRROR/V_FLIP`：确认画面方向。
2. `EXPOSURE_US`、`IR_THRESHOLD`：在比赛最亮和最暗环境采样。
3. `ROI`：记录红外灯在所有允许队形下的包络范围并加余量。
4. `REF_CX/REF_CY`：理想并排队形下连续 100 帧中点均值。
5. `REF_LINE_ANGLE_DEG`：同一批帧的周期角均值。
6. `REF_DISTANCE_PX`：同一批帧的灯距均值。
7. `PAIR_DISTANCE_MIN/MAX_PX`：最近/最远合法队形测量值加安全余量。
8. `BLOB_PIXELS_MIN/BLOB_AREA_MIN`、宽高比范围和 `PAIR_SIZE_RATIO_MIN`。
9. 中点、距离和角度最大单帧跳变。
10. `EMA_ALPHA`、确认帧数和发送周期。

同时写明先静态、再架空轮、最后低速落地的验证顺序。

### Task 6: 最终验证

**Files:**
- Verify: `car/跟随摄像头/*.py`
- Verify: `car/tests/test_follow_camera_tracker.py`

- [x] **Step 1: 运行目标测试**

Run: `python -B -m unittest car.tests.test_follow_camera_tracker -v`

Expected: 全部 PASS，无 warning。

- [x] **Step 2: 运行现有内层 car 回归测试**

Run: `python -B -m unittest discover -s car/tests -p "test_*.py" -v`

Expected: 新旧测试全部 PASS。

- [x] **Step 3: 运行语法检查**

Run:

```powershell
python -B -m py_compile "car/跟随摄像头/config.py" "car/跟随摄像头/ir_tracker.py" "car/跟随摄像头/main.py"
```

Expected: exit code 0。

- [x] **Step 4: 核对文件与协议**

确认目标目录只包含设计规定的四个文件，串口 found/lost 均为六字段。
