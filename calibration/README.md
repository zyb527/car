# 上电自动标定

主板不需要额外按键。脚本运行流程：

1. 上电后PWM保持为零；
2. 完成IMU初始化；
3. C4灯闪烁倒计时5–7秒；
4. 自动执行固定时长测试；
5. 自动停车并将数据写入 `calibration_data.txt`。

运动过程中只能通过关闭电源紧急中止，因此必须先架空确认方向，再在空旷地面使用低速参数。

## 数据文件

所有测试追加写入：

```text
calibration_data.txt
```

日志采用分批写入。突然断电可能损失最后一小批采样，之前的完整记录仍可读取。

## 测试顺序

### 1. `calibrate_pulses_per_meter.py`

电机不输出，倒计时结束后进入15秒记录窗口。在窗口内手动把车移动准确距离，然后保持不动直到记录结束。

脚本顶部设置：

```python
MOVEMENT_MODE = "forward"
```

先运行一次前进测试，再改为：

```python
MOVEMENT_MODE = "right"
```

重新上电运行横移测试。两次结果会由电脑分析器合并。

### 2. `calibrate_open_loop.py`

车辆自动原地旋转并扫描PWM，用于测量：

- 正反方向启动死区；
- 每个轮的前馈；
- 当前测试占空比范围内的最高轮速。

默认占空比较低。确认安全后才能增加 `DUTY_LEVELS`。

### 3. `calibrate_speed_pi.py`

使用 `command()` 自动完成前进、后退、横移和旋转阶跃，记录：

- 90%上升时间；
- 超调；
- 稳态误差。

运行前应先把脉冲数、死区和前馈写回 `MotorConfig`。

### 4. `calibrate_motion_profile.py`

使用正常 `move()` 测试S曲线，记录速度、加速度、减速度、jerk和编码器估算刹车距离。

默认依次测试前移、右移、原地逆时针旋转、前移同时逆时针旋转。前移/右移/组合的平移终点均为 `150 cm/s`；原地旋转和组合的终点角速度均为 `2 rad/s`。每档从静止开始，保持 4.5 秒、软刹车后再等待 1.5 秒。

平移维度分别扫描加速度 `720 → 960 → 1200 → 1440 cm/s²`（jerk 固定 `12000 cm/s³`），以及 jerk `9000 → 12000 → 15000 → 18000 cm/s³`（加速度固定 `960 cm/s²`）。旋转维度分别扫描角加速度 `12 → 17 → 22 → 28 rad/s²`（角 jerk 固定 `260 rad/s³`），以及角 jerk `160 → 260 → 360 → 480 rad/s³`（角加速度固定 `17 rad/s²`）。组合运动会把平移和旋转两个维度各扫描一次。每档日志记录实际使用的四个参数：`configured_xy_accel_up_cm_s2`、`configured_xy_jerk_cm_s3`、`configured_w_accel_up_rad_s2` 和 `configured_w_jerk_rad_s3`。

采样周期为 50 ms，只保存目标车体速度、S 曲线限幅后的车体速度和三轮轮速；不保存 PWM、编码器总计与位姿，以减少 `calibration_data.txt` 的占用。

### 5. `calibrate_body_speed_limit.py`

连续前进提速测试。速度以脚本顶部 `RAMP_RATE_CM_S2` 从零连续增加，最高请求由 `MAX_REQUEST_SPEED_CM_S` 决定；测试过程不按姿态、航向或漂移自动停止，只记录实际前进速度和姿态数据。达到最高请求并保持 `MAX_SPEED_HOLD_MS` 后软刹车。

实际刹车距离和打滑仍需用卷尺或地面标记测量。

### 5. `calibrate_odometry_scale.py`

一次开机只测试一个模式：

```python
TEST_MODE = "forward"  # 或 "right"、"rotate"
```

分别运行三次。每次自动运动结束后，测量车辆真实前进距离、右移距离或旋转角度，填写：

```text
calibration_physical_measurements.json
```

电脑分析器据此计算前进比例、横移比例、陀螺仪比例和 `rotation_gain`。

### 6. 双车同步

无线模块完成后，在每个有效控制帧处调用：

```python
recorder.record_received_frame(sequence, peer_command, motor, peer_mode)
```

`sync_recorder.py` 会记录丢帧、周期、指令误差和本车轮速。

### 7. 50 cm/s 迁移比例实测

运行：

```text
python calibration/test_forward_speed_50cms.py
```

脚本完全脱机运行，依次以前进和右移方向调用 `MotorSystem.command()`；每段
先预热 1 秒，随后 C4 灯亮起并保持 50 cm/s 整整 1 秒。只测量指示灯亮起的
一秒区间位移，不测预热或停车段。对每个方向分别按下式给出
`LINEAR_SPEED_SCALE` 的建议值：

```text
new_scale = old_scale * 50 / actual_speed_cm_s
```

若前进和右移建议值相差很大，不应粗暴取一个统一比例，而应继续检查三轮
运动学、轮速闭环和方向独立标定。由于 `LINEAR_SPEED_SCALE` 是
`car141929` 参数迁移比例，不建议在没有实测位移前直接修改它。

## 电脑分析

复制车载日志后运行：

```text
python calibration/analyze_calibration.py calibration_data.txt
```

输出为 `calibration_suggestions.json`，不会自动修改车辆参数。
