# 电机、IMU与坐标积分标定清单

新版统一使用：

- `vx`：车体向右，cm/s
- `vy`：车体向前，cm/s
- `w`：逆时针为正，rad/s
- 世界坐标航向角：车头相对世界 `+X` 的逆时针夹角

## 建议标定顺序

### 1. 电机和编码器方向

把车架空，每次只测试一个轮，使用较小占空比。

需要确认：

1. 正轮速目标对应预期的轮子正方向。
2. 电机正转时编码器读数必须为正。
3. 三个轮的编号与接线一致：
   - wheel 1：电机 `D4/D5`，编码器 `D13/D14`
   - wheel 2：电机 `C30/C31`，编码器 `D15/D16`
   - wheel 3：电机 `C28/C29`，编码器 `C2/C3`

调整 `WheelConfig.motor_invert` 和 `WheelConfig.encoder_invert`，不要在上层运动代码里临时改符号。

### 2. 每米编码器脉冲数

分别让三个轮低速、无打滑地滚过已知距离，累计每个10ms采样周期的编码器脉冲。

```text
pulses_per_meter = 累计脉冲绝对值 / 实际距离(m)
```

每个轮正向、反向各测至少3次，取平均值，分别写入三个 `WheelConfig.pulses_per_meter`。旧值 `6000` 只作为占位值，未确认前坐标距离不可信。

### 3. 机械半径和旋转比例

- `MotorConfig.robot_radius_cm`：测量车体旋转中心到轮子驱动力作用线的距离。
- `MotorConfig.rotation_gain`：物理模型应接近 `1.0`。

完成轮速标定后，让车原地旋转若干整圈并比较指令角速度和IMU角速度：

```text
new_rotation_gain =
    old_rotation_gain × commanded_angle / measured_angle
```

每次只修正一半误差并复测，避免把地面打滑全部吸收到该参数中。旧工程的 `8.1` 混合了多种未标定比例，不应直接沿用。

### 4. 前馈和静摩擦补偿

三个轮分别标定，不要共用一组结果。

1. 找到轮子在实际地面和载荷下能够连续转动的最小占空比，作为 `stiction_duty` 初值。
2. 在多个稳定轮速点记录占空比和实际速度。
3. 对正反方向分别拟合：

```text
duty ≈ feedforward × target_speed + stiction_duty × sign(target_speed)
```

如果正反方向差异明显，后续应把配置扩展成正反两套参数。`stiction_full_speed` 决定静摩擦补偿从0渐增到全值的速度范围。

### 5. 速度PI

先完成编码器、前馈和静摩擦标定，再调PI。

1. `ki=0`，逐步增加 `kp`，直到速度响应足够快但没有持续振荡。
2. 逐步增加 `ki`，消除稳态误差。
3. 测试低速、常用速度、高速、正反切换和堵转恢复。
4. 三个轮分别记录上升时间、超调、稳态误差和最大占空比。

新版 `ki` 使用真实秒作为时间基准，不能直接照搬以“每次循环”为单位的旧参数。当前 `kp=3.1、ki=50` 只是按旧2ms算法换算的起点。

### 6. 速度和S曲线限制

需要在满电、低电量以及正常载荷下测试：

- `max_wheel_speed_cm_s`
- `max_xy_speed_cm_s`
- `max_w_rad_s`
- `xy_accel_up_cm_s2`
- `xy_accel_down_cm_s2`
- `xy_jerk_cm_s3`
- `w_accel_up_rad_s2`
- `w_accel_down_rad_s2`
- `w_jerk_rad_s3`

调节顺序：最大速度 → 加速度/减速度 → jerk。测试前进、横移、斜移、原地旋转、平移旋转叠加和正反急切换。正常动作应使用 `soft_stop()`；急停、通信超时和异常使用 `hard_stop()`。

### 7. IMU轴向、零偏和比例

先打印静止原始数据，再做以下动作：

1. 车头抬起：`pitch` 的符号应固定且符合约定。
2. 右侧抬起：`roll` 的符号应固定。
3. 俯视逆时针转车：`heading` 和 `yaw_rate` 必须增大。

若不符合，修改 `OdometryConfig.axis_indices`、`axis_signs`，不要在积分公式中改符号。

原地静置至少30秒，记录：

- 三轴陀螺均值和标准差；
- 加速度模长均值；
- 航向漂移量；
- 冷启动和运行发热后的差别。

随后缓慢旋转准确的360°，计算对应原始轴的比例修正：

```text
new_gyro_scale = old_gyro_scale × 360° / reported_angle
```

当前 `gyro_scale_raw[1]=1.165048536` 来自旧车，只能作为初值。

### 8. 坐标距离比例

轮速闭环和IMU标定完成后再标定坐标。

1. 沿车头方向直行100–200cm：

```text
new_forward_scale =
    old_forward_scale × actual_distance / reported_distance
```

2. 保持航向横移100–200cm：

```text
new_lateral_scale =
    old_lateral_scale × actual_distance / reported_distance
```

写入：

- `OdometryConfig.forward_distance_scale`
- `OdometryConfig.lateral_distance_scale`

正向、反向、左移、右移都要测。如果同一方向往返误差不同，通常是轮子打滑、重心或电机参数问题，不应只靠比例系数掩盖。

### 9. 最终组合验证

至少实测：

- 静置60秒航向漂移；
- 直行2m后位置误差和航向误差；
- 横移1m后位置误差；
- 原地正反各旋转3圈；
- 1m × 1m闭合方形路线的回零误差；
- 平移与旋转同时进行；
- 指令中断超过 `watchdog_timeout_ms` 后PWM是否立即归零；
- 人工抛出异常后PWM是否归零；
- 两车接收同一 `get_limited_command()` 时速度变化是否同步。

## 基本用法

```python
from motor import MotorSystem
from odometry import OdometrySystem

odometry = OdometrySystem()
motor = MotorSystem(odometry=odometry)

try:
    motor.start()                 # 启动时保持车辆静止约2秒
    while True:
        motor.move(0.0, 40.0, 0.0)
        # 主车无线发送 motor.get_limited_command()
finally:
    motor.hard_stop()
```

上层的 `modpack.move_robot()` 可以继续存在，但它最终只应调用一次底层 `motor.move(vx, vy, w)`，不要再重复做轮速转换或S曲线限制。

如果某个控制阶段明确需要跳过S曲线，可以使用：

```python
motor.command(vx, vy, w)
```

`command()` 仍然保留底盘速度限制、轮速限制、PI闭环和150ms指令看门狗，只会跳过S曲线。它会立即改变轮速目标，因此不应用于普通巡航、跟随或停车。之后调用 `move()` 或 `soft_stop()` 时，控制器会从当前执行速度重新进入S曲线；异常和紧急情况仍必须调用 `hard_stop()`。
