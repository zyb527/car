"""calibration_data.txt 的电脑端汇总工具。

用法：
    python calibration/analyze_calibration.py calibration_data.txt
"""

import json
import os
import sys


SUMMARY_TYPES = (
    "pulses_summary",
    "open_loop_summary",
    "pi_stage_summary",
    "motion_stage_summary",
    "odometry_summary",
    "sync_summary",
)


def load_records(path):
    records = []
    damaged_lines = 0
    with open(path, "r", encoding="utf-8") as source:
        for line in source:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                damaged_lines += 1
    return records, damaged_lines


def latest(records, record_type):
    matches = [record for record in records if record.get("type") == record_type]
    return matches[-1] if matches else None


def latest_trial_by_mode(records, record_type):
    result = {}
    for record in records:
        if record.get("type") == record_type and record.get("mode"):
            result[record["mode"]] = record
    return result


def safe_scale(current, actual, reported):
    if actual is None or reported is None or abs(reported) < 1.0e-9:
        return None
    return current * actual / reported


def build_suggestions(records, physical=None):
    suggestions = {}
    pulse_trials = latest_trial_by_mode(records, "pulses_trial")
    if pulse_trials:
        wheel_values = [[], [], []]
        for trial in pulse_trials.values():
            estimates = trial.get("pulses_per_meter_estimates", [])
            for index in range(min(3, len(estimates))):
                if estimates[index] is not None:
                    wheel_values[index].append(estimates[index])
        suggestions["pulses_per_meter"] = [
            sum(values) / len(values) if values else None
            for values in wheel_values
        ]
    else:
        pulses = latest(records, "pulses_summary")
        if pulses:
            suggestions["pulses_per_meter"] = pulses.get(
                "suggested_pulses_per_meter"
            )

    open_loop = latest(records, "open_loop_summary")
    if open_loop:
        suggestions["stiction_duty"] = open_loop.get(
            "suggested_stiction_duty"
        )
        suggestions["feedforward"] = open_loop.get("suggested_feedforward")
        suggestions["observed_max_wheel_speed_cm_s"] = open_loop.get(
            "max_measured_wheel_speed_cm_s"
        )

    odometry = latest(records, "odometry_summary")
    if odometry:
        for key in (
            "suggested_forward_distance_scale",
            "suggested_lateral_distance_scale",
            "suggested_gyro_scale_raw",
            "suggested_rotation_gain",
        ):
            suggestions[key] = odometry.get(key)

    physical = physical or {}
    trials = latest_trial_by_mode(records, "odometry_trial")
    forward = trials.get("forward")
    if forward:
        suggestions["suggested_forward_distance_scale"] = safe_scale(
            forward.get("current_forward_distance_scale", 1.0),
            physical.get("forward_actual_cm"),
            forward.get("reported_x_cm"),
        )
    right = trials.get("right")
    if right:
        suggestions["suggested_lateral_distance_scale"] = safe_scale(
            right.get("current_lateral_distance_scale", 1.0),
            physical.get("right_actual_cm"),
            -right.get("reported_y_cm", 0.0),
        )
    rotation = trials.get("rotate")
    if rotation:
        actual_angle_deg = physical.get("rotate_actual_angle_deg")
        actual_angle_rad = (
            actual_angle_deg * 3.141592653589793 / 180.0
            if actual_angle_deg is not None
            else None
        )
        reported_angle = rotation.get("reported_heading_unwrapped_rad")
        current_scales = list(rotation.get("current_gyro_scale_raw", ()))
        yaw_axis = int(rotation.get("yaw_raw_axis_index", 1))
        gyro_multiplier = safe_scale(1.0, actual_angle_rad, reported_angle)
        if (
            gyro_multiplier is not None
            and 0 <= yaw_axis < len(current_scales)
        ):
            current_scales[yaw_axis] *= gyro_multiplier
            suggestions["suggested_gyro_scale_raw"] = current_scales
        suggestions["suggested_rotation_gain"] = safe_scale(
            rotation.get("current_rotation_gain", 1.0),
            rotation.get("integrated_command_angle_rad"),
            actual_angle_rad,
        )

    suggestions["pi_stage_results"] = [
        record for record in records if record.get("type") == "pi_stage_summary"
    ]
    suggestions["motion_stage_results"] = [
        record
        for record in records
        if record.get("type") == "motion_stage_summary"
    ]
    suggestions["sync_results"] = [
        record for record in records if record.get("type") == "sync_summary"
    ]
    return suggestions


def main():
    default_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        "calibration_data.txt",
    )
    path = sys.argv[1] if len(sys.argv) > 1 else default_path
    records, damaged_lines = load_records(path)
    physical_path = (
        sys.argv[2]
        if len(sys.argv) > 2
        else os.path.join(
            os.path.dirname(os.path.abspath(path)),
            "calibration_physical_measurements.json",
        )
    )
    physical = {}
    if os.path.exists(physical_path):
        with open(physical_path, "r", encoding="utf-8") as source:
            physical = json.load(source)
    suggestions = build_suggestions(records, physical)

    output_path = os.path.join(
        os.path.dirname(os.path.abspath(path)),
        "calibration_suggestions.json",
    )
    with open(output_path, "w", encoding="utf-8") as output:
        json.dump(suggestions, output, ensure_ascii=False, indent=2)

    print("records:", len(records))
    print("ignored incomplete lines:", damaged_lines)
    print("physical measurements:", physical_path)
    for record_type in SUMMARY_TYPES:
        count = sum(
            1 for record in records if record.get("type") == record_type
        )
        print("{}: {}".format(record_type, count))
    print("suggestions written to:", output_path)


if __name__ == "__main__":
    main()
