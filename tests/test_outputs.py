"""Acceptance tests for the inverted-pendulum recovery envelope."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest


ENTRY = os.environ.get("PENDULUM_ENTRY", "/app/inverted_pendulum_limits.py")
TRUSTED_PYTHON = Path("/usr/local/bin/python3")
SETPRIV = Path("/usr/bin/setpriv")


def candidate_argv(*arguments):
    values = [str(value) for value in arguments]
    if SETPRIV.is_file() and TRUSTED_PYTHON.is_file():
        return [
            str(SETPRIV),
            "--reuid=65534",
            "--regid=65534",
            "--clear-groups",
            str(TRUSTED_PYTHON),
            "-I",
            "-B",
            *values,
        ]
    return [sys.executable, "-I", "-B", *values]


def expose_to_candidate(path):
    if not SETPRIV.is_file():
        return
    path.chmod(0o777)
    for parent in path.parents:
        if parent == Path("/tmp"):
            break
        parent.chmod(parent.stat().st_mode | 0o001)


def invoke_candidate(*arguments, timeout=60):
    return subprocess.run(
        candidate_argv(*arguments),
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def sample():
    return {
        "controller_id": "RIG-UP-04",
        "mass_kg": 1.0,
        "length_m": 1.0,
        "gravity_m_s2": 9.81,
        "damping_s_inv": 0.2,
        "gains": [25.0, 8.0],
        "torque_limit_nm": 5.0,
        "simulation": {
            "dt_s": 0.005,
            "duration_s": 10.0,
            "initial_angle_rad": 0.2,
            "initial_rate_rad_s": 0.1,
            "recovery_low_rad": 0.01,
            "recovery_high_rad": 1.0,
            "settle_angle_rad": 0.02,
            "settle_rate_rad_s": 0.1,
            "search_tolerance_rad": 1e-5,
        },
    }


def relaxed_sample():
    payload = sample()
    payload["simulation"].update(
        {
            "dt_s": 0.05,
            "duration_s": 0.1,
            "recovery_low_rad": 0.01,
            "recovery_high_rad": 0.5,
            "settle_angle_rad": 10.0,
            "settle_rate_rad_s": 10.0,
        }
    )
    return payload


def write(path, payload, raw=None):
    if raw is None:
        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
    path.write_bytes(raw.encode("utf-8"))
    return path


def invoke(tmp_path, payload, raw=None, name="result"):
    source = write(tmp_path / "controller.json", payload, raw=raw)
    output = tmp_path / f"{name}.json"
    expose_to_candidate(tmp_path)
    result = invoke_candidate(ENTRY, source, output)
    return result, source, output


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_success_is_quiet(tmp_path):
    """题面：「正常完成后，标准输出 stdout 和标准错误 stderr 都不写内容。」"""
    result, _, output = invoke(tmp_path, sample())
    assert result.returncode == 0 and result.stdout == "" and result.stderr == "" and output.is_file()


def test_requested_initial_state_is_settled(tmp_path):
    """题面：「`recovered` 是布尔值，表示指定初始状态是否在终态达到两个 settle 阈值。」"""
    result, _, output = invoke(tmp_path, sample())
    document = read(output)
    assert result.returncode == 0 and document["recovered"] is True
    assert abs(document["final_angle_rad"]) <= 0.02
    assert abs(document["final_rate_rad_s"]) <= 0.1


def test_peak_torque_respects_actuator_limit(tmp_path):
    """题面：「u = min(torque_limit_nm, max(-torque_limit_nm, u_raw))」。"""
    result, _, output = invoke(tmp_path, sample())
    assert result.returncode == 0 and read(output)["peak_torque_nm"] <= 5.0


def test_recovery_angle_uses_nonlinear_simulation(tmp_path):
    """题面：「恢复判定不假设“角度越大越难收住”，恢复区间可能断开，也可能出现多个岛状区间。」"""
    result, _, output = invoke(tmp_path, sample())
    assert result.returncode == 0 and read(output)["recovery_angle_rad"] == pytest.approx(0.5320703125, rel=1e-12)


def test_nonmonotonic_recovery_islands_use_global_maximum(tmp_path):
    """题面：「恢复区间可能断开，也可能出现多个岛状区间」，最终取全扫描最大值。"""
    payload = sample()
    payload.update(
        {
            "mass_kg": 0.841,
            "length_m": 4.02,
            "gravity_m_s2": 7.441,
            "damping_s_inv": 0.555,
            "gains": [51.31, -5.04],
            "torque_limit_nm": 12.637,
        }
    )
    payload["simulation"].update(
        {
            "dt_s": 0.02,
            "duration_s": 15.0,
            "recovery_low_rad": 0.001,
            "recovery_high_rad": 3.14,
            "settle_angle_rad": 0.03,
            "settle_rate_rad_s": 0.2,
        }
    )
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 0
    assert read(output)["recovery_angle_rad"] == pytest.approx(0.50373046875, rel=1e-12)


def test_top_level_keys_are_exact(tmp_path):
    """题面：「台架参数存成 UTF-8 JSON 对象，字段集合如下：」。"""
    payload = sample()
    payload["operator"] = "night"
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_simulation_keys_are_exact(tmp_path):
    """题面：「`simulation` 的参数由台架标定软件填写：」。"""
    payload = sample()
    payload["simulation"]["filter"] = "lowpass"
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


@pytest.mark.parametrize("identifier", ["A", "A" * 49])
def test_controller_identifier_length_is_strict(tmp_path, identifier):
    """题面：「长度必须在 2–48 之间。」"""
    payload = sample()
    payload["controller_id"] = identifier
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


@pytest.mark.parametrize("identifier", ["R2", "R" * 48])
def test_controller_identifier_length_endpoints_are_accepted(tmp_path, identifier):
    """题面：「长度必须在 2–48 之间。」"""
    payload = relaxed_sample()
    payload["controller_id"] = identifier
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 0 and read(output)["controller_id"] == identifier


@pytest.mark.parametrize("identifier", [" leading", "trailing "])
def test_controller_identifier_edges_have_no_whitespace(tmp_path, identifier):
    """题面：「`controller_id` 两端不能有空白，字符串内部也不能出现控制字符。」"""
    payload = sample()
    payload["controller_id"] = identifier
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_controller_identifier_has_no_control_character(tmp_path):
    """题面：「`controller_id` 两端不能有空白，字符串内部也不能出现控制字符。」"""
    payload = sample()
    payload["controller_id"] = "RIG\n04"
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_duplicate_json_key_is_bad_input(tmp_path):
    """题面：「若参数对象不合规，退出码取 1；消息固定为 `运行拒收: BAD_INPUT`。」"""
    raw = json.dumps(sample(), separators=(",", ":"))
    raw = raw[:-1] + ',"gains":[25,8]}'
    result, _, output = invoke(tmp_path, None, raw=raw)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_boolean_mass_is_bad_input(tmp_path):
    """题面：「布尔值不能当数字。」"""
    payload = sample()
    payload["mass_kg"] = True
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_gain_vector_needs_two_numbers(tmp_path):
    """题面：「`gains` 恰好两个有限数，依次为角度增益和角速度增益。」"""
    payload = sample()
    payload["gains"] = [25.0]
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_dt_upper_bound_is_rejected(tmp_path):
    """题面：「`dt_s` 在 `[0.0001,0.05]`」。"""
    payload = sample()
    payload["simulation"]["dt_s"] = 0.06
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_dt_upper_bound_is_accepted(tmp_path):
    """题面：「`dt_s` 在 `[0.0001,0.05]`」。"""
    payload = sample()
    payload["simulation"]["dt_s"] = 0.05
    payload["simulation"]["duration_s"] = 0.1
    payload["simulation"]["settle_angle_rad"] = 10.0
    payload["simulation"]["settle_rate_rad_s"] = 10.0
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 0 and read(output)["status"] == "evaluated"


def test_dt_lower_bound_is_accepted(tmp_path):
    """题面：「`dt_s` 在 `[0.0001,0.05]`」。"""
    payload = relaxed_sample()
    payload["simulation"]["dt_s"] = 0.0001
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 0 and read(output)["status"] == "evaluated"


def test_recovery_bounds_must_be_ordered(tmp_path):
    """题面：「`recovery_low_rad` 小于 `recovery_high_rad`」。"""
    payload = sample()
    payload["simulation"]["recovery_low_rad"] = 1.0
    payload["simulation"]["recovery_high_rad"] = 0.5
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_recovery_bounds_must_not_be_equal(tmp_path):
    """题面：「`recovery_low_rad` 小于 `recovery_high_rad`」。"""
    payload = relaxed_sample()
    payload["simulation"]["recovery_low_rad"] = 0.25
    payload["simulation"]["recovery_high_rad"] = 0.25
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_search_tolerance_upper_bound_is_rejected(tmp_path):
    # CASE CTRL-16-TOL CTRL-22-SCAN
    """题面：「`search_tolerance_rad` 不大于 `1e-4`」。"""
    payload = sample()
    payload["simulation"]["search_tolerance_rad"] = 1e-3
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: BAD_INPUT\n" and not output.exists()


def test_no_recovery_at_low_end_is_refused(tmp_path):
    """题面：「若恢复区间下端无法收住，则终止并返回 `运行拒收: NO_RECOVERY`（码 1）。」"""
    payload = sample()
    payload["simulation"]["recovery_low_rad"] = 2.0
    payload["simulation"]["recovery_high_rad"] = 2.5
    result, _, output = invoke(tmp_path, payload)
    assert result.returncode == 1 and result.stderr == "运行拒收: NO_RECOVERY\n" and not output.exists()


def test_result_has_fixed_key_order(tmp_path):
    """题面：「评估文件写成一行紧凑 JSON，最后只保留一个换行符。」"""
    result, _, output = invoke(tmp_path, sample())
    raw = output.read_bytes()
    document = json.loads(raw)
    assert result.returncode == 0
    assert list(document) == [
        "controller_id",
        "status",
        "final_angle_rad",
        "final_rate_rad_s",
        "peak_torque_nm",
        "recovery_angle_rad",
        "recovered",
    ]
    assert b": " not in raw and raw.endswith(b"\n") and raw.count(b"\n") == 1


def test_absent_parent_is_io_failure(tmp_path):
    """题面：「目标无法创建时，保留上一版文件并返回 `运行拒收: IO_FAILURE`（码 1）。」"""
    source = write(tmp_path / "controller.json", sample())
    output = tmp_path / "absent-parent" / "result.json"
    expose_to_candidate(tmp_path)
    result = invoke_candidate(ENTRY, source, output)
    assert result.returncode == 1 and result.stderr == "运行拒收: IO_FAILURE\n" and not output.exists()


def test_bad_input_preserves_existing_result(tmp_path):
    """题面：「任何拒收都不得泄漏 Python 调用栈。」"""
    result, source, output = invoke(tmp_path, sample())
    before = output.read_bytes()
    payload = sample()
    payload["gains"] = [25.0, True]
    write(source, payload)
    rejected = invoke_candidate(ENTRY, source, output)
    assert result.returncode == 0 and rejected.returncode == 1
    assert output.read_bytes() == before and rejected.stderr == "运行拒收: BAD_INPUT\n"


def test_repeat_run_is_byte_identical(tmp_path):
    """题面：「相同台架参数再次核算时，输出文件字节不得变化。」"""
    result, source, output = invoke(tmp_path, sample())
    first = output.read_bytes()
    second = invoke_candidate(ENTRY, source, output)
    assert result.returncode == 0 and second.returncode == 0 and output.read_bytes() == first
