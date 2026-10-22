# 倒立摆恢复角包络核算

实验台要做控制器换型前的安全评估。旧评估与本台电机的实际限矩不一致，请让 `/app/inverted_pendulum_limits.py` 按当前参数重新给出恢复包络。

## 台架参数

台架参数存成 UTF-8 JSON 对象，字段集合如下：

```text
controller_id
mass_kg
length_m
gravity_m_s2
damping_s_inv
gains
torque_limit_nm
simulation
```

`controller_id` 填台架工位号。长度必须在 2–48 之间。`controller_id` 两端不能有空白，字符串内部也不能出现控制字符。`mass_kg` 取 0.1 到 100，`length_m` 取 0.1 到 10，`gravity_m_s2` 取 1 到 30，`damping_s_inv` 取 0 到 20，`torque_limit_nm` 取 0.001 到 10000，全部为有限数。`gains` 恰好两个有限数，依次为角度增益和角速度增益。

`simulation` 的参数由台架标定软件填写：

```text
dt_s
duration_s
initial_angle_rad
initial_rate_rad_s
recovery_low_rad
recovery_high_rad
settle_angle_rad
settle_rate_rad_s
search_tolerance_rad
```

`dt_s` 在 `[0.0001,0.05]`，`duration_s` 在 `[0.1,60]`；角度及速率参数的绝对值不超过 π；`recovery_low_rad` 小于 `recovery_high_rad`；两个 settle 阈值和搜索容差均为正；`search_tolerance_rad` 不大于 `1e-4`。

布尔值不能当数字。台架软件会传入输入文件与结果文件。

```bash
python3 /app/inverted_pendulum_limits.py <台架参数> <评估结果>
```

## 非线性模型

角度从竖直向上位置量起。记角度为 θ、角速度为 ω，控制力矩先按

```text
u_raw = -(k_angle*theta + k_rate*rate)
u = min(torque_limit_nm, max(-torque_limit_nm, u_raw))
```

限幅。然后积分

```text
theta' = rate
rate' = gravity_m_s2/length_m*sin(theta) - damping_s_inv*rate + u/(mass_kg*length_m^2)
```

每轮使用经典四阶 Runge–Kutta，固定步长 `dt_s`。最后一步如果不足 `dt_s`，按剩余时长积分，不能跳过尾段。给定初始角和初始角速度时，积分整段并给出终态和最大绝对控制力矩。

恢复判定不假设“角度越大越难收住”，恢复区间可能断开，也可能出现多个岛状区间。
用同一模型把 `recovery_low_rad` 到 `recovery_high_rad` 等分成 `256` 段，共检查
`257` 个含两端点的候选角，初始角速度均取 0。若某个候选的终态同时满足
`|theta| <= settle_angle_rad` 且 `|rate| <= settle_rate_rad_s`，该候选可恢复。
最终返回全部可恢复候选中的最大角；若没有任何候选可恢复，按包络不存在处理。
`search_tolerance_rad` 仍是台架声明的搜索精度字段，但这次必须遍历整个区间，不能据此把检查退化为单调二分。

## 评估结果

评估文件写成一行紧凑 JSON，最后只保留一个换行符。键顺序为：

```text
controller_id
status
final_angle_rad
final_rate_rad_s
peak_torque_nm
recovery_angle_rad
recovered
```

`status` 写 `evaluated`；`recovered` 是布尔值，表示指定初始状态是否在终态达到两个 settle 阈值。正常完成后，标准输出 stdout 和标准错误 stderr 都不写内容。相同台架参数再次核算时，输出文件字节不得变化。评估文件先在目标目录生成暂存副本，再一次性换到正式名字；换名失败则保留上一版，并清走暂存副本。

## 异常处理

若参数对象不合规，退出码取 1；消息固定为 `运行拒收: BAD_INPUT`。

若恢复区间下端无法收住，则终止并返回 `运行拒收: NO_RECOVERY`（码 1）。

目标无法创建时，保留上一版文件并返回 `运行拒收: IO_FAILURE`（码 1）。

任何拒收都不得泄漏 Python 调用栈。
