# 毕业论文实验流程 — 三组对比测试详细操作手册

> **目标**：在同一场景下完成 Group A / B / C 各 5 次试验，采集轨迹数据、Gemini 日志、图像响应。  
> **预计耗时**：约 3-4 小时（含摆放障碍物、复位、等 API 响应等待时间）  
> **最后更新**：2026-04-24

---

## 0. 实验前准备

### 0.1 场景布置
1. 将小车放在实验室两排工作站之间，朝向走廊（面对出口门方向）
2. 地面放置金属排插底座（可通行视觉干扰物）
3. 走廊中间放置纸箱（真正障碍物）
4. 用胶带在地面标记小车的 **精确起始位置和朝向**（每次试验复位用）
5. 拍一张场景全景照片，用于报告 Figure 4.1

### 0.2 标记起始位姿
```bash
# 启动底盘和相机后，记录起始 odom 坐标（用于验证复位精度）
ros2 topic echo /odom --once
```
记下起始位置 (x, y, yaw)，写在纸上。每次复位后用同样命令确认偏差 < 5cm。

### 0.3 编译工作区
```bash
cd /home/kuko/humble_ws
colcon build --symlink-install
source install/setup.bash
```

### 0.4 确认 API 可用
```bash
# 简单测试 Gemini API 连通性
python3 -c "
from google import genai
from dotenv import load_dotenv
import os
load_dotenv('src/image_to_llm/llm_config.env')
client = genai.Client(api_key=os.getenv('GEMINI_API_KEY'))
r = client.models.generate_content(model='gemini-2.5-flash', contents='hello')
print('API OK:', r.text[:50])
"
```

### 0.5 创建数据存放目录
```bash
mkdir -p /home/kuko/humble_ws/data/experiment/{groupA,groupB,groupC}
mkdir -p /home/kuko/humble_ws/data/experiment/images
```

---

## 1. Group A — Naive Prompt（单次模式）

### 1.1 原理
- Skill 文件：`naive.yaml`（最小化提示词，无透视补偿）
- 管线：`image_to_llm_node` → `image_conversion`（深度反投影 + TF2）→ `/path` → `track_path`
- 需要启动 3 个终端

### 1.2 启动步骤

**终端 1：启动底盘 + 相机驱动**（根据你的实际驱动命令）
```bash
# 启动小车底盘
ros2 launch （你的底盘 launch 命令）

# 启动 Orbbec Gemini 336 相机
ros2 launch orbbec_camera gemini_336_launch.py
```

**终端 2：启动 LLM 节点（使用 naive.yaml）**
```bash
cd /home/kuko/humble_ws
source install/setup.bash
ros2 launch image_to_llm llm_node_launch.py skill_name:=naive
```
等待看到 `✅ 已收到首帧 RGB 图像` 后再继续。

**终端 3：启动下游处理链 + track_path**
```bash
cd /home/kuko/humble_ws
source install/setup.bash
ros2 launch my_robot_planner robot_vlm_launch.py task_id:=0
```
等待看到 `✅ camera_info + 深度图 均已就绪` 后再继续。

**终端 4：启动轨迹记录器**（每次试验都要重新启动）
```bash
cd /home/kuko/humble_ws
source install/setup.bash
python3 data/pose_logger.py
```
> pose_logger 会以 2Hz 记录 odom (x, y, yaw)，Ctrl+C 停止时自动保存到 `data/log/pose_*.log`

### 1.3 执行试验（重复 5 次）

对每次试验 i（i = 1, 2, 3, 4, 5）：

1. **复位小车**到胶带标记位置，确认朝向一致
2. **重启 pose_logger**（终端 4 按 Ctrl+C 再重新运行）
3. **手动触发 Gemini**：
   ```bash
   ros2 service call /trigger_llm_plan std_srvs/srv/Trigger
   ```
4. **开始计时**（从触发命令发出的瞬间开始用手机秒表计时）
5. **观察小车运动**，记录以下信息到纸上：
   - 试验编号：A-i
   - 是否成功到达门口（距门 < 0.15m）：✅ / ❌
   - 是否碰撞障碍物：是 / 否
   - 总时间（秒）：从触发到到达/超时
   - 超时标准：120 秒
   - 需要人工干预则记为失败
6. 小车到达终点（或超时/失败）后：
   - **终端 4**：Ctrl+C 停止 pose_logger → 自动保存轨迹文件
   - 将轨迹文件重命名并移动：
     ```bash
     # pose_logger 保存为 data/log/pose_taskN.log（N 为自动编号）
     mv data/log/pose_task最新编号.log data/experiment/groupA/trial_A${i}_pose.log
     ```
   - 将 image_conversion 的 debug 日志也保存：
     ```bash
     cp data/log/conversion_task0_*.log data/experiment/groupA/
     ```
7. **记录 Gemini 调用次数**：在终端 2 的输出中数一下 `"Gemini 返回成功"` 出现了几次
8. **在终端 3 观察**：如果 track_path 打印了 `"✅ 路径完成！"` 多少次

### 1.4 试验间操作
- 每次试验之间不需要重启节点（除了 pose_logger）
- 把小车搬回起始位置即可
- 如果要重置 conversion 日志编号，可以在 launch 时改 `task_id:=1` 等

### 1.5 完成 5 次后
- 关闭终端 2 和 3 的节点
- 检查 `data/experiment/groupA/` 下有 5 个 pose 文件

---

## 2. Group B — Optimized Prompt（单次模式）

### 2.1 与 Group A 的唯一区别
Skill 文件改为 `default`（默认值，不需要额外指定）

### 2.2 启动步骤

**终端 1**：底盘和相机（如果还在运行就不用重启）

**终端 2：启动 LLM 节点（使用 default.yaml）**
```bash
cd /home/kuko/humble_ws
source install/setup.bash
ros2 launch image_to_llm llm_node_launch.py skill_name:=default
```

**终端 3：启动下游处理链**
```bash
cd /home/kuko/humble_ws
source install/setup.bash
ros2 launch my_robot_planner robot_vlm_launch.py task_id:=10
```
> task_id 设为 10 起步，避免和 Group A 的日志文件冲突

**终端 4**：pose_logger（同 Group A）

### 2.3 执行试验（重复 5 次）
流程与 Group A 完全相同，只是：
- 试验编号改为 B-i
- 轨迹文件移到 `data/experiment/groupB/`
  ```bash
  mv data/log/pose_task最新编号.log data/experiment/groupB/trial_B${i}_pose.log
  cp data/log/conversion_task10_*.log data/experiment/groupB/
  ```

---

## 3. Group C — Function Calling Agent 模式

### 3.1 与 Group A/B 的关键区别
- **不需要** `image_to_llm_node` 和 `image_conversion`
- 只需要 `agent_node` + `track_path`
- 使用 `agent_launch.py` 一键启动

### 3.2 启动步骤

**终端 1**：底盘和相机（如果还在运行就不用重启）

**终端 2：启动 Agent 模式（一键启动 agent_node + track_path）**
```bash
cd /home/kuko/humble_ws
source install/setup.bash
ros2 launch image_to_llm agent_launch.py skill_name:=agent_default max_turns:=8 2>&1 | tee data/experiment/groupC/agent_full_log.txt
```
> `tee` 命令会把所有终端输出同时保存到日志文件，这是 Group C 最重要的数据源！

等待看到 `Agent Node 已启动!` 后再继续。

**终端 3**：pose_logger（同前）

### 3.3 执行试验（重复 5 次）

对每次试验 i（i = 1, 2, 3, 4, 5）：

1. **复位小车**到胶带标记位置
2. **重启 pose_logger**
3. **在 agent 日志中标记试验开始**（在终端 2 中观察）
4. **手动触发 Agent**：
   ```bash
   ros2 service call /trigger_llm_plan std_srvs/srv/Trigger
   ```
5. **开始计时**
6. **观察终端 2 的 Agent 输出**，你会看到类似：
   ```
   [Agent] --- Turn 1/8 ---
   [Agent] Gemini 调用工具 'get_robot_pose', 参数: {}
   [Agent] 工具 'get_robot_pose' 返回: {"x": 0.123, "y": -0.456, "yaw_deg": 85.2}
   [Agent] --- Turn 2/8 ---
   [Agent] Gemini 调用工具 'get_front_image', 参数: {}
   [Agent] 已获取前方图像并传入 Gemini
   [Agent] --- Turn 3/8 ---
   [Agent] Gemini 调用工具 'publish_path', 参数: {"points": [...]}
   [RobotTools] 已发布 6 个路径点到 /path
   [Agent] 任务完成: 已规划路径...
   ```
7. **重要**：Agent 模式下 track_path 到达路径终点后会自动再次触发 `/trigger_llm_plan`，Agent 会自动开始下一轮规划。如果 Agent 调用了 `finish_task()`，小车会自动停车。
8. 等待小车到达终点（Agent 调用 finish_task）或超时 120 秒
9. 记录：
   - 试验编号：C-i
   - 成功/失败
   - 碰撞
   - 总时间
   - Agent 被触发了几次（track_path 调了几次 /trigger_llm_plan）
   - 每次触发内的 Turn 数
   - 是否调用了 label_region()，标注了什么
10. 停止 pose_logger，保存轨迹：
    ```bash
    mv data/log/pose_task最新编号.log data/experiment/groupC/trial_C${i}_pose.log
    ```

### 3.4 关于 Agent 日志保存
由于终端 2 使用了 `tee` 命令，所有 5 次试验的完整日志都在 `data/experiment/groupC/agent_full_log.txt` 中。如果你希望每次试验单独保存，可以每次试验前重新启动 launch（但这样会比较麻烦）。更简单的做法是：在每次试验的触发之前，在终端里手动加一行标记：
```bash
echo "===== TRIAL C-${i} START =====" >> data/experiment/groupC/agent_full_log.txt
```

### 3.5 完成 5 次后
- 检查 `data/experiment/groupC/` 下有 5 个 pose 文件 + 1 个完整 agent 日志

---

## 4. 数据采集清单

完成所有 15 次试验后，你应该有以下文件：

```
data/experiment/
├── groupA/
│   ├── trial_A1_pose.log      # 试验 1 轨迹
│   ├── trial_A2_pose.log
│   ├── trial_A3_pose.log
│   ├── trial_A4_pose.log
│   ├── trial_A5_pose.log
│   └── conversion_task0_*.log  # image_conversion 转换日志
├── groupB/
│   ├── trial_B1_pose.log
│   ├── trial_B2_pose.log
│   ├── trial_B3_pose.log
│   ├── trial_B4_pose.log
│   ├── trial_B5_pose.log
│   └── conversion_task10_*.log
├── groupC/
│   ├── trial_C1_pose.log
│   ├── trial_C2_pose.log
│   ├── trial_C3_pose.log
│   ├── trial_C4_pose.log
│   ├── trial_C5_pose.log
│   └── agent_full_log.txt      # Agent 完整日志（含所有工具调用）
└── images/
    └── scene_overview.jpg       # 场景全景照片
```

另外，在纸上/手机备忘录中手动记录的数据（每次试验）：

| 字段 | 说明 |
|------|------|
| 试验编号 | A-1, A-2, ... B-1, ... C-1, ... |
| 成功/失败 | ✅/❌ |
| 总时间 (s) | 手机秒表计时 |
| 碰撞 | 是/否，碰了什么 |
| Gemini 调用次数 | Group A/B: 终端 2 中 "Gemini 返回成功" 次数 |
| Agent Turn 数 | Group C only: 终端 2 中 "Turn X/8" 的最大 X |
| Agent 触发次数 | Group C only: "收到触发请求" 出现几次 |
| label_region 次数 | Group C only: "语义标签" 出现几次 |
| 备注 | 任何异常情况（API 超时、深度图异常等） |

---

## 5. 数据后处理方法

### 5.1 从 pose_logger 提取轨迹数据

pose_logger 输出格式类似：
```
=== 位姿记录日志 (Task 0) ===
时间: ...
记录点数: ...
采样间隔: 0.5s
持续时间: ...

--- 汇总 ---
起点: (...), yaw=...
终点: (...), yaw=...
总位移: ...

--- 详细轨迹 (N 个点) ---
   时间(s)           x           y    yaw(°)
      0.5     +0.0000     +0.0000     +85.2
      1.0     +0.0120     -0.0030     +85.5
      1.5     +0.0450     -0.0100     +84.8
...
```

**提取为 CSV**（可以用 Python 或手动）：
```python
# data/scripts/parse_pose_log.py
import re, csv, sys

def parse_pose_log(filepath):
    """解析 pose_logger 输出的 .log 文件为列表"""
    data = []
    with open(filepath) as f:
        in_table = False
        for line in f:
            line = line.strip()
            if line.startswith('时间(s)'):
                in_table = True
                continue
            if in_table and line:
                parts = line.split()
                if len(parts) >= 4:
                    try:
                        t, x, y, yaw = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
                        data.append((t, x, y, yaw))
                    except ValueError:
                        pass
    return data

if __name__ == '__main__':
    filepath = sys.argv[1]
    data = parse_pose_log(filepath)
    outpath = filepath.replace('.log', '.csv')
    with open(outpath, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['time_s', 'x_m', 'y_m', 'yaw_deg'])
        w.writerows(data)
    print(f"已导出 {len(data)} 条数据到 {outpath}")
```

批量处理：
```bash
for f in data/experiment/group*/trial_*_pose.log; do
    python3 data/scripts/parse_pose_log.py "$f"
done
```

### 5.2 计算评价指标

```python
# data/scripts/compute_metrics.py
"""
从 pose log CSV 计算各项评价指标。
用法: python3 compute_metrics.py trial_A1_pose.csv
"""
import csv, math, sys, numpy as np

def load_csv(path):
    data = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                't': float(row['time_s']),
                'x': float(row['x_m']),
                'y': float(row['y_m']),
                'yaw': float(row['yaw_deg'])
            })
    return data

def compute_metrics(data):
    if len(data) < 2:
        return {}
    
    # 1. 总时间
    total_time = data[-1]['t'] - data[0]['t']
    
    # 2. 总行驶距离
    total_dist = 0.0
    for i in range(1, len(data)):
        dx = data[i]['x'] - data[i-1]['x']
        dy = data[i]['y'] - data[i-1]['y']
        total_dist += math.hypot(dx, dy)
    
    # 3. 角速度方差（Path Smoothness）
    angular_velocities = []
    for i in range(1, len(data)):
        dt = data[i]['t'] - data[i-1]['t']
        if dt <= 0:
            continue
        dyaw = data[i]['yaw'] - data[i-1]['yaw']
        # 处理角度回绕 (-180 到 180)
        while dyaw > 180: dyaw -= 360
        while dyaw < -180: dyaw += 360
        omega = math.radians(dyaw) / dt  # rad/s
        angular_velocities.append(omega)
    
    omega_variance = float(np.var(angular_velocities)) if angular_velocities else 0.0
    omega_mean = float(np.mean(np.abs(angular_velocities))) if angular_velocities else 0.0
    
    # 4. 起终点距离（直线距离）
    start_to_end = math.hypot(
        data[-1]['x'] - data[0]['x'],
        data[-1]['y'] - data[0]['y']
    )
    
    # 5. 路径效率（直线距离 / 实际行驶距离）
    path_efficiency = start_to_end / total_dist if total_dist > 0 else 0.0
    
    return {
        'total_time_s': round(total_time, 1),
        'total_distance_m': round(total_dist, 3),
        'straight_distance_m': round(start_to_end, 3),
        'path_efficiency': round(path_efficiency, 3),
        'angular_vel_variance_rad2s2': round(omega_variance, 6),
        'angular_vel_mean_abs_rads': round(omega_mean, 4),
        'start_pose': f"({data[0]['x']:.3f}, {data[0]['y']:.3f}, {data[0]['yaw']:.1f}°)",
        'end_pose': f"({data[-1]['x']:.3f}, {data[-1]['y']:.3f}, {data[-1]['yaw']:.1f}°)",
    }

if __name__ == '__main__':
    path = sys.argv[1]
    data = load_csv(path)
    metrics = compute_metrics(data)
    print(f"\n=== {path} ===")
    for k, v in metrics.items():
        print(f"  {k}: {v}")
```

### 5.3 从 Agent 日志提取 Group C 数据

```python
# data/scripts/parse_agent_log.py
"""
从 agent_full_log.txt 提取每次试验的工具调用统计。
用法: python3 parse_agent_log.py agent_full_log.txt
"""
import re, sys

def parse_agent_log(filepath):
    with open(filepath) as f:
        content = f.read()
    
    # 按试验分割（如果你手动插入了 ===== TRIAL C-X START ===== 标记）
    trials = re.split(r'={5,}\s*TRIAL\s+C-\d+\s+START\s*={5,}', content)
    if len(trials) <= 1:
        # 没有标记，当作一整段处理
        trials = [content]
    
    for i, trial in enumerate(trials):
        if not trial.strip():
            continue
        
        # 统计触发次数
        triggers = len(re.findall(r'收到触发请求', trial))
        
        # 统计各工具调用次数
        tool_calls = re.findall(r"Gemini 调用工具 '(\w+)'", trial)
        tool_counts = {}
        for t in tool_calls:
            tool_counts[t] = tool_counts.get(t, 0) + 1
        
        # 统计 Turn 数
        turns = re.findall(r'Turn (\d+)/\d+', trial)
        max_turn = max(int(t) for t in turns) if turns else 0
        total_turns = len(turns)
        
        # 统计 label_region
        labels = re.findall(r"语义标签 '([^']+)'", trial)
        
        # 统计 finish_task
        finish = re.findall(r'finish_task 已调用.*原因: (.+)', trial)
        
        # 统计 publish_path 发布的路径点数
        path_points = re.findall(r'已发布 (\d+) 个路径点到 /path', trial)
        total_path_points = sum(int(p) for p in path_points)
        
        print(f"\n=== Trial {i} ===")
        print(f"  触发次数 (trigger): {triggers}")
        print(f"  总 Turn 数: {total_turns}")
        print(f"  工具调用统计: {tool_counts}")
        print(f"  路径点总数: {total_path_points}")
        print(f"  语义标签: {labels}")
        print(f"  finish_task: {finish}")

if __name__ == '__main__':
    parse_agent_log(sys.argv[1])
```

### 5.4 汇总结果表

最终你需要把以下数据填入报告中的表格：

```python
# data/scripts/summary_table.py
"""
汇总所有试验数据，输出最终对比表格。
需要手动填入的数据用 ??? 标记。
"""

# ======= 手动填入区域 =======
# 从纸上记录填入
results = {
    'A': {
        'trials': [
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
        ]
    },
    'B': {
        'trials': [
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'gemini_calls': 0, 'collision': False},
        ]
    },
    'C': {
        'trials': [
            {'success': True, 'time_s': 0, 'agent_triggers': 0, 'total_turns': 0, 'labels': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'agent_triggers': 0, 'total_turns': 0, 'labels': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'agent_triggers': 0, 'total_turns': 0, 'labels': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'agent_triggers': 0, 'total_turns': 0, 'labels': 0, 'collision': False},
            {'success': True, 'time_s': 0, 'agent_triggers': 0, 'total_turns': 0, 'labels': 0, 'collision': False},
        ]
    }
}
# ======= 手动填入区域结束 =======

# 从 compute_metrics.py 的输出填入
# angular_vel_variance 从各 trial 的 CSV 计算后取平均
# step_distance = total_distance / gemini_calls

print("请将上述 results 字典中的 0 替换为实际数据，然后运行此脚本生成汇总表。")
```

### 5.5 绘制轨迹对比图

```python
# data/scripts/plot_trajectories.py
"""
绘制三组试验的轨迹对比图（用于报告 Figure 4.6）。
用法: python3 plot_trajectories.py
"""
import csv, matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # 无 GUI 模式（Jetson 上可能没有显示器）

def load_csv(path):
    xs, ys = [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row['x_m']))
            ys.append(float(row['y_m']))
    return xs, ys

# 修改下面的文件路径为实际路径
files = {
    'Group A (Naive)': 'data/experiment/groupA/trial_A1_pose.csv',
    'Group B (Optimized)': 'data/experiment/groupB/trial_B1_pose.csv',
    'Group C (Agent)': 'data/experiment/groupC/trial_C1_pose.csv',
}

fig, ax = plt.subplots(figsize=(10, 6))
colors = {'Group A (Naive)': 'red', 'Group B (Optimized)': 'blue', 'Group C (Agent)': 'green'}

for label, path in files.items():
    try:
        xs, ys = load_csv(path)
        ax.plot(xs, ys, label=label, color=colors[label], linewidth=1.5)
        ax.plot(xs[0], ys[0], 'o', color=colors[label], markersize=8)  # 起点
        ax.plot(xs[-1], ys[-1], 's', color=colors[label], markersize=8)  # 终点
    except FileNotFoundError:
        print(f"文件不存在: {path}")

ax.set_xlabel('X (m) - odom frame')
ax.set_ylabel('Y (m) - odom frame')
ax.set_title('Comparison of Navigation Trajectories')
ax.legend()
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('data/experiment/trajectory_comparison.png', dpi=200)
print("轨迹对比图已保存到 data/experiment/trajectory_comparison.png")
```

---

## 6. 常见问题排查

### 6.1 相机驱动没有输出
```bash
# 检查相机话题是否存在
ros2 topic list | grep camera
# 应该看到 /camera/color/image_raw, /camera/depth/image_raw, /camera/depth/camera_info
```

### 6.2 Gemini API 超时或 503
- 正常现象，节点内置了指数退避重试（最多 5 次）
- 如果连续失败，检查网络代理是否正常
- 记录在备注栏即可

### 6.3 track_path 没有自动触发重规划
```bash
# 检查服务是否可用
ros2 service list | grep trigger
# 应该看到 /trigger_llm_plan
```

### 6.4 Agent 模式下小车不动
- 检查终端 2 是否有工具调用输出
- 检查是否调用了 `publish_path()` 且返回了 `success: true`
- 可能是 get_front_image() 超时（RGB 图像未就绪），等待几秒后重试

### 6.5 复位时 odom 有漂移
- 每次复位后用 `ros2 topic echo /odom --once` 确认位置
- 如果 odom 有累积误差，记录实际起始坐标到备注中

---

## 7. 快速参考卡（打印带到实验室）

```
===== Group A (Naive, 单次) =====
终端2: ros2 launch image_to_llm llm_node_launch.py skill_name:=naive
终端3: ros2 launch my_robot_planner robot_vlm_launch.py task_id:=0
触发:  ros2 service call /trigger_llm_plan std_srvs/srv/Trigger

===== Group B (Optimized, 单次) =====
终端2: ros2 launch image_to_llm llm_node_launch.py skill_name:=default
终端3: ros2 launch my_robot_planner robot_vlm_launch.py task_id:=10
触发:  ros2 service call /trigger_llm_plan std_srvs/srv/Trigger

===== Group C (Agent, 多轮) =====
终端2: ros2 launch image_to_llm agent_launch.py 2>&1 | tee data/experiment/groupC/agent_full_log.txt
触发:  ros2 service call /trigger_llm_plan std_srvs/srv/Trigger

===== 每次试验 =====
1. 复位小车到胶带标记
2. 重启 pose_logger: python3 data/pose_logger.py
3. 触发 Gemini，开始秒表计时
4. 等待到达/超时
5. Ctrl+C 停止 pose_logger
6. 重命名日志文件
7. 在纸上记录: 成功? 时间? 碰撞? Gemini调用次数?
```
