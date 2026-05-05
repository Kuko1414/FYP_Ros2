"""
Gemini Function Calling 的工具 Schema 定义。

核心逻辑：每个 schema 对应 RobotTools 中的一个 public 方法。
Gemini 根据 description 决定何时调用哪个工具。

新增工具时，必须同步在此文件添加 schema + 在 robot_tools.py 添加方法。
"""

from google.genai import types

# 使用 google-genai SDK 的原生 FunctionDeclaration 格式
TOOL_DECLARATIONS = [
    types.FunctionDeclaration(
        name="get_robot_pose",
        description=(
            "获取机器人当前在 odom 坐标系下的位置 (x, y) 和朝向角 (yaw_deg)。"
            "坐标单位为米，朝向角单位为度（0度=正东/x轴正方向，逆时针为正，即90度=正北）。"
            "在规划目标点前应先调用此函数了解机器人当前位置和朝向。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={},
        ),
    ),
    types.FunctionDeclaration(
        name="get_front_image",
        description=(
            "获取机器人前方摄像头的最新 RGB 图像。"
            "返回的图像可用于观察前方环境、识别目标（如门口）、判断是否到达。"
            "注意：每次调用都会消耗较多 Token，避免频繁调用。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={},
        ),
    ),
    types.FunctionDeclaration(
        name="get_obstacle_distance",
        description=(
            "获取机器人正前方最近障碍物的距离（米）。"
            "使用深度相机中央区域检测。"
            "has_obstacle=True 表示前方 0.8m 内有障碍物。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={},
        ),
    ),
    types.FunctionDeclaration(
        name="get_depth_at_regions",
        description=(
            "获取深度图各区域的精确障碍物距离和 odom 坐标。"
            "将前方视野分为 3×3 九宫格，返回每个区域中最近障碍物的距离（米）"
            "和该障碍物点在 odom 坐标系下的位置。"
            "帮助你判断目标方向是否有障碍物，以及估算目标点的大致 odom 坐标。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={},
        ),
    ),
    types.FunctionDeclaration(
        name="publish_goal_relative",
        description=(
            "【推荐】根据相对方向和距离发布导航目标点。"
            "你只需判断目标大致在左边还是右边、大约多远，代码会自动计算 odom 坐标。"
            "比 publish_goal 更简单且不容易出错。"
            "方向参考：0=正前方，正值=左偏，负值=右偏。"
            "例如：direction_deg=-30 表示右前方30°，direction_deg=20 表示左前方20°。"
            "距离建议 2-3 米（太近无意义，太远可能导致路径偏差）。"
            "机器人到达后会自动触发你的下一轮决策。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "direction_deg": types.Schema(
                    type=types.Type.NUMBER,
                    description=(
                        "相对于机器人当前朝向的目标方向（度）。"
                        "0=正前方，正值=左偏，负值=右偏。"
                        "范围建议 -45° 到 +45°（避免急转弯）。"
                    ),
                ),
                "distance_m": types.Schema(
                    type=types.Type.NUMBER,
                    description="目标距离（米），建议 2-3m。",
                ),
            },
            required=["direction_deg", "distance_m"],
        ),
    ),
    types.FunctionDeclaration(
        name="label_region",
        description=(
            "为 odom 坐标系下的一个矩形区域添加语义标签（如 'door', 'corridor'）。"
            "标签会被持久存储在本次会话中，后续可通过 get_semantic_labels 查询。"
            "用于建立环境的语义地图记忆。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "name": types.Schema(
                    type=types.Type.STRING,
                    description="区域名称（如 corridor, desk_area, door）",
                ),
                "x_min": types.Schema(type=types.Type.NUMBER, description="矩形左下角 x 坐标（米）"),
                "y_min": types.Schema(type=types.Type.NUMBER, description="矩形左下角 y 坐标（米）"),
                "x_max": types.Schema(type=types.Type.NUMBER, description="矩形右上角 x 坐标（米）"),
                "y_max": types.Schema(type=types.Type.NUMBER, description="矩形右上角 y 坐标（米）"),
            },
            required=["name", "x_min", "y_min", "x_max", "y_max"],
        ),
    ),
    types.FunctionDeclaration(
        name="get_semantic_labels",
        description=(
            "获取所有已标注的语义区域标签及其坐标范围。"
            "返回一个字典，key 为区域名称，value 为坐标范围。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={},
        ),
    ),
    types.FunctionDeclaration(
        name="get_current_region",
        description=(
            "查询机器人当前所在的语义区域名称。"
            "根据当前 odom 位置与已标注区域的边界对比。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={},
        ),
    ),
    types.FunctionDeclaration(
        name="rotate_robot",
        description=(
            "原地旋转机器人指定角度（阻塞等待完成）。"
            "正值=逆时针（左转），负值=顺时针（右转）。"
            "当需要转向观察其他方向时使用，转完后可调用 get_front_image() 观察新方向。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "angle_deg": types.Schema(
                    type=types.Type.NUMBER,
                    description="旋转角度（度）。正值=左转，负值=右转。",
                ),
            },
            required=["angle_deg"],
        ),
    ),
    types.FunctionDeclaration(
        name="finish_task",
        description=(
            "结束当前导航任务，立即停止机器人并汇报最终状态。"
            "当你判断已到达目标位置（如门口就在眼前），"
            "或者前方无路可走、任务无法继续时，必须调用此函数。"
            "调用后机器人会立即停车，并返回最终位置和语义地图。"
        ),
        parameters=types.Schema(
            type=types.Type.OBJECT,
            properties={
                "reason": types.Schema(
                    type=types.Type.STRING,
                    description="结束原因（如：已到达目标门口、前方死路无法通行）",
                ),
            },
            required=["reason"],
        ),
    ),
]
