#!/usr/bin/env python3
"""
Agent 节点：Gemini Function Calling 的主控循环。

核心逻辑：
  1. 接收触发指令（通过 /trigger_llm_plan 服务，与传统模式接口兼容）
  2. 从 Skill YAML 加载系统提示词 + 工具 Schema 发送给 Gemini
  3. 如果 Gemini 返回 function_call -> 执行对应工具 -> 将结果返回给 Gemini
  4. 循环步骤 2-3，直到 Gemini 返回纯文本（表示任务完成）
  5. 设置最大循环次数（max_turns），防止无限循环和 API 费用失控

【重要】此节点与 image_to_llm_node 是并列关系，不是替代关系。
启动 Agent 模式时不需要启动 image_to_llm_node 和 image_conversion。
track_path 仍然独立运行，Agent 通过 /path Topic 与其通信。
"""

import os
import io
import json
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from PIL import Image as PILImage

from google import genai
from google.genai import types
from dotenv import load_dotenv

from image_to_llm.robot_tools import RobotTools
from image_to_llm.tool_schemas import TOOL_DECLARATIONS
from image_to_llm.skills import load_skill


class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')

        # ---- 中断标志（Ctrl+C 时快速退出 Agent 循环）----
        self._shutdown_event = threading.Event()

        # ---- 参数 ----
        self.declare_parameter('env_path', 'src/image_to_llm/llm_config.env')
        self.declare_parameter('skill_name', 'agent_default')
        self.declare_parameter('max_turns', 10)
        self.declare_parameter('max_session_rounds', 3)  # 跨触发保留的最大轮次数

        env_path = self.get_parameter('env_path').value
        skill_name = self.get_parameter('skill_name').value
        self.max_turns = self.get_parameter('max_turns').value
        self.max_session_rounds = self.get_parameter('max_session_rounds').value

        # ---- 加载 .env（API_KEY、代理）----
        if os.path.exists(env_path):
            load_dotenv(env_path, override=True)
            self.get_logger().info(f"已加载配置文件: {env_path}")
        else:
            self.get_logger().warn(f"未找到配置文件: {env_path}")

        self.api_key = os.getenv('GEMINI_API_KEY', '')
        http_proxy = os.getenv('HTTP_PROXY', '')
        https_proxy = os.getenv('HTTPS_PROXY', '')
        if not self.api_key:
            self.get_logger().error("未配置 GEMINI_API_KEY！")
        if http_proxy:
            os.environ["http_proxy"] = http_proxy
        if https_proxy:
            os.environ["https_proxy"] = https_proxy

        # ---- 加载 Skill ----
        try:
            self.skill = load_skill(skill_name)
            self.get_logger().info(
                f"已加载 Skill: '{self.skill.name}' - {self.skill.description}")
            if self.skill.max_turns > 0:
                self.max_turns = self.skill.max_turns
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            self.skill = None

        # ---- 模型选择 ----
        env_model = os.getenv('GEMINI_MODEL', '')
        if env_model:
            self.model_name = env_model
        elif self.skill and self.skill.model:
            self.model_name = self.skill.model
        else:
            self.model_name = 'gemini-2.5-flash'

        # ---- 初始化 Gemini Client ----
        self.client = genai.Client(api_key=self.api_key)

        # ---- 初始化工具层（RobotTools 会在内部创建 ROS2 订阅）----
        self.tools = RobotTools(self)

        # ---- 工具分发表：函数名 -> 实际方法 ----
        self.tool_dispatcher = {
            "get_robot_pose":        self.tools.get_robot_pose,
            "get_front_image":       self.tools.get_front_image,
            "get_obstacle_distance": self.tools.get_obstacle_distance,
            "get_depth_at_regions":  self.tools.get_depth_at_regions,
            "publish_goal_relative": self.tools.publish_goal_relative,
            "rotate_robot":          self.tools.rotate_robot,
            "label_region":          self.tools.label_region,
            "get_semantic_labels":   self.tools.get_semantic_labels,
            "get_current_region":    self.tools.get_current_region,
            "finish_task":           self.tools.finish_task,
        }

        # ---- Gemini 工具配置 ----
        self.gemini_tools = types.Tool(function_declarations=TOOL_DECLARATIONS)

        # ---- 回调组（service 放独立组，避免阻塞 Topic 回调）----
        self._srv_cb_group = MutuallyExclusiveCallbackGroup()

        # ---- 触发服务（与 image_to_llm_node 的接口完全兼容）----
        self.srv = self.create_service(
            Trigger, 'trigger_llm_plan', self.plan_callback,
            callback_group=self._srv_cb_group)

        # ---- 跨触发对话历史摘要 ----
        self._session_summaries = []  # list of str, 每轮结束后的摘要

        self.get_logger().info(f"Agent Node 已启动! 模型: {self.model_name}, "
                               f"最大轮数: {self.max_turns}")
        self.get_logger().info(
            "触发命令: ros2 service call /trigger_llm_plan std_srvs/srv/Trigger")

    # ================================================================
    #  服务回调：触发 Agent 任务循环
    # ================================================================

    def plan_callback(self, request, response):
        """被 track_path 或手动触发时，执行一次完整的 Agent 多轮循环。"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("[Agent] 收到触发请求，开始 Agent 任务循环...")

        # 构建系统提示词
        if self.skill and self.skill.system_prompt:
            system_prompt = self.skill.system_prompt
        else:
            system_prompt = (
                "你是一个轮式机器人的导航 Agent。"
                "你可以调用工具来获取机器人状态、查看前方图像、发布导航路径。"
                "请根据当前环境做出合理的导航决策。"
            )

        try:
            result_text = self._execute_agent_loop(system_prompt)
            response.success = True
            response.message = result_text[:200] if result_text else "Agent 任务完成"
        except Exception as e:
            self.get_logger().error(f"[Agent] 任务执行异常: {e}")
            response.success = False
            response.message = str(e)[:200]

        self.get_logger().info("=" * 50)
        return response

    # ================================================================
    #  Agent 多轮循环核心
    # ================================================================

    def _execute_agent_loop(self, system_prompt: str) -> str:
        """执行一个完整的 Agent 任务循环。

        Args:
            system_prompt: 系统提示词（来自 Skill YAML）

        Returns:
            Gemini 最终的文本输出
        """
        # 构建初始提示词：系统提示 + 历史摘要注入
        initial_text = system_prompt
        if self._session_summaries:
            history_block = "\n".join(
                f"  第{i+1}轮: {s}"
                for i, s in enumerate(self._session_summaries))
            initial_text += (
                f"\n\n【历史上下文】以下是之前规划轮次的摘要，"
                f"请基于这些信息继续规划（不要重复已走过的路线）：\n{history_block}")
            self.get_logger().info(
                f"[Agent] 注入 {len(self._session_summaries)} 轮历史摘要")

        # 对话历史：首轮发送系统提示词（含历史摘要）作为 user 消息
        contents = [
            types.Content(
                role="user",
                parts=[types.Part.from_text(text=initial_text)]
            )
        ]

        # 本轮摘要收集
        _round_pose = None
        _round_path_count = 0
        _round_actions = []
        _has_published_goal = False   # 本轮是否调用过 publish_goal
        _has_finished_task = False    # 本轮是否调用过 finish_task
        _no_action_retries = 0       # "只分析不行动"重试计数
        _max_no_action_retries = 2   # 最多重试次数

        for turn in range(1, self.max_turns + 1):
            # 检查是否收到中断信号
            if self._shutdown_event.is_set():
                self.get_logger().info("[Agent] 收到中断信号，退出 Agent 循环")
                return "Agent 循环被中断"

            self.get_logger().info(
                f"[Agent] --- Turn {turn}/{self.max_turns} ---")

            # 调用 Gemini（带工具声明）
            try:
                gemini_response = self.client.models.generate_content(
                    model=self.model_name,
                    contents=contents,
                    config=types.GenerateContentConfig(
                        tools=[self.gemini_tools],
                    ),
                )
            except Exception as e:
                error_str = str(e)
                # 可重试的错误
                is_retryable = any(k in error_str for k in
                                   ['503', 'UNAVAILABLE', '429', 'RESOURCE_EXHAUSTED'])
                if is_retryable and turn < self.max_turns:
                    delay = 2 ** turn
                    self.get_logger().warn(
                        f"[Agent] Gemini 暂时不可用: {e}. {delay}s 后重试...")
                    time.sleep(delay)
                    continue
                else:
                    raise

            # 解析响应
            candidate = gemini_response.candidates[0]
            response_parts = candidate.content.parts

            # 检查是否包含 function_call
            function_calls = [
                p for p in response_parts
                if p.function_call is not None
            ]

            if not function_calls:
                # 没有 function_call -> Gemini 返回纯文本
                final_text = gemini_response.text or "(Agent 未返回文本)"

                # 兜底检查：如果本轮没有调用过 publish_goal 或 finish_task，
                # 说明 Gemini "只分析不行动"，强制要求它再试一次
                if (not _has_published_goal and not _has_finished_task
                        and _no_action_retries < _max_no_action_retries
                        and turn < self.max_turns):
                    _no_action_retries += 1
                    self.get_logger().warn(
                        f"[Agent] ⚠️ Gemini 未调用 publish_goal 或 finish_task 就结束了！"
                        f"强制重试 ({_no_action_retries}/{_max_no_action_retries})")
                    # 把 Gemini 的文本响应加入历史，再追加提醒消息
                    contents.append(candidate.content)
                    contents.append(types.Content(
                        role="user",
                        parts=[types.Part.from_text(
                            text="WARNING: You have not called publish_goal_relative() or finish_task()! "
                                 "The robot is stopped. You MUST call one of these tools NOW: "
                                 "- To move forward: publish_goal_relative(direction_deg, distance_m) "
                                 "- To end the task: finish_task(reason) "
                                 "Act immediately!"
                        )]
                    ))
                    continue  # 回到循环，让 Gemini 再跑一轮

                self.get_logger().info(f"[Agent] 任务完成: {final_text[:100]}")
                # 保存本轮摘要
                self._save_round_summary(_round_pose, _round_actions, final_text)
                return final_text

            # 有 function_call -> 逐个执行
            # 先把 Gemini 的响应加入对话历史
            contents.append(candidate.content)

            # 收集所有 function_response parts
            function_response_parts = []

            for fc_part in function_calls:
                fc = fc_part.function_call
                tool_name = fc.name
                tool_args = dict(fc.args) if fc.args else {}

                self.get_logger().info(
                    f"[Agent] Gemini 调用工具 '{tool_name}', "
                    f"参数: {json.dumps(tool_args, ensure_ascii=False, default=str)}")

                # 执行工具
                if tool_name not in self.tool_dispatcher:
                    result = {"error": f"未知工具: {tool_name}"}
                    self.get_logger().warn(f"[Agent] 未知工具: {tool_name}")
                else:
                    try:
                        result = self.tool_dispatcher[tool_name](**tool_args)
                    except Exception as e:
                        result = {"error": f"工具执行失败: {str(e)}"}
                        self.get_logger().error(
                            f"[Agent] 工具 '{tool_name}' 执行失败: {e}")

                # 收集摘要信息
                if tool_name == "get_robot_pose" and isinstance(result, dict) and "x" in result:
                    _round_pose = result
                elif tool_name == "publish_goal_relative" and isinstance(result, dict):
                    _has_published_goal = True
                    _round_actions.append(
                        f"发布目标点({result.get('goal_x', 0):.2f},{result.get('goal_y', 0):.2f})")
                elif tool_name == "finish_task":
                    _has_finished_task = True
                elif tool_name == "get_front_image":
                    _round_actions.append("观察前方")

                # 特殊处理 get_front_image：返回的是 PIL Image
                if tool_name == "get_front_image":
                    if result is not None and isinstance(result, PILImage.Image):
                        # 图像作为 function_response 返回，同时缓存图像待后续追加
                        function_response_parts.append(
                            types.Part.from_function_response(
                                name=tool_name,
                                response={"status": "图像已获取，将在下方展示"}
                            )
                        )
                        # 先提交已积累的 function_response（含本次图像的 response）
                        contents.append(types.Content(
                            role="user",
                            parts=function_response_parts
                        ))
                        function_response_parts = []  # 清空，避免重复提交
                        # 将 PIL Image 转为 JPEG bytes，使用 from_bytes 传入
                        img_buf = io.BytesIO()
                        result.save(img_buf, format='JPEG', quality=85)
                        img_bytes = img_buf.getvalue()
                        # 追加图像作为 user 消息
                        contents.append(types.Content(
                            role="user",
                            parts=[
                                types.Part.from_text(
                                    text="这是 get_front_image 返回的前方摄像头图像，请分析:"),
                                types.Part.from_bytes(
                                    data=img_bytes,
                                    mime_type='image/jpeg'),
                            ]
                        ))
                        self.get_logger().info(
                            f"[Agent] 已获取前方图像并传入 Gemini")
                        # 图像已单独处理，跳过普通 function_response 流程
                        continue
                    else:
                        result = {"error": "前方图像未就绪"}

                # 普通工具：构造 function_response Part
                self.get_logger().info(
                    f"[Agent] 工具 '{tool_name}' 返回: "
                    f"{json.dumps(result, ensure_ascii=False, default=str)[:150]}")

                function_response_parts.append(
                    types.Part.from_function_response(
                        name=tool_name,
                        response=result
                    )
                )

                # 特殊处理 finish_task：立即结束 Agent 循环
                if tool_name == "finish_task":
                    self.get_logger().info(
                        f"[Agent] finish_task 已调用，Agent 循环结束。"
                        f"原因: {tool_args.get('reason', 'unknown')}")
                    # finish_task 时清空历史摘要（任务真正完成）
                    self._session_summaries.clear()
                    self.get_logger().info("[Agent] 任务结束，已清空历史摘要")
                    # 把 function_response 返回给 Gemini 后立即退出
                    if function_response_parts:
                        contents.append(types.Content(
                            role="user",
                            parts=function_response_parts
                        ))
                    return (f"任务已结束: {tool_args.get('reason', '')}. "
                            f"语义地图: {json.dumps(result.get('semantic_map', {}), ensure_ascii=False, default=str)}")

            # 将所有 function_response 作为一条 user 消息加入对话历史
            if function_response_parts:
                contents.append(types.Content(
                    role="user",
                    parts=function_response_parts
                ))

        # 超过最大轮数，也保存摘要
        self._save_round_summary(_round_pose, _round_actions, "超时未完成")
        self.get_logger().warn(
            f"[Agent] 达到最大轮数 {self.max_turns}，强制结束")
        return f"Agent 达到最大轮数 {self.max_turns}，任务未完成"

    # ================================================================
    #  跨触发对话历史摘要管理
    # ================================================================

    def _save_round_summary(self, pose, actions, result_text):
        """生成并保存本轮的文本摘要，用于下一次触发时注入。"""
        parts = []
        if pose:
            parts.append(f"位置=({pose['x']:.1f},{pose['y']:.1f},yaw={pose['yaw_deg']:.0f}°)")
        if actions:
            parts.append("动作=[" + ",".join(actions) + "]")
        if result_text:
            # 截取 Gemini 的总结（前80字）
            summary_text = result_text[:80].replace("\n", " ")
            parts.append(f"结果: {summary_text}")

        summary = "; ".join(parts) if parts else "无有效信息"
        self._session_summaries.append(summary)

        # 截断：只保留最近 N 轮
        while len(self._session_summaries) > self.max_session_rounds:
            self._session_summaries.pop(0)

        self.get_logger().info(
            f"[Agent] 已保存本轮摘要（当前共 {len(self._session_summaries)} 轮）: {summary[:100]}")


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    # 使用多线程执行器：service 回调（Gemini API 调用）不阻塞 Topic 回调
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[Agent] 收到 Ctrl+C，正在关闭...")
        node._shutdown_event.set()  # 通知 Agent 循环立即退出
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 忽略重复 shutdown 错误


if __name__ == '__main__':
    main()
