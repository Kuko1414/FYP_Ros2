#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

import os
import json
import re
import time
import cv2
from PIL import Image as PILImage
from google import genai
from dotenv import load_dotenv

class ImageToLLMNode(Node):
    def __init__(self):
        super().__init__('image_to_llm_node')
        
        self.bridge = CvBridge()
        
        # 声明参数
        self.declare_parameter('rgb_topic', '/camera/color/image_raw') 
        self.declare_parameter('pixel_path_topic', '/llm_pixels')
        self.declare_parameter('env_path', 'src/image_to_llm/llm_config.env')
        self.declare_parameter('skill_name', 'default')
        
        rgb_topic = self.get_parameter('rgb_topic').value
        pixel_path_topic = self.get_parameter('pixel_path_topic').value
        env_path = self.get_parameter('env_path').value
        skill_name = self.get_parameter('skill_name').value
        
        # 尝试加载 .env 配置文件（API_KEY、代理等敏感信息）
        if os.path.exists(env_path):
            load_dotenv(env_path, override=True)
            self.get_logger().info(f"已加载配置文件: {env_path}")
        else:
            self.get_logger().warn(f"未找到配置文件: {env_path}, 将尝试使用系统的环境变量。请确保你创建了它避免泄露API KEY")
        
        # 从环境变量提取敏感配置（API_KEY、代理）
        self.api_key = os.getenv('GEMINI_API_KEY', '')
        http_proxy = os.getenv('HTTP_PROXY', '')
        https_proxy = os.getenv('HTTPS_PROXY', '')
        
        if not self.api_key:
            self.get_logger().error("未配置 GEMINI_API_KEY，节点可能无法正常工作！")
            
        # 配置代理
        if http_proxy:
            os.environ["http_proxy"] = http_proxy
        if https_proxy:
            os.environ["https_proxy"] = https_proxy
        
        # 加载 Skill（prompt 和模型配置从 YAML 文件读取）
        from image_to_llm.skills import load_skill, list_skills
        try:
            self.skill = load_skill(skill_name)
            self.get_logger().info(f"已加载 Skill: '{self.skill.name}' — {self.skill.description}")
        except FileNotFoundError as e:
            self.get_logger().error(str(e))
            self.get_logger().warn("将使用空 prompt 运行，可能无法正常工作")
            self.skill = None
        
        # 模型优先级：env 中的 GEMINI_MODEL > Skill YAML 中的 model > 默认值
        env_model = os.getenv('GEMINI_MODEL', '')
        if env_model:
            self.model_name = env_model
        elif self.skill and self.skill.model:
            self.model_name = self.skill.model
        else:
            self.model_name = 'gemini-2.5-flash'
        
        # 初始化 Gemini
        self.client = genai.Client(api_key=self.api_key)
        
        # 回调组：将 service 放入独立的回调组，避免阻塞 rgb_callback
        self._srv_cb_group = MutuallyExclusiveCallbackGroup()
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅彩色图片
        self.sub_rgb = self.create_subscription(
            Image, 
            rgb_topic, 
            self.rgb_callback, 
            qos_profile_sensor_data
        )
        
        # 发布 2D 像素 JSON
        self.pixel_pub = self.create_publisher(String, pixel_path_topic, 10)
        
        # 触发服务（放入独立回调组，配合 MultiThreadedExecutor 使用）
        self.srv = self.create_service(Trigger, 'trigger_llm_plan', self.plan_callback,
                                       callback_group=self._srv_cb_group)
        
        self.latest_rgb = None
        self._rgb_ready_logged = False
        
        self.get_logger().info(f"Image_to_LLM Node 已启动... 模型: {self.model_name}")
        self.get_logger().info(f"订阅彩色图像: {rgb_topic}")
        self.get_logger().info("⏳ 等待首帧 RGB 图像...")
        self.get_logger().info("触发命令: ros2 service call /trigger_llm_plan std_srvs/srv/Trigger")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not self._rgb_ready_logged:
                self._rgb_ready_logged = True
                self.get_logger().info("✅ 已收到首帧 RGB 图像，服务 /trigger_llm_plan 已就绪。")
        except Exception as e:
            self.get_logger().error(f"RGB 转换错误: {e}")

    def plan_callback(self, request, response):
        if self.latest_rgb is None:
            response.success = False
            response.message = "还未收到彩色图像！"
            return response
            
        self.get_logger().info("向 Gemini 发送图像以获取像素路径点...")
        
        cv_rgb = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(cv_rgb)
        img_width, img_height = pil_img.size
        
        # 从 Skill 获取 prompt，替换 {width}/{height} 占位符
        if self.skill and self.skill.system_prompt:
            prompt = self.skill.system_prompt.replace('{width}', str(img_width)).replace('{height}', str(img_height))
        else:
            prompt = f"图片实际尺寸为 {img_width}x{img_height}。请规划20个点..."
            self.get_logger().warn("警告：未加载 Skill 或 Skill 中无 system_prompt")

        max_retries = 5
        base_delay = 2.0  # 初始退避秒数

        for attempt in range(1, max_retries + 1):
            try:
                llm_response = self.client.models.generate_content(
                    model=self.model_name,
                    contents=[prompt, pil_img]
                )
                raw_text = llm_response.text.strip()
                
                # 提取 JSON
                json_str = raw_text
                if json_str.startswith("```"):
                    json_str = re.sub(r'^```(?:json)?\s*', '', json_str)
                if json_str.endswith("```"):
                    json_str = json_str[:-3]
                json_str = json_str.strip()
                json_match = re.search(r'\[.*\]', json_str, re.DOTALL)
                if json_match:
                    json_str = json_match.group(0)
                
                # 先验证 JSON 合法性，再发布
                parsed = json.loads(json_str)
                
                msg = String()
                msg.data = json_str
                self.pixel_pub.publish(msg)
                
                self.get_logger().info(f"Gemini 返回成功，已发布包含 {len(parsed)} 个像素点的 JSON。")
                response.success = True
                response.message = f"成功获取 {len(parsed)} 个点"
                return response
                    
            except Exception as e:
                error_str = str(e)
                is_retryable = '503' in error_str or 'UNAVAILABLE' in error_str or '429' in error_str or 'RESOURCE_EXHAUSTED' in error_str
                
                if is_retryable and attempt < max_retries:
                    delay = base_delay * (2 ** (attempt - 1))  # 指数退避: 2, 4, 8, 16s
                    self.get_logger().warn(
                        f"LLM 暂时不可用 (尝试 {attempt}/{max_retries})：{e}。"
                        f"{delay:.0f} 秒后重试..."
                    )
                    time.sleep(delay)
                    self.get_logger().info("向 Gemini 发送图像以获取像素路径点...")
                else:
                    self.get_logger().error(f"LLM 错误 (尝试 {attempt}/{max_retries})：{e}")
                    response.success = False
                    response.message = error_str
                    return response
        
        # 理论上不会到达这里，但作为安全兜底
        response.success = False
        response.message = "已达最大重试次数"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageToLLMNode()
    # 使用多线程执行器，使 service 回调（Gemini API 调用）不阻塞 rgb_callback
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()