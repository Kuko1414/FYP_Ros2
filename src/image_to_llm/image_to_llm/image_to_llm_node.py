#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

import os
import json
import re
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
        
        rgb_topic = self.get_parameter('rgb_topic').value
        pixel_path_topic = self.get_parameter('pixel_path_topic').value
        env_path = self.get_parameter('env_path').value
        
        # 尝试加载 .env 配置文件
        if os.path.exists(env_path):
            load_dotenv(env_path)
            self.get_logger().info(f"已加载配置文件: {env_path}")
        else:
            self.get_logger().warn(f"未找到配置文件: {env_path}, 将尝试使用系统的环境变量。请确保你创建了它避免泄露API KEY")
        
        # 从环境变量提取敏感或可变配置
        self.api_key = os.getenv('GEMINI_API_KEY', '')
        self.model_name = os.getenv('GEMINI_MODEL', 'gemini-2.5-flash')
        self.custom_prompt = os.getenv('PROMPT', '')
        http_proxy = os.getenv('HTTP_PROXY', '')
        https_proxy = os.getenv('HTTPS_PROXY', '')
        
        if not self.api_key:
            self.get_logger().error("未配置 GEMINI_API_KEY，节点可能无法正常工作！")
            
        # 配置代理
        if http_proxy:
            os.environ["http_proxy"] = http_proxy
        if https_proxy:
            os.environ["https_proxy"] = https_proxy
        
        # 初始化 Gemini
        self.client = genai.Client(api_key=self.api_key)
        
        # 订阅彩色图片
        self.sub_rgb = self.create_subscription(Image, rgb_topic, self.rgb_callback, 10)
        
        # 发布 2D 像素 JSON
        self.pixel_pub = self.create_publisher(String, pixel_path_topic, 10)
        
        # 触发服务
        self.srv = self.create_service(Trigger, 'trigger_llm_plan', self.plan_callback)
        
        self.latest_rgb = None
        
        self.get_logger().info(f"Image_to_LLM Node 已启动... 模型: {self.model_name}")
        self.get_logger().info(f"订阅彩色图像: {rgb_topic}")
        self.get_logger().info("触发命令: ros2 service call /trigger_llm_plan std_srvs/srv/Trigger")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
        
        # 将配置中写好的 prompt 格式化填入实际的图片长宽
        if self.custom_prompt:
            prompt = self.custom_prompt.replace('{width}', str(img_width)).replace('{height}', str(img_height))
            # 处理配置文本中的转义换行符
            prompt = prompt.replace('\\n', '\n')
        else:
            prompt = f"图片实际尺寸为 {img_width}x{img_height}。请规划20个点..."
            self.get_logger().warn("警告：未配置 PROMPT 环境变量")

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
            
            # 发布纯 JSON 字符串
            msg = String()
            msg.data = json_str
            self.pixel_pub.publish(msg)
            
            parsed = json.loads(json_str)
            self.get_logger().info(f"Gemini 返回成功，已发布包含 {len(parsed)} 个像素点的 JSON。")
            response.success = True
            response.message = f"成功获取 {len(parsed)} 个点"
                
        except Exception as e:
            self.get_logger().error(f"LLM 错误：{e}")
            response.success = False
            response.message = str(e)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageToLLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()