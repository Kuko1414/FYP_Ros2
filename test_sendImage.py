from google import genai
from PIL import Image, ImageDraw
import os
import json
import re

# 配置代理，解决 Gemini 地区限制问题 (http_proxy 和 https_proxy)
os.environ["http_proxy"] = "http://127.0.0.1:7897"
os.environ["https_proxy"] = "http://127.0.0.1:7897"

# 1. 初始化客户端 (优先从环境变量读取 API Key)
api_key = os.environ.get("GEMINI_API_KEY", "INPUT_YOUR_API_KEY")
client = genai.Client(api_key=api_key)


def analyze_local_image():
    image_path = "/home/kuko/Pictures/Sense.jpg"

    # 检查文件是否存在
    if not os.path.exists(image_path):
        print(f"错误：找不到文件 {image_path}")
        return

    try:
        # 2. 加载图片
        img = Image.open(image_path)
        img_width, img_height = img.size
        print(f"图片尺寸: {img_width}x{img_height}")
        print("正在上传并分析图片，请稍候...")

        # 3. 构建提示词：使用归一化坐标 [0, 1000]，统一 [x, y] 格式，注入图片尺寸
        prompt = (
            f"这张图片的实际尺寸为 {img_width}x{img_height} 像素。这是一张从正上方俯拍的室内场景图。\n\n"

            "【场景描述】\n"
            "- 图片四周的黄色区域和白色区域是墙壁/围栏，属于不可通行的边界，路径点绝对不能进入这些区域。\n"
            "- 场景中还有一些障碍物（如黄色叉车、银色金属板/管道、其他物体），路径必须绕开它们。\n"
            "- 图片底部中央偏右有一个绿色的小型AGV机器人，这是起点。\n"
            "- 图片左上方有一个写有'TVP'字样的纸箱，这是终点。\n\n"

            "【任务】\n"
            "请规划一条从绿色AGV机器人到TVP纸箱的避障路径，包含20个路径点。\n"
            "- 所有路径点必须在深色瓷砖地面范围内，远离黄色和白色的墙壁边界。\n"
            "- 路径必须绕开所有障碍物（叉车、金属板等），保持安全距离。\n\n"
            "- 路径点应该保持均匀分布，方便轮式机器人行使的平滑路径。\n"

            "【坐标系规则】\n"
            "- 原点 (0, 0) 在图片左上角，x 轴向右，y 轴向下。\n"
            "- 使用归一化坐标，范围为 [0, 1000]。(0, 0) 代表图片左上角，(1000, 1000) 代表图片右下角。\n"
            "- 每个点的格式为 [x, y]（先 x 后 y）。\n\n"

            "【输出格式】\n"
            '请只返回纯 JSON 数组，格式为：[{"point": [x, y]}, {"point": [x, y]}, ...]\n'
            "不要输出任何额外的思考过程、解释文字或 Markdown 代码块，只返回 JSON。"
        )

        # 4. 调用 Gemini 多模态模型
        response = client.models.generate_content(
            model="gemini-2.5-flash",
            contents=[prompt, img]
        )

        # 5. 解析返回结果并绘图
        raw_text = response.text.strip()
        print(f"模型原始回复:\n{raw_text}\n")

        # 清理可能的 markdown 格式包裹
        json_str = raw_text
        if json_str.startswith("```"):
            # 移除开头的 ```json 或 ```
            json_str = re.sub(r'^```(?:json)?\s*', '', json_str)
        if json_str.endswith("```"):
            json_str = json_str[:-3]
        json_str = json_str.strip()

        # 尝试用正则提取 JSON 数组部分（更健壮）
        json_match = re.search(r'\[.*\]', json_str, re.DOTALL)
        if json_match:
            json_str = json_match.group(0)

        points_data = json.loads(json_str)
        print(f"解析成功，获取到 {len(points_data)} 个路径点:")

        # 画图
        draw = ImageDraw.Draw(img)
        path_points = []

        for i, item in enumerate(points_data):
            pt = item.get("point")
            if not pt or len(pt) < 2:
                print(f"  跳过无效点 #{i}: {item}")
                continue

            # 归一化坐标 [0, 1000] 还原为实际像素坐标
            x = pt[0] / 1000.0 * img_width
            y = pt[1] / 1000.0 * img_height

            # 裁剪到图片范围内
            x = max(0, min(x, img_width - 1))
            y = max(0, min(y, img_height - 1))

            path_points.append((x, y))
            print(f"  点 #{i+1}: 归一化({pt[0]}, {pt[1]}) -> 像素({x:.0f}, {y:.0f})")

            # 画点
            r = 6
            draw.ellipse((x - r, y - r, x + r, y + r), fill="red", outline="white")

            # 标注序号
            draw.text((x + r + 2, y - r), str(i + 1), fill="yellow")

        # 画线连接路径点
        if len(path_points) > 1:
            draw.line(path_points, fill="blue", width=3)

        output_path = "/home/kuko/humble_ws/result.jpg"
        img.save(output_path)
        print(f"\n已将结果保存至 {output_path}")
        print(f"共绘制 {len(path_points)} 个有效路径点")

    except json.JSONDecodeError as e:
        print(f"JSON 解析失败: {e}")
        print(f"尝试解析的字符串:\n{json_str}")
    except Exception as e:
        print(f"调用 API 遇到错误: {e}")


if __name__ == "__main__":
    analyze_local_image()
