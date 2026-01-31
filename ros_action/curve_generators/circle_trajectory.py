"""
圆形轨迹生成器
生成圆形参考轨迹
"""

import numpy as np
import math


class CircleTrajectory:
    """
    生成圆形轨迹
    """
    
    def __init__(self, center_x, center_y, radius, num_points=500):
        """
        初始化圆形轨迹生成器
        
        Args:
            center_x: 圆心x坐标
            center_y: 圆心y坐标
            radius: 半径
            num_points: 采样点数量
        """
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.num_points = num_points
        
        # 生成轨迹点
        self.x, self.y, self.yaw, self.curvature = self._generate_trajectory()
        
        # 计算累积路程s
        self.s = self._calculate_arc_length()
        
    def _generate_trajectory(self):
        """生成圆形轨迹的x, y, yaw, curvature"""
        theta = np.linspace(0, 2 * np.pi, self.num_points)
        
        # 生成圆上的点
        x = self.center_x + self.radius * np.cos(theta)
        y = self.center_y + self.radius * np.sin(theta)
        
        # 计算航向角（切线方向）
        yaw = theta + np.pi / 2.0  # 切线方向
        
        # 圆的曲率是常数 1/r
        curvature = np.ones(self.num_points) / self.radius
        
        return x, y, yaw, curvature
    
    def _calculate_arc_length(self):
        """计算累积弧长"""
        s = [0.0]
        for i in range(1, len(self.x)):
            dx = self.x[i] - self.x[i-1]
            dy = self.y[i] - self.y[i-1]
            ds = math.hypot(dx, dy)
            s.append(s[-1] + ds)
        return np.array(s)
    
    def calc_position(self, s_query):
        """
        根据弧长s查询位置
        
        Args:
            s_query: 查询的弧长
            
        Returns:
            x, y: 对应的x, y坐标
        """
        # 处理循环（圆形是闭合的）
        s_total = self.s[-1]
        s_query = s_query % s_total
        
        # 找到最近的点
        idx = np.searchsorted(self.s, s_query)
        if idx >= len(self.s):
            idx = len(self.s) - 1
        elif idx > 0:
            # 线性插值
            s0, s1 = self.s[idx-1], self.s[idx]
            ratio = (s_query - s0) / (s1 - s0) if s1 != s0 else 0
            x = self.x[idx-1] + ratio * (self.x[idx] - self.x[idx-1])
            y = self.y[idx-1] + ratio * (self.y[idx] - self.y[idx-1])
            return x, y
        
        return self.x[idx], self.y[idx]
    
    def calc_yaw(self, s_query):
        """
        根据弧长s查询航向角
        
        Args:
            s_query: 查询的弧长
            
        Returns:
            yaw: 对应的航向角
        """
        s_total = self.s[-1]
        s_query = s_query % s_total
        
        idx = np.searchsorted(self.s, s_query)
        if idx >= len(self.s):
            idx = len(self.s) - 1
        elif idx > 0:
            s0, s1 = self.s[idx-1], self.s[idx]
            ratio = (s_query - s0) / (s1 - s0) if s1 != s0 else 0
            yaw0 = self.yaw[idx-1]
            yaw1 = self.yaw[idx]
            # 处理角度插值
            yaw = yaw0 + ratio * self._angle_diff(yaw1, yaw0)
            return yaw
        
        return self.yaw[idx]
    
    def calc_curvature(self, s_query):
        """
        根据弧长s查询曲率
        
        Args:
            s_query: 查询的弧长
            
        Returns:
            curvature: 对应的曲率
        """
        # 圆的曲率是常数
        return 1.0 / self.radius
    
    def _angle_diff(self, angle1, angle0):
        """计算两个角度的最小差值"""
        diff = angle1 - angle0
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
    
    def get_trajectory(self):
        """
        获取完整轨迹
        
        Returns:
            x, y, yaw, curvature, s: 轨迹数据
        """
        return self.x, self.y, self.yaw, self.curvature, self.s


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    # 测试：生成半径为2，圆心为(1,1)的圆
    circle = CircleTrajectory(center_x=1.0, center_y=1.0, radius=2.0, num_points=100)
    
    x, y, yaw, curv, s = circle.get_trajectory()
    
    # 可视化
    plt.figure(figsize=(10, 5))
    
    plt.subplot(1, 2, 1)
    plt.plot(x, y, 'b-', linewidth=2, label='Trajectory')
    plt.plot(1.0, 1.0, 'ro', markersize=10, label='Center')
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Circle Trajectory')
    plt.legend()
    
    plt.subplot(1, 2, 2)
    plt.plot(s, curv, 'g-', linewidth=2)
    plt.grid(True)
    plt.xlabel('Arc Length [m]')
    plt.ylabel('Curvature [1/m]')
    plt.title('Curvature Profile')
    
    plt.tight_layout()
    plt.show()
    
    # 测试查询功能
    print(f"Total arc length: {s[-1]:.2f} m")
    print(f"Position at s=0: {circle.calc_position(0)}")
    print(f"Position at s=π: {circle.calc_position(np.pi)}")
    print(f"Yaw at s=0: {circle.calc_yaw(0):.2f} rad")
    print(f"Curvature: {circle.calc_curvature(0):.2f} 1/m (constant)")
