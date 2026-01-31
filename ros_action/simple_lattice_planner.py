"""
简化版Lattice Planner
基于原始lattice_planner.py的简化实现
用于实时路径跟踪和修正
"""

import math
import copy
import numpy as np

# 尝试相对导入，如果失败则使用绝对导入
try:
    from .curve_generators import CircleTrajectory
except ImportError:
    from curve_generators.circle_trajectory import CircleTrajectory


class PlannerConfig:
    """规划器配置参数"""
    
    # 车辆限制
    MAX_SPEED = 1.0  # m/s (降低速度以适应小车)
    MAX_ACCEL = 2.0  # m/s^2
    MAX_CURVATURE = 1.0  # 1/m
    
    # 道路和采样参数
    ROAD_WIDTH = 0.5  # m (根据实际调整)
    ROAD_SAMPLE_STEP = 0.2  # m
    
    # 时间参数
    T_STEP = 0.1  # s
    MIN_T = 1.0  # s
    MAX_T = 2.0  # s
    
    # 目标速度
    TARGET_SPEED = 0.5  # m/s
    
    # 代价函数权重
    K_JERK = 0.1
    K_TIME = 0.1
    K_V_DIFF = 1.0
    K_OFFSET = 2.0
    K_COLLISION = 500
    
    # 车辆几何参数（根据实际小车调整）
    K_SIZE = 0.3
    RF = 0.15  # 车辆前端到后轴距离
    RB = 0.05  # 车辆后端到后轴距离
    W = 0.15   # 车辆宽度
    WB = 0.12  # 轴距
    
    # 障碍物列表（初始为空）
    obstacles = []


class TrajectoryPath:
    """轨迹路径数据结构"""
    
    def __init__(self):
        self.t = []        # 时间序列
        
        # 横向状态（Frenet坐标）
        self.l = []        # 横向位置
        self.l_v = []      # 横向速度
        self.l_a = []      # 横向加速度
        self.l_jerk = []   # 横向急动度
        
        # 纵向状态（Frenet坐标）
        self.s = []        # 纵向位置
        self.s_v = []      # 纵向速度
        self.s_a = []      # 纵向加速度
        self.s_jerk = []   # 纵向急动度
        
        # 笛卡尔坐标
        self.x = []        # x坐标
        self.y = []        # y坐标
        self.yaw = []      # 航向角
        self.ds = []       # 路径段长度
        self.curv = []     # 曲率
        
        self.cost = 0.0    # 路径代价


class QuarticPolynomial:
    """
    四次多项式
    用于纵向轨迹规划（位置、速度、加速度约束）
    """
    
    def __init__(self, xs, vxs, axs, vxe, axe, T):
        """
        Args:
            xs: 初始位置
            vxs: 初始速度
            axs: 初始加速度
            vxe: 终点速度
            axe: 终点加速度
            T: 时间
        """
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.vxe = vxe
        self.axe = axe
        
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0
        
        A = np.array([[3 * T ** 2, 4 * T ** 3],
                      [6 * T, 12 * T ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        
        self.a3 = x[0]
        self.a4 = x[1]
    
    def calc_xt(self, t):
        """计算t时刻的位置"""
        return self.a0 + self.a1 * t + self.a2 * t ** 2 + \
               self.a3 * t ** 3 + self.a4 * t ** 4
    
    def calc_dxt(self, t):
        """计算t时刻的速度"""
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2 + \
               4 * self.a4 * t ** 3
    
    def calc_ddxt(self, t):
        """计算t时刻的加速度"""
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2
    
    def calc_dddxt(self, t):
        """计算t时刻的急动度"""
        return 6 * self.a3 + 24 * self.a4 * t


class QuinticPolynomial:
    """
    五次多项式
    用于横向轨迹规划（位置、速度、加速度约束）
    """
    
    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        """
        Args:
            xs: 初始位置
            vxs: 初始速度
            axs: 初始加速度
            xe: 终点位置
            vxe: 终点速度
            axe: 终点加速度
            T: 时间
        """
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0
        
        A = np.array([[T ** 3, T ** 4, T ** 5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T ** 2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        
        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]
    
    def calc_xt(self, t):
        """计算t时刻的位置"""
        return self.a0 + self.a1 * t + self.a2 * t ** 2 + \
               self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5
    
    def calc_dxt(self, t):
        """计算t时刻的速度"""
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t ** 2 + \
               4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4
    
    def calc_ddxt(self, t):
        """计算t时刻的加速度"""
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + \
               20 * self.a5 * t ** 3
    
    def calc_dddxt(self, t):
        """计算t时刻的急动度"""
        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2


class SimpleLatticePlanner:
    """简化版Lattice规划器"""
    
    def __init__(self, config=None):
        """
        初始化规划器
        
        Args:
            config: 配置参数，如果为None则使用默认配置
        """
        self.config = config if config else PlannerConfig()
        self.ref_trajectory = None
    
    def set_reference_trajectory(self, trajectory):
        """
        设置参考轨迹
        
        Args:
            trajectory: 参考轨迹对象（需要有calc_position, calc_yaw等方法）
        """
        self.ref_trajectory = trajectory
    
    def plan(self, current_state):
        """
        规划路径
        
        Args:
            current_state: 当前状态字典，包含：
                - l: 横向位置
                - l_v: 横向速度
                - l_a: 横向加速度
                - s: 纵向位置
                - s_v: 纵向速度
                - s_a: 纵向加速度
        
        Returns:
            TrajectoryPath: 最优路径，如果没有找到则返回None
        """
        if self.ref_trajectory is None:
            print("错误：未设置参考轨迹！")
            return None
        
        # 生成候选路径
        candidate_paths = self._sample_paths(current_state)
        
        # 选择最优路径
        best_path = self._select_best_path(candidate_paths)
        
        return best_path
    
    def _sample_paths(self, state):
        """
        采样生成候选路径
        
        Args:
            state: 当前状态
            
        Returns:
            dict: 候选路径字典 {path: cost}
        """
        paths = {}
        
        l0, l0_v, l0_a = state['l'], state['l_v'], state['l_a']
        s0, s0_v, s0_a = state['s'], state['s_v'], state['s_a']
        
        # 采样目标速度
        v_samples = np.arange(
            self.config.TARGET_SPEED * 0.8,
            self.config.TARGET_SPEED * 1.2,
            self.config.TARGET_SPEED * 0.2
        )
        
        # 采样时间
        t_samples = np.arange(
            self.config.MIN_T,
            self.config.MAX_T,
            0.3
        )
        
        # 采样横向位置
        l_samples = np.arange(
            -self.config.ROAD_WIDTH,
            self.config.ROAD_WIDTH + self.config.ROAD_SAMPLE_STEP,
            self.config.ROAD_SAMPLE_STEP
        )
        
        for target_v in v_samples:
            for T in t_samples:
                # 生成纵向轨迹
                lon_traj = QuarticPolynomial(s0, s0_v, s0_a, target_v, 0.0, T)
                
                # 时间序列
                t_series = np.arange(0.0, T, self.config.T_STEP)
                
                # 计算纵向状态
                s_series = [lon_traj.calc_xt(t) for t in t_series]
                s_v_series = [lon_traj.calc_dxt(t) for t in t_series]
                s_a_series = [lon_traj.calc_ddxt(t) for t in t_series]
                s_jerk_series = [lon_traj.calc_dddxt(t) for t in t_series]
                
                for target_l in l_samples:
                    # 生成横向轨迹
                    lat_traj = QuinticPolynomial(l0, l0_v, l0_a, target_l, 0.0, 0.0, T)
                    
                    # 创建路径
                    path = TrajectoryPath()
                    path.t = list(t_series)
                    
                    # 横向状态
                    path.l = [lat_traj.calc_xt(t) for t in t_series]
                    path.l_v = [lat_traj.calc_dxt(t) for t in t_series]
                    path.l_a = [lat_traj.calc_ddxt(t) for t in t_series]
                    path.l_jerk = [lat_traj.calc_dddxt(t) for t in t_series]
                    
                    # 纵向状态
                    path.s = s_series
                    path.s_v = s_v_series
                    path.s_a = s_a_series
                    path.s_jerk = s_jerk_series
                    
                    # 转换到笛卡尔坐标
                    path.x, path.y = self._frenet_to_cartesian(path.s, path.l)
                    
                    if len(path.x) < 2:
                        continue
                    
                    # 计算航向和曲率
                    path.yaw, path.curv, path.ds = self._calc_yaw_curvature(path.x, path.y)
                    
                    if path.yaw is None:
                        continue
                    
                    # 计算代价
                    path.cost = self._calculate_cost(path, target_v, T)
                    
                    paths[path] = path.cost
        
        return paths
    
    def _frenet_to_cartesian(self, s_series, l_series):
        """
        Frenet坐标转笛卡尔坐标
        
        Args:
            s_series: 纵向位置序列
            l_series: 横向位置序列
            
        Returns:
            x_series, y_series: 笛卡尔坐标序列
        """
        x_series, y_series = [], []
        
        for s, l in zip(s_series, l_series):
            x_ref, y_ref = self.ref_trajectory.calc_position(s)
            
            if x_ref is None:
                break
            
            yaw = self.ref_trajectory.calc_yaw(s)
            
            # 沿着垂直于参考线的方向偏移l
            x = x_ref + l * math.cos(yaw + math.pi / 2.0)
            y = y_ref + l * math.sin(yaw + math.pi / 2.0)
            
            x_series.append(x)
            y_series.append(y)
        
        return x_series, y_series
    
    def _calc_yaw_curvature(self, x, y):
        """
        计算航向角和曲率
        
        Args:
            x, y: 位置序列
            
        Returns:
            yaw, curv, ds: 航向角、曲率、路径段长度
        """
        yaw, curv, ds = [], [], []
        
        for i in range(len(x) - 1):
            dx = x[i + 1] - x[i]
            dy = y[i + 1] - y[i]
            ds.append(math.hypot(dx, dy))
            yaw.append(math.atan2(dy, dx))
        
        if len(yaw) == 0:
            return None, None, None
        
        yaw.append(yaw[-1])
        ds.append(ds[-1])
        
        for i in range(len(yaw) - 1):
            if ds[i] > 0:
                curv.append((yaw[i + 1] - yaw[i]) / ds[i])
            else:
                curv.append(0.0)
        
        curv.append(curv[-1] if curv else 0.0)
        
        return yaw, curv, ds
    
    def _calculate_cost(self, path, target_v, T):
        """
        计算路径代价
        
        Args:
            path: 路径对象
            target_v: 目标速度
            T: 时间
            
        Returns:
            float: 总代价
        """
        # 急动度代价
        l_jerk_sum = sum(np.abs(path.l_jerk))
        s_jerk_sum = sum(np.abs(path.s_jerk))
        
        # 速度差代价
        v_diff = abs(self.config.TARGET_SPEED - path.s_v[-1])
        
        # 偏移代价
        offset = abs(path.l[-1])
        
        # 碰撞代价
        collision = self._check_collision(path)
        
        total_cost = (
            self.config.K_JERK * (l_jerk_sum + s_jerk_sum) +
            self.config.K_V_DIFF * v_diff +
            self.config.K_TIME * T +
            self.config.K_OFFSET * offset +
            self.config.K_COLLISION * collision
        )
        
        return total_cost
    
    def _check_collision(self, path):
        """
        检查路径是否与障碍物碰撞
        
        Args:
            path: 路径对象
            
        Returns:
            float: 1.0表示碰撞，0.0表示无碰撞
        """
        if len(self.config.obstacles) == 0:
            return 0.0
        
        # 简化碰撞检测：检查路径点是否过于接近障碍物
        for x, y in zip(path.x[::5], path.y[::5]):  # 每5个点检查一次
            for obs in self.config.obstacles:
                obs_x, obs_y, obs_r = obs
                dist = math.hypot(x - obs_x, y - obs_y)
                if dist < obs_r + self.config.W / 2.0:
                    return 1.0
        
        return 0.0
    
    def _select_best_path(self, paths):
        """
        从候选路径中选择最优路径
        
        Args:
            paths: 候选路径字典
            
        Returns:
            TrajectoryPath: 最优路径
        """
        if len(paths) == 0:
            return None
        
        # 按代价排序
        sorted_paths = sorted(paths.items(), key=lambda x: x[1])
        
        for path, cost in sorted_paths:
            if self._verify_path(path):
                return path
        
        # 如果没有满足约束的路径，返回代价最小的
        return sorted_paths[0][0] if sorted_paths else None
    
    def _verify_path(self, path):
        """
        验证路径是否满足约束
        
        Args:
            path: 路径对象
            
        Returns:
            bool: True表示满足约束
        """
        # 检查速度约束
        if any([v > self.config.MAX_SPEED for v in path.s_v]):
            return False
        
        # 检查加速度约束
        if any([abs(a) > self.config.MAX_ACCEL for a in path.s_a]):
            return False
        
        # 检查曲率约束
        if any([abs(c) > self.config.MAX_CURVATURE for c in path.curv]):
            return False
        
        return True


def test_planner():
    """测试规划器"""
    import matplotlib.pyplot as plt
    
    print("=" * 50)
    print("测试简化版Lattice Planner")
    print("=" * 50)
    
    # 1. 创建圆形参考轨迹
    print("\n1. 创建圆形参考轨迹（半径=2m，圆心=(1,1)）")
    ref_traj = CircleTrajectory(center_x=1.0, center_y=1.0, radius=2.0, num_points=200)
    rx, ry, _, _, _ = ref_traj.get_trajectory()
    print(f"   轨迹总长度: {ref_traj.s[-1]:.2f} m")
    
    # 2. 创建规划器
    print("\n2. 初始化规划器")
    planner = SimpleLatticePlanner()
    planner.set_reference_trajectory(ref_traj)
    print("   规划器已初始化")
    
    # 3. 设置初始状态
    print("\n3. 设置初始状态")
    current_state = {
        'l': 0.0,      # 横向位置（在参考线上）
        'l_v': 0.0,    # 横向速度
        'l_a': 0.0,    # 横向加速度
        's': 0.0,      # 纵向位置
        's_v': 0.3,    # 纵向速度 0.3 m/s
        's_a': 0.0     # 纵向加速度
    }
    print(f"   初始位置: s={current_state['s']}, l={current_state['l']}")
    print(f"   初始速度: {current_state['s_v']} m/s")
    
    # 4. 规划路径
    print("\n4. 开始规划...")
    path = planner.plan(current_state)
    
    if path is None:
        print("   规划失败！")
        return
    
    print(f"   规划成功！路径代价: {path.cost:.2f}")
    print(f"   路径点数: {len(path.x)}")
    print(f"   规划时间: {path.t[-1]:.2f} s")
    
    # 5. 可视化
    print("\n5. 可视化结果")
    plt.figure(figsize=(12, 5))
    
    # 子图1：路径
    plt.subplot(1, 2, 1)
    plt.plot(rx, ry, 'k--', linewidth=1, label='Reference', alpha=0.5)
    plt.plot(path.x, path.y, 'b-', linewidth=2, label='Planned Path')
    plt.plot(path.x[0], path.y[0], 'go', markersize=10, label='Start')
    plt.plot(path.x[-1], path.y[-1], 'ro', markersize=10, label='End')
    plt.plot(1.0, 1.0, 'k+', markersize=15, label='Circle Center')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Planned Trajectory')
    plt.legend()
    
    # 子图2：速度曲线
    plt.subplot(1, 2, 2)
    plt.plot(path.t, path.s_v, 'b-', linewidth=2, label='Longitudinal Velocity')
    plt.axhline(y=planner.config.TARGET_SPEED, color='r', linestyle='--', label='Target Speed')
    plt.grid(True, alpha=0.3)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Velocity Profile')
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('/tmp/lattice_planner_test.png', dpi=150)
    print("   图像已保存到 /tmp/lattice_planner_test.png")
    plt.show()
    
    print("\n" + "=" * 50)
    print("测试完成！")
    print("=" * 50)


if __name__ == '__main__':
    test_planner()
