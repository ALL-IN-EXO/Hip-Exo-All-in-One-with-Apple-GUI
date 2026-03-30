#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一滤波器库 - 支持多种滤波器类型
包括: Butterworth, Bessel, Chebyshev2, Kalman, 指数加权平均
用于实时信号处理
"""

import numpy as np
from scipy.signal import butter, bessel, cheby2, lfilter_zi
from abc import ABC, abstractmethod


# =====================================================
#              推荐参数表 (采样率 100Hz)
# =====================================================
RECOMMENDED_FILTERS = {
    # 格式: 'name': {'cutoff': Hz, 'order': N, 'type': 'filter_type', 'description': '用途说明'}
    
    # Butterworth 滤波器 - 通带最平坦
    'butter_1_2hz_2nd': {'cutoff': 1.2, 'order': 2, 'type': 'butterworth', 
                       'description': '极强平滑，适合去除高频噪声'},
    'butter_1_5hz_2nd': {'cutoff': 1.5, 'order': 2, 'type': 'butterworth', 
                       'description': '极强平滑，适合去除高频噪声'},
    'butter_3hz_2nd': {'cutoff': 3.0, 'order': 2, 'type': 'butterworth', 
                       'description': '极强平滑，适合去除高频噪声'},
    'butter_4hz_2nd': {'cutoff': 4.0, 'order': 2, 'type': 'butterworth', 
                       'description': '极强平滑，适合去除高频噪声'},
    'butter_5hz_2nd': {'cutoff': 5.0, 'order': 2, 'type': 'butterworth', 
                       'description': '极强平滑，适合去除高频噪声'},
    'butter_6hz_2nd': {'cutoff': 6.0, 'order': 2, 'type': 'butterworth',
                       'description': '强平滑，保留主要运动特征'},
    'butter_12hz_2nd': {'cutoff': 12.0, 'order': 2, 'type': 'butterworth',
                        'description': '★默认推荐★ 平衡平滑与响应'},
    'butter_15hz_2nd': {'cutoff': 15.0, 'order': 2, 'type': 'butterworth',
                        'description': '轻度平滑，较快响应'},
    'butter_20hz_2nd': {'cutoff': 20.0, 'order': 2, 'type': 'butterworth',
                        'description': '最小延迟，适合快速动作'},
    
    'butter_12hz_4th': {'cutoff': 12.0, 'order': 4, 'type': 'butterworth',
                        'description': '更陡峭滚降，更强噪声抑制'},
    
    # Bessel 滤波器 - 最小相位失真（线性相位）
    'bessel_12hz_2nd': {'cutoff': 12.0, 'order': 2, 'type': 'bessel',
                        'description': '线性相位，最小波形失真'},
    'bessel_15hz_2nd': {'cutoff': 15.0, 'order': 2, 'type': 'bessel',
                        'description': '快速响应，保持波形形状'},
    'bessel_12hz_4th': {'cutoff': 12.0, 'order': 4, 'type': 'bessel',
                        'description': '更好的线性相位特性'},
    
    # Chebyshev2 滤波器 - 阻带最平坦（反向切比雪夫）
    'cheby2_12hz_2nd': {'cutoff': 12.0, 'order': 2, 'type': 'chebyshev2',
                        'description': '陡峭滚降，通带无纹波'},
    'cheby2_12hz_4th': {'cutoff': 15.0, 'order': 4, 'type': 'chebyshev2',
                        'description': '最陡峭滚降，强噪声抑制'},
    'cheby2_8hz_2nd': {'cutoff': 8.0, 'order': 2, 'type': 'chebyshev2',
                        'description': '陡峭滚降，通带无纹波'},
    
    # 指数加权平均
    'ema_alpha_0.3': {'alpha': 0.3, 'type': 'ema',
                      'description': '强平滑 (等效~5Hz)'},
    'ema_alpha_0.5': {'alpha': 0.5, 'type': 'ema',
                      'description': '★快速响应★ (等效~10Hz)'},
    'ema_alpha_0.7': {'alpha': 0.7, 'type': 'ema',
                      'description': '轻度平滑 (等效~15Hz)'},
    
    # Kalman 滤波器
    'kalman_default': {'process_variance': 1e-5, 'measurement_variance': 1e-2, 'type': 'kalman',
                       'description': '★自适应滤波★ 动态噪声抑制'},
    'kalman_aggressive': {'process_variance': 1e-6, 'measurement_variance': 1e-1, 'type': 'kalman',
                          'description': '强平滑，信任模型'},
    'kalman_responsive': {'process_variance': 1e-4, 'measurement_variance': 1e-3, 'type': 'kalman',
                          'description': '快速跟踪，信任测量'},
}


# =====================================================
#              滤波器基类
# =====================================================
class BaseFilter(ABC):
    """滤波器抽象基类"""
    
    @abstractmethod
    def reset(self):
        """重置滤波器状态"""
        pass
    
    @abstractmethod
    def filter(self, x):
        """
        对输入进行滤波
        
        Args:
            x: 输入标量
        
        Returns:
            y: 滤波后的输出
        """
        pass
    
    def __call__(self, x):
        """支持 filter(x) 调用方式"""
        return self.filter(x)


# =====================================================
#              IIR 滤波器 (Butterworth, Bessel, Chebyshev2)
# =====================================================
class IIRFilter(BaseFilter):
    """
    通用IIR滤波器类
    支持 Butterworth, Bessel, Chebyshev2
    
    差分方程: y[n] = b0*x[n] + b1*x[n-1] + ... - a1*y[n-1] - a2*y[n-2] - ...
    """
    
    def __init__(self, b, a):
        """
        Args:
            b: 分子系数 [b0, b1, b2, ...]
            a: 分母系数 [1, a1, a2, ...] (a[0]必须为1)
        """
        self.b = np.array(b, dtype=np.float64)
        self.a = np.array(a, dtype=np.float64)
        
        if self.a[0] != 1.0:
            raise ValueError("分母系数 a[0] 必须为 1.0")
        
        self.order = len(self.b) - 1
        self.x_hist = np.zeros(len(self.b))  # 输入历史
        self.y_hist = np.zeros(len(self.a))  # 输出历史
    
    def reset(self):
        """重置滤波器状态"""
        self.x_hist[:] = 0.0
        self.y_hist[:] = 0.0
    
    def filter(self, x):
        """
        单步滤波
        
        Args:
            x: 输入标量
        
        Returns:
            y: 滤波后的输出
        """
        # 更新输入历史 (右移)
        self.x_hist[1:] = self.x_hist[:-1]
        self.x_hist[0] = x
        
        # 更新输出历史 (右移)
        self.y_hist[1:] = self.y_hist[:-1]
        
        # 计算输出: y[n] = sum(b*x) - sum(a[1:]*y[1:])
        y = np.dot(self.b, self.x_hist) - np.dot(self.a[1:], self.y_hist[1:])
        
        self.y_hist[0] = y
        return y


# =====================================================
#              指数加权平均滤波器 (EMA)
# =====================================================
class EMAFilter(BaseFilter):
    """
    指数加权平均滤波器 (Exponential Moving Average)
    
    y[n] = alpha * x[n] + (1-alpha) * y[n-1]
    
    alpha 越大，响应越快；alpha 越小，平滑越强
    等效截止频率约为: fc ≈ alpha * fs / (2*pi)
    """
    
    def __init__(self, alpha=0.5):
        """
        Args:
            alpha: 平滑系数 (0 < alpha <= 1)
                   alpha=1: 无滤波
                   alpha=0.5: 中等平滑
                   alpha=0.1: 强平滑
        """
        if not 0 < alpha <= 1:
            raise ValueError("alpha 必须在 (0, 1] 范围内")
        
        self.alpha = alpha
        self.y_prev = 0.0
        self.initialized = False
    
    def reset(self):
        """重置滤波器状态"""
        self.y_prev = 0.0
        self.initialized = False
    
    def filter(self, x):
        """
        单步滤波
        
        Args:
            x: 输入标量
        
        Returns:
            y: 滤波后的输出
        """
        if not self.initialized:
            self.y_prev = x  # 首次输入直接作为输出
            self.initialized = True
            return x
        
        y = self.alpha * x + (1 - self.alpha) * self.y_prev
        self.y_prev = y
        return y


# =====================================================
#              Kalman 滤波器 (1D)
# =====================================================
class KalmanFilter(BaseFilter):
    """
    一维卡尔曼滤波器
    
    适合处理带测量噪声的信号，自适应调整滤波强度
    """
    
    def __init__(self, process_variance=1e-5, measurement_variance=1e-2, initial_estimate=0.0):
        """
        Args:
            process_variance: 过程噪声方差 (Q) - 模型不确定性
                             越小越信任预测模型，滤波越强
            measurement_variance: 测量噪声方差 (R) - 传感器噪声
                                 越小越信任测量值，响应越快
            initial_estimate: 初始估计值
        """
        self.q = process_variance      # 过程噪声
        self.r = measurement_variance  # 测量噪声
        self.x = initial_estimate      # 状态估计
        self.p = 1.0                   # 估计误差协方差
    
    def reset(self):
        """重置滤波器状态"""
        self.x = 0.0
        self.p = 1.0
    
    def filter(self, measurement):
        """
        单步滤波
        
        Args:
            measurement: 测量值
        
        Returns:
            x: 滤波后的状态估计
        """
        # 预测步骤
        x_pred = self.x  # 简单模型: x[n+1] = x[n]
        p_pred = self.p + self.q
        
        # 更新步骤
        k = p_pred / (p_pred + self.r)  # Kalman 增益
        self.x = x_pred + k * (measurement - x_pred)
        self.p = (1 - k) * p_pred
        
        return self.x


# =====================================================
#              滤波器工厂函数
# =====================================================
def compute_iir_coeffs(cutoff_freq, filter_type='butterworth', sample_rate=100.0, order=2, **kwargs):
    """
    计算IIR滤波器系数
    
    Args:
        cutoff_freq: 截止频率 (Hz)
        filter_type: 滤波器类型 ('butterworth', 'bessel', 'chebyshev2')
        sample_rate: 采样频率 (Hz)
        order: 滤波器阶数
        **kwargs: 其他参数
            - rs (float): Chebyshev2 阻带衰减 (dB)，默认40dB
    
    Returns:
        b, a: 滤波器系数 (numpy arrays)
    """
    if cutoff_freq <= 0 or cutoff_freq >= sample_rate / 2:
        raise ValueError(f"截止频率必须在 (0, {sample_rate/2}) 范围内, 当前: {cutoff_freq}")
    
    if filter_type == 'butterworth':
        b, a = butter(N=order, Wn=cutoff_freq, btype='low', fs=sample_rate)
    elif filter_type == 'bessel':
        b, a = bessel(N=order, Wn=cutoff_freq, btype='low', fs=sample_rate, norm='mag')
    elif filter_type == 'chebyshev2':
        rs = kwargs.get('rs', 40)  # 阻带衰减，默认40dB
        b, a = cheby2(N=order, rs=rs, Wn=cutoff_freq, btype='low', fs=sample_rate)
    else:
        raise ValueError(f"不支持的滤波器类型: {filter_type}")
    
    return b.astype(np.float64), a.astype(np.float64)


def create_filter(filter_name=None, **custom_params):
    """
    滤波器工厂函数 - 根据名称或自定义参数创建滤波器
    
    Args:
        filter_name: 预定义滤波器名称 (见 RECOMMENDED_FILTERS)
        **custom_params: 自定义参数，会覆盖预定义参数
            - cutoff: 截止频率 (Hz)
            - order: 滤波器阶数
            - filter_type: 'butterworth', 'bessel', 'chebyshev2', 'ema', 'kalman'
            - sample_rate: 采样频率 (Hz)
            - alpha: EMA平滑系数
            - process_variance, measurement_variance: Kalman参数
            - b, a: 直接指定IIR系数
    
    Returns:
        filter: 滤波器对象
    
    Examples:
        # 使用预定义滤波器
        f1 = create_filter('butter_12hz_2nd')
        
        # 自定义参数
        f2 = create_filter(cutoff=15, order=2, filter_type='butterworth', sample_rate=100)
        
        # 直接指定系数
        f3 = create_filter(b=[0.0461, 0.0923, 0.0461], a=[1.0, -1.3073, 0.4918])
        
        # EMA滤波器
        f4 = create_filter('ema_alpha_0.5')
        f5 = create_filter(filter_type='ema', alpha=0.6)
        
        # Kalman滤波器
        f6 = create_filter('kalman_default')
        f7 = create_filter(filter_type='kalman', process_variance=1e-4)
    """
    # 获取基础配置
    config = {}
    if filter_name is not None:
        if filter_name not in RECOMMENDED_FILTERS:
            raise ValueError(f"未知的预定义滤波器: {filter_name}\n"
                           f"可用选项: {list(RECOMMENDED_FILTERS.keys())}")
        config = RECOMMENDED_FILTERS[filter_name].copy()
    
    # 自定义参数覆盖
    config.update(custom_params)
    
    # 获取滤波器类型
    filter_type = config.get('filter_type') or config.get('type')
    
    # 直接指定系数的情况
    if 'b' in config and 'a' in config:
        return IIRFilter(b=config['b'], a=config['a'])
    
    # 根据类型创建滤波器
    if filter_type == 'ema':
        alpha = config.get('alpha')
        if alpha is None:
            raise ValueError("EMA滤波器需要指定 alpha 参数")
        return EMAFilter(alpha=alpha)
    
    elif filter_type == 'kalman':
        process_var = config.get('process_variance', 1e-5)
        measure_var = config.get('measurement_variance', 1e-2)
        return KalmanFilter(process_variance=process_var, 
                          measurement_variance=measure_var)
    
    elif filter_type in ['butterworth', 'bessel', 'chebyshev2']:
        cutoff = config.get('cutoff')
        order = config.get('order', 2)
        sample_rate = config.get('sample_rate', 100.0)
        
        if cutoff is None:
            raise ValueError(f"{filter_type}滤波器需要指定 cutoff 参数")
        
        # 提取其他参数（避免重复传递）
        extra_kwargs = {k: v for k, v in config.items() 
                       if k not in ['cutoff', 'order', 'sample_rate', 'type', 'filter_type', 'description']}
        
        b, a = compute_iir_coeffs(cutoff, filter_type, sample_rate, order, **extra_kwargs)
        return IIRFilter(b=b, a=a)
    
    else:
        raise ValueError(f"未指定滤波器类型或类型不支持: {filter_type}")


# =====================================================
#              便捷函数 (向后兼容)
# =====================================================
def compute_filter_coeffs(cutoff_freq, sample_rate=100.0, order=2):
    """
    向后兼容的 Butterworth 滤波器系数计算函数
    
    Args:
        cutoff_freq: 截止频率 (Hz)
        sample_rate: 采样频率 (Hz)
        order: 滤波器阶数
    
    Returns:
        b, a: 滤波器系数
    """
    return compute_iir_coeffs(cutoff_freq, 'butterworth', sample_rate, order)


# 向后兼容的别名
ButterworthFilter = IIRFilter
LPF = IIRFilter


# =====================================================
#              工具函数
# =====================================================
def print_filter_recommendations():
    """打印所有推荐的滤波器配置"""
    print("\n" + "="*80)
    print("推荐滤波器配置 (采样率 100Hz)".center(80))
    print("="*80 + "\n")
    
    categories = {
        'Butterworth': [],
        'Bessel': [],
        'Chebyshev2': [],
        'EMA': [],
        'Kalman': []
    }
    
    for name, config in RECOMMENDED_FILTERS.items():
        ftype = config['type']
        if 'butterworth' in ftype:
            categories['Butterworth'].append((name, config))
        elif 'bessel' in ftype:
            categories['Bessel'].append((name, config))
        elif 'chebyshev2' in ftype:
            categories['Chebyshev2'].append((name, config))
        elif 'ema' in ftype:
            categories['EMA'].append((name, config))
        elif 'kalman' in ftype:
            categories['Kalman'].append((name, config))
    
    for category, items in categories.items():
        if items:
            print(f"\n【{category}】")
            print("-" * 80)
            for name, config in items:
                desc = config.get('description', '')
                if 'cutoff' in config:
                    info = f"{config['cutoff']}Hz, {config['order']}阶"
                elif 'alpha' in config:
                    info = f"alpha={config['alpha']}"
                else:
                    info = "自适应"
                print(f"  '{name}'")
                print(f"    参数: {info}")
                print(f"    说明: {desc}")
    
    print("\n" + "="*80)
    print("使用示例:")
    print("  filter = create_filter('butter_12hz_2nd')  # 使用预定义")
    print("  filter = create_filter(cutoff=15, order=2, filter_type='bessel')  # 自定义")
    print("  y = filter.filter(x)  # 滤波单个值")
    print("="*80 + "\n")


def compare_filters(input_signal, filter_names, sample_rate=100.0):
    """
    比较不同滤波器的效果
    
    Args:
        input_signal: 输入信号数组
        filter_names: 要比较的滤波器名称列表
        sample_rate: 采样频率
    
    Returns:
        results: {filter_name: filtered_signal}
    """
    results = {}
    
    for name in filter_names:
        filt = create_filter(name)
        output = np.zeros_like(input_signal)
        
        for i, x in enumerate(input_signal):
            output[i] = filt.filter(x)
        
        results[name] = output
    
    return results


# =====================================================
#              测试代码
# =====================================================
if __name__ == "__main__":
    # 打印推荐配置
    print_filter_recommendations()
    
    # 测试滤波器
    print("\n" + "="*80)
    print("滤波器测试".center(80))
    print("="*80 + "\n")
    
    # 生成测试信号: 正弦波 + 噪声
    t = np.arange(0, 5, 0.01)  # 5秒, 100Hz
    signal = np.sin(2 * np.pi * 2 * t)  # 2Hz正弦波
    noise = 0.5 * np.random.randn(len(t))  # 噪声
    noisy_signal = signal + noise
    
    # 测试不同滤波器
    test_filters = [
        'butter_12hz_2nd',
        'bessel_12hz_2nd',
        'cheby2_12hz_2nd',
        'ema_alpha_0.5',
        'kalman_default'
    ]
    
    results = compare_filters(noisy_signal, test_filters)
    
    print("已生成测试信号:")
    print(f"  长度: {len(noisy_signal)} 样本")
    print(f"  采样率: 100Hz")
    print(f"  信号: 2Hz正弦波 + 高斯噪声")
    print(f"\n测试的滤波器: {test_filters}")
    print(f"\n结果已保存到 results 字典")
    print("\n提示: 使用 matplotlib 可视化比较效果:")
    print("  import matplotlib.pyplot as plt")
    print("  plt.plot(t, noisy_signal, alpha=0.5, label='Noisy')")
    print("  for name, filtered in results.items():")
    print("      plt.plot(t, filtered, label=name)")
    print("  plt.legend(); plt.show()")
