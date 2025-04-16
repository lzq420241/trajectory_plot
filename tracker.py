from dataclasses import dataclass
from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import butter, filtfilt
from collections import deque


class EventType(Enum):
    POSITION = 1
    ANNOTATION = 2


@dataclass
class TrajectoryEvent:
    event_type: EventType
    data: dict  # 包含坐标、标签等信息


class RealtimeTrajectoryTracker:
    def __init__(self, window_size=50, acc_var_thresh=0.05, gyro_thresh=5.0, zupt_alpha=0.3, event_callback=None):
        # 滑动窗口配置
        self.window_size = window_size  # 1秒窗口（100Hz数据）
        self.acc_window = deque(maxlen=window_size)
        self.gyro_window = deque(maxlen=window_size)

        # 静止检测阈值
        self.acc_var_thresh = acc_var_thresh
        self.gyro_thresh = gyro_thresh

        # 零偏与状态
        self.bias = np.zeros(3)
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)
        self.zupt_alpha = zupt_alpha  # 零偏更新系数
        self.event_callback = event_callback

        # 滤波器设置：为了减少加速度传感器噪声
        self.low_pass_filter = self._butter_lowpass(0.5, 100)

    def _euler_to_quaternion(self, yaw, pitch, roll):
        """将ZYX欧拉角转换为四元数（单位：度）"""
        return Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True).as_quat()

    def _detect_static(self):
        # 计算加速度方差和角速度均值
        acc_mag = np.linalg.norm(self.acc_window, axis=1)
        acc_var = np.var(acc_mag)
        gyro_mag_mean = np.mean(np.linalg.norm(self.gyro_window, axis=1))

        return acc_var < self.acc_var_thresh and gyro_mag_mean < self.gyro_thresh

    def _update_bias(self, acc_frame, quaternion):
        """动态更新零偏（指数平滑）"""
        # 理论重力方向（世界坐标系到设备坐标系）
        R = Rotation.from_quat(quaternion).as_matrix()
        gravity_world = np.array([0, 0, 9.81])
        gravity_device = R.T @ gravity_world  # 设备坐标系中的理论重力

        # 零偏估计：当前观测值 - 理论重力
        current_bias = acc_frame - gravity_device
        self.bias = self.zupt_alpha * current_bias + (1 - self.zupt_alpha) * self.bias

    def _butter_lowpass(self, cutoff, fs, order=4):
        """设计低通滤波器"""
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff)
        return b, a

    def _low_pass_filter_data(self, data):
        """对加速度数据进行低通滤波"""
        return filtfilt(self.low_pass_filter[0], self.low_pass_filter[1], data, axis=0)

    def process_frame(self, current_position):
        position_event = TrajectoryEvent(
            event_type=EventType.POSITION,
            data={'position': current_position}
        )
        self.event_callback(position_event)

    # def process_frame(self, acc_raw, gyro_raw, euler_angles):
    #     """处理单帧数据，返回当前位置"""
    #     # 1. 欧拉角转四元数
    #     yaw, pitch, roll = euler_angles
    #     quat = self._euler_to_quaternion(yaw, pitch, roll)
    #
    #     """实时静止检测（单帧+窗口）"""
    #     self.acc_window.append(acc_raw)
    #     self.gyro_window.append(gyro_raw)
    #
    #     if len(self.acc_window) < self.window_size:
    #         return# 窗口未填满时不触发检测
    #
    #     # 2. 静止检测
    #     is_static = self._detect_static()
    #
    #     # 3. 零偏更新（仅在静止时更新）
    #     if is_static:
    #         self._update_bias(acc_raw, quat)
    #
    #     # 4. 加速度数据去噪（低通滤波）
    #     acc_filtered = self._low_pass_filter_data(np.array(self.acc_window))
    #
    #     # 5. 运动加速度计算
    #     acc_corrected = acc_filtered[-1] - self.bias
    #     R = Rotation.from_quat(quat).as_matrix()
    #     a_world = R @ acc_corrected - np.array([0, 0, 9.81])  # 去除重力
    #
    #     # 6. 积分与ZUPT
    #     dt = 0.01  # 100Hz对应0.01秒
    #     if is_static:
    #         self.velocity *= 0  # 速度归零
    #         # return
    #     else:
    #         self.velocity += a_world * dt
    #         self.position += self.velocity * dt
    #
    #     current_position = self.position.copy()
    #
    #     print(f"current_position: {current_position}")
    #
    #     # 触发位置更新事件
    #     if self.event_callback:
    #         position_event = TrajectoryEvent(
    #             event_type=EventType.POSITION,
    #             data={'position': current_position}
    #         )
    #         self.event_callback(position_event)
    #
    #     # 检测踢击动作并触发标注事件
    #     if self._detect_kick(acc_raw):
    #         kick_event = TrajectoryEvent(
    #             event_type=EventType.ANNOTATION,
    #             data={'position': current_position, 'label': 'Kick!'}
    #         )
    #         self.event_callback(kick_event)

    @staticmethod
    def _detect_kick(acc_raw, threshold=15.0):
        """检测加速度突变的踢击动作"""
        return np.linalg.norm(acc_raw) > threshold