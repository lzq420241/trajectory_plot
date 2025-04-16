import matplotlib

matplotlib.use('Qt5Agg')  # 在导入pyplot前设置

import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from queue import Queue
from collections import deque
from tracker import TrajectoryEvent, EventType


class RealtimeTrajectoryVisualizer:
    def __init__(self, max_points=5000):
        # 必须使用支持多线程的后端
        # matplotlib.use('Qt5Agg')
        print("当前Matplotlib后端:", matplotlib.get_backend())
        plt.ion()  # 启用交互模式

        # 初始化图形
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line, = self.ax.plot([], [], [], 'b-', lw=1, antialiased=False)
        self.point, = self.ax.plot([], [], [], 'ro', markersize=6)

        # 坐标轴设置
        self.ax.set_xlim(-200, 200)
        self.ax.set_ylim(-200, 200)
        self.ax.set_zlim(0, 300)
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

        # 数据存储
        self.max_points = max_points
        self.position_queue = Queue(maxsize=5000)
        self.annotation_queue = Queue(maxsize=100)
        self.positions = deque(maxlen=max_points)
        for _ in range(3):
            self.position_queue.put([0.0, 0.0, 0.0])

        # 主线程定时器
        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self._check_queue)
        self.timer.start()

    def _add_annotation(self, pos, label):
        """添加动作标注"""
        # 创建标注点
        point = self.ax.scatter([pos[0]], [pos[1]], [pos[2]], c='r', marker='*', s=100)
        # 添加文字标签
        text = self.ax.text(pos[0], pos[1], pos[2], label, color='green')
        self.annotation_queue.put((point, text))
        # 限制标注数量
        if self.annotation_queue.qsize() > 10:
            self.annotation_queue.get()

    def _show_plot(self):
        plt.show(block=True)  # 阻塞式显示，需在独立线程运行

    def add_event(self, event: TrajectoryEvent):
        """处理不同类型的事件"""
        if event.event_type == EventType.POSITION:
            self._add_position(event.data['position'])
        elif event.event_type == EventType.ANNOTATION:
            self._add_annotation(event.data['position'], event.data['label'])

    def _check_queue(self):
        """主线程定时更新（50ms间隔）"""
        while not self.position_queue.empty():
            pos = self.position_queue.get_nowait()
            self.positions.append(pos)

        if len(self.positions) > 0:
            # 更新轨迹
            pos_array = np.array(self.positions)
            self.line.set_data(pos_array[:, 0], pos_array[:, 1])
            self.line.set_3d_properties(pos_array[:, 2])

            # 更新当前位置
            last_pos = pos_array[-1]
            self.point.set_data([last_pos[0]], [last_pos[1]])
            self.point.set_3d_properties([last_pos[2]])

        # 处理标注更新
        while not self.annotation_queue.empty():
            pos, label = self.annotation_queue.get_nowait()
            self.ax.text(pos[0], pos[1], pos[2], label, color='green', fontsize=8)

        # 触发轻量级重绘
        self.fig.canvas.draw_idle()

    def _add_position(self, pos):
        """添加新位置到缓冲区"""
        self.position_queue.put(pos)
        if self.position_queue.qsize() > self.max_points:
            self.position_queue.get()
