import matplotlib.pyplot as plt
import datetime
import numpy as np

# 假设的时间戳数据
timestamps = [
    '2025-01-29 11:10:04.583',
    '2025-01-29 11:10:04.583',
    '2025-01-29 11:10:04.583',
    '2025-01-29 11:10:04.583',
    '2025-01-29 11:10:04.616',
    '2025-01-29 11:10:04.616',
    '2025-01-29 11:10:04.616',
    '2025-01-29 11:10:04.616',
    '2025-01-29 11:10:04.661',
    '2025-01-29 11:10:04.661',
    '2025-01-29 11:10:04.662',
    '2025-01-29 11:10:04.662',
    '2025-01-29 11:10:04.706',
    '2025-01-29 11:10:04.706',
    '2025-01-29 11:10:04.706',
    '2025-01-29 11:10:04.706',
    '2025-01-29 11:10:04.740',
    '2025-01-29 11:10:04.740',
    '2025-01-29 11:10:04.740',
    '2025-01-29 11:10:04.740',
    '2025-01-29 11:10:04.785',
    '2025-01-29 11:10:04.785',
    '2025-01-29 11:10:04.785',
    '2025-01-29 11:10:04.785',
    '2025-01-29 11:10:04.818',
    '2025-01-29 11:10:04.818',
    '2025-01-29 11:10:04.818',
    '2025-01-29 11:10:04.818',
    '2025-01-29 11:10:04.863',
    '2025-01-29 11:10:04.863',
    '2025-01-29 11:10:04.863',
    '2025-01-29 11:10:04.863',
    '2025-01-29 11:10:04.897',
    '2025-01-29 11:10:04.898',
    '2025-01-29 11:10:04.898',
    '2025-01-29 11:10:04.898',
    '2025-01-29 11:10:04.942',
    '2025-01-29 11:10:04.943',
    '2025-01-29 11:10:04.943',
    '2025-01-29 11:10:04.943',
    '2025-01-29 11:10:04.988',
    '2025-01-29 11:10:04.988',
    '2025-01-29 11:10:04.988',
    '2025-01-29 11:10:04.988',
    '2025-01-29 11:10:05.021',
    '2025-01-29 11:10:05.021',
    '2025-01-29 11:10:05.021',
    '2025-01-29 11:10:05.021',
    '2025-01-29 11:10:05.066',
    '2025-01-29 11:10:05.066',
    '2025-01-29 11:10:05.066',
    '2025-01-29 11:10:05.066',
    '2025-01-29 11:10:05.100',
    '2025-01-29 11:10:05.100',
    '2025-01-29 11:10:05.100',
    '2025-01-29 11:10:05.100',
    '2025-01-29 11:10:05.145',
    '2025-01-29 11:10:05.145',
    '2025-01-29 11:10:05.145',
    '2025-01-29 11:10:05.145',
    '2025-01-29 11:10:05.190',
    '2025-01-29 11:10:05.190',
    '2025-01-29 11:10:05.190',
    '2025-01-29 11:10:05.190',
    '2025-01-29 11:10:05.224',
    '2025-01-29 11:10:05.224',
    '2025-01-29 11:10:05.224',
    '2025-01-29 11:10:05.224',
    '2025-01-29 11:10:05.269',
    '2025-01-29 11:10:05.269',
    '2025-01-29 11:10:05.269',
    '2025-01-29 11:10:05.269',
    '2025-01-29 11:10:05.302',
    '2025-01-29 11:10:05.302',
    '2025-01-29 11:10:05.302',
    '2025-01-29 11:10:05.302',
    '2025-01-29 11:10:05.347',
    '2025-01-29 11:10:05.347',
    '2025-01-29 11:10:05.347',
    '2025-01-29 11:10:05.347',
    '2025-01-29 11:10:05.381',
    '2025-01-29 11:10:05.381',
    '2025-01-29 11:10:05.381',
    '2025-01-29 11:10:05.381',
    '2025-01-29 11:10:05.426',
    '2025-01-29 11:10:05.426',
    '2025-01-29 11:10:05.426',
    '2025-01-29 11:10:05.426',
    '2025-01-29 11:10:05.471',
    '2025-01-29 11:10:05.471',
    '2025-01-29 11:10:05.471',
    '2025-01-29 11:10:05.471',
    '2025-01-29 11:10:05.505',
    '2025-01-29 11:10:05.505',
    '2025-01-29 11:10:05.505',
    '2025-01-29 11:10:05.505',
    '2025-01-29 11:10:05.550',
    '2025-01-29 11:10:05.550',
    '2025-01-29 11:10:05.550',
    '2025-01-29 11:10:05.550',
    '2025-01-29 11:10:05.595',
    '2025-01-29 11:10:05.595',
    '2025-01-29 11:10:05.595',
    '2025-01-29 11:10:05.595',
    '2025-01-29 11:10:05.628',
    '2025-01-29 11:10:05.628',
    '2025-01-29 11:10:05.628',
    '2025-01-29 11:10:05.628',
    '2025-01-29 11:10:05.662',
    '2025-01-29 11:10:05.662',
    '2025-01-29 11:10:05.662',
    '2025-01-29 11:10:05.662',
    '2025-01-29 11:10:05.708',
    '2025-01-29 11:10:05.708',
    '2025-01-29 11:10:05.708',
    '2025-01-29 11:10:05.708',
    '2025-01-29 11:10:05.753',
    '2025-01-29 11:10:05.753',
    '2025-01-29 11:10:05.753',
    '2025-01-29 11:10:05.753',
    '2025-01-29 11:10:05.786',
    '2025-01-29 11:10:05.786',
    '2025-01-29 11:10:05.786',
    '2025-01-29 11:10:05.786',
    '2025-01-29 11:10:05.831',
    '2025-01-29 11:10:05.831',
    '2025-01-29 11:10:05.831',
    '2025-01-29 11:10:05.831',
    '2025-01-29 11:10:05.865',
    '2025-01-29 11:10:05.865',
    '2025-01-29 11:10:05.865',
    '2025-01-29 11:10:05.865',
    '2025-01-29 11:10:05.909',
    '2025-01-29 11:10:05.910',
    '2025-01-29 11:10:05.910',
    '2025-01-29 11:10:05.910',
    '2025-01-29 11:10:05.954',
    '2025-01-29 11:10:05.955',
    '2025-01-29 11:10:05.955',
    '2025-01-29 11:10:05.955',
    '2025-01-29 11:10:05.988',
    '2025-01-29 11:10:05.988',
    '2025-01-29 11:10:05.988',
    '2025-01-29 11:10:05.988',
    '2025-01-29 11:10:06.033',
    '2025-01-29 11:10:06.033',
    '2025-01-29 11:10:06.033',
    '2025-01-29 11:10:06.033',
    '2025-01-29 11:10:06.067',
    '2025-01-29 11:10:06.067',
    '2025-01-29 11:10:06.067',
    '2025-01-29 11:10:06.067',
    '2025-01-29 11:10:06.112',
    '2025-01-29 11:10:06.113',
    '2025-01-29 11:10:06.113',
    '2025-01-29 11:10:06.113',
    '2025-01-29 11:10:06.146',
    '2025-01-29 11:10:06.146',
    '2025-01-29 11:10:06.146',
    '2025-01-29 11:10:06.146',
    '2025-01-29 11:10:06.191',
    '2025-01-29 11:10:06.191',
    '2025-01-29 11:10:06.191',
    '2025-01-29 11:10:06.191',
    '2025-01-29 11:10:06.236',
    '2025-01-29 11:10:06.236',
    '2025-01-29 11:10:06.236',
    '2025-01-29 11:10:06.236',
    '2025-01-29 11:10:06.270',
    '2025-01-29 11:10:06.270',
    '2025-01-29 11:10:06.270',
    '2025-01-29 11:10:06.270',
    '2025-01-29 11:10:06.315',
    '2025-01-29 11:10:06.315',
    '2025-01-29 11:10:06.315',
    '2025-01-29 11:10:06.315',
    '2025-01-29 11:10:06.348',
    '2025-01-29 11:10:06.348',
    '2025-01-29 11:10:06.348',
    '2025-01-29 11:10:06.348',
    '2025-01-29 11:10:06.393',
    '2025-01-29 11:10:06.393',
    '2025-01-29 11:10:06.393',
    '2025-01-29 11:10:06.393',
    '2025-01-29 11:10:06.439',
    '2025-01-29 11:10:06.439',
    '2025-01-29 11:10:06.439',
    '2025-01-29 11:10:06.439',
    '2025-01-29 11:10:06.472',
    '2025-01-29 11:10:06.472',
    '2025-01-29 11:10:06.473',
    '2025-01-29 11:10:06.473',
    '2025-01-29 11:10:06.517',
    '2025-01-29 11:10:06.517',
    '2025-01-29 11:10:06.517',
    '2025-01-29 11:10:06.517',
    '2025-01-29 11:10:06.551',
    '2025-01-29 11:10:06.551',
    '2025-01-29 11:10:06.551',
    '2025-01-29 11:10:06.551',
    '2025-01-29 11:10:06.596',
    '2025-01-29 11:10:06.596',
    '2025-01-29 11:10:06.596',
    '2025-01-29 11:10:06.596',
    '2025-01-29 11:10:06.630',
    '2025-01-29 11:10:06.630',
    '2025-01-29 11:10:06.630',
    '2025-01-29 11:10:06.630',
    '2025-01-29 11:10:06.675',
    '2025-01-29 11:10:06.675',
    '2025-01-29 11:10:06.675',
    '2025-01-29 11:10:06.675',
    '2025-01-29 11:10:06.720',
    '2025-01-29 11:10:06.720',
    '2025-01-29 11:10:06.720',
    '2025-01-29 11:10:06.720',
    '2025-01-29 11:10:06.754',
    '2025-01-29 11:10:06.754',
    '2025-01-29 11:10:06.754',
    '2025-01-29 11:10:06.754',
    '2025-01-29 11:10:06.799',
    '2025-01-29 11:10:06.799',
    '2025-01-29 11:10:06.799',
    '2025-01-29 11:10:06.799',
    '2025-01-29 11:10:06.832',
    '2025-01-29 11:10:06.832',
    '2025-01-29 11:10:06.832',
    '2025-01-29 11:10:06.832',
    '2025-01-29 11:10:06.877',
    '2025-01-29 11:10:06.878',
    '2025-01-29 11:10:06.878',
    '2025-01-29 11:10:06.878',
    '2025-01-29 11:10:06.911',
    '2025-01-29 11:10:06.911',
    '2025-01-29 11:10:06.911',
    '2025-01-29 11:10:06.911',
    '2025-01-29 11:10:06.956',
    '2025-01-29 11:10:06.956',
    '2025-01-29 11:10:06.956',
    '2025-01-29 11:10:06.956',
    '2025-01-29 11:10:07.001',
    '2025-01-29 11:10:07.001',
    '2025-01-29 11:10:07.001',
    '2025-01-29 11:10:07.001',
    '2025-01-29 11:10:07.034',
    '2025-01-29 11:10:07.035',
    '2025-01-29 11:10:07.035',
    '2025-01-29 11:10:07.035',
    '2025-01-29 11:10:07.080',
    '2025-01-29 11:10:07.080',
    '2025-01-29 11:10:07.080',
    '2025-01-29 11:10:07.080',
    '2025-01-29 11:10:07.113',
    '2025-01-29 11:10:07.113',
    '2025-01-29 11:10:07.113',
    '2025-01-29 11:10:07.113',
    '2025-01-29 11:10:07.159',
    '2025-01-29 11:10:07.159',
    '2025-01-29 11:10:07.159',
    '2025-01-29 11:10:07.159',
    '2025-01-29 11:10:07.204',
    '2025-01-29 11:10:07.204',
    '2025-01-29 11:10:07.204',
    '2025-01-29 11:10:07.204',
    '2025-01-29 11:10:07.237',
    '2025-01-29 11:10:07.238',
    '2025-01-29 11:10:07.238',
    '2025-01-29 11:10:07.238',
    '2025-01-29 11:10:07.282',
    '2025-01-29 11:10:07.282',
    '2025-01-29 11:10:07.282',
    '2025-01-29 11:10:07.282',
    '2025-01-29 11:10:07.316',
    '2025-01-29 11:10:07.316',
    '2025-01-29 11:10:07.316',
    '2025-01-29 11:10:07.316',
    '2025-01-29 11:10:07.361',
    '2025-01-29 11:10:07.361',
    '2025-01-29 11:10:07.361',
    '2025-01-29 11:10:07.361',
    '2025-01-29 11:10:07.395'
]

# 将时间戳转换为 datetime 对象
timestamps = [datetime.datetime.strptime(ts, '%Y-%m-%d %H:%M:%S.%f') for ts in timestamps]

# 计算每个包之间的间隔时间（单位：毫秒）
intervals = [(timestamps[i] - timestamps[i - 1]).total_seconds() * 1000 for i in range(1, len(timestamps))]

# 计算间隔的平均值
avg_interval = np.mean(intervals)

# 包序号
packet_numbers = list(range(1, len(intervals) + 1))

# 绘制图表
plt.figure(figsize=(8, 6))
plt.scatter(packet_numbers, intervals, color='blue', marker='o')
plt.axhline(y=avg_interval, color='red', linestyle='--', label=f'Average Interval ({avg_interval:.2f} ms)')

plt.title('Packet Interval Time with Average Line')

plt.title('Packet Interval Time')
plt.xlabel('Packet Number')
plt.ylabel('Time Interval (ms)')
plt.grid(True)
plt.show()
