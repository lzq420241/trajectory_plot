import math


def rpy_to_quaternion(roll, pitch, yaw):
    # 转换角度为弧度
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    # 计算四元数分量
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # 四元数的分量计算 (使用Z-Y-X顺序)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw


# 输入你提供的RPY角度
roll = -36.29  # 滚转角
pitch = -91.03  # 俯仰角
yaw = 15.61  # 偏航角

qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
print("四元数: qx={}, qy={}, qz={}, qw={}".format(qx, qy, qz, qw))

