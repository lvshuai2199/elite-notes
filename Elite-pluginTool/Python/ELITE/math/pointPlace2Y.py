import numpy as np

def is_in_positive_y(point, ucs_origin, ucs_y_axis):
    """
    point: 待判断的点 [x, y, z]
    ucs_origin: 用户坐标系原点 [x, y, z]
    ucs_y_axis: 用户坐标系Y轴的方向向量 (无需归一化，函数内部会处理)
    """
    # 1. 构造向量
    v = np.array(point) - np.array(ucs_origin)
    
    # 2. 归一化Y轴向量
    y_unit = np.array(ucs_y_axis) / np.linalg.norm(ucs_y_axis)
    
    # 3. 计算点积 (即点在Y轴上的投影长度)
    y_coordinate = np.dot(v, y_unit)
    
    return y_coordinate > 0

# 示例
origin = [0, 0, 0]
y_dir = [0, 1, 0] # 假设Y轴就是世界坐标系的Y轴
p1 = [1, 5, 1]    # 在正向
p2 = [1, -2, 1]   # 在负向

print(is_in_positive_y(p1, origin, y_dir)) # 输出: True
print(is_in_positive_y(p2, origin, y_dir)) # 输出: False
