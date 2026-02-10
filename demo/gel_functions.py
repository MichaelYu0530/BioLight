import cv2
import numpy as np

# 初始化BGR相同的三通道图像
def initialize_img(height, width):
    max_bg_gray = 40 # 背景区域最大灰度值
    
    # 创建灰度值有一定波动的单通道
    gray_channel = np.random.randint(0, max_bg_gray, (height, width), np.uint8)
    # 扩展为BGR相同的三通道
    img = cv2.merge([gray_channel, gray_channel, gray_channel])
    
    return img

# 改变图像中指定区域内的灰度值为固定值或者区间内的随机值
def change_area_gray_value(img, x1, x2, y1, y2, value = -1, interval = [-1, -1]):
    # 确保坐标顺序正确
    x1, x2 = sorted([x1, x2])
    y1, y2 = sorted([y1, y2])
    # 避免处理区域为一列或一行的情况时的报错
    x2, y2 = x2 + 1, y2 + 1

    region = img[y1: y2, x1: x2] # 截取出目标区域
    
    if value >= 0: # 改变区域内灰度值为固定值
        region[:, :, 0] = value
        region[:, :, 1] = value
        region[:, :, 2] = value
        
    if len(interval) == 2 and interval[0] >= 0 and interval[1] >= 0: # 改变区域内灰度值为区间内随机数
        min_gray, max_gray = sorted(interval)
        gray_values = np.random.randint(min_gray, max_gray, (y2 - y1, x2 - x1), dtype=np.uint8)
        region[:, :, 0] = gray_values
        region[:, :, 1] = gray_values
        region[:, :, 2] = gray_values

# 用径向渐变填充条带内部，中心点坐标需输入，中心最亮，边缘渐暗，参数坐标格式均为(x, y)
def fill_in_band(img, center = (0, 0), point_list = [], border_list = [], internal_list = []):
    if len(point_list) > 0: # 输入了顶点列表，要处理的是四边形条带
        if len(point_list) == 2: # 输入的是矩形的左上、右下顶点，否则为一般四边形顺时针顺序的四个顶点
            pt1, pt3 = point_list
            pt2, pt4 = (pt3[0], pt1[1]), (pt1[0], pt3[1])
            point_list = [pt1, pt2, pt3, pt4] # 得出矩形顺时针顺序的四个顶点
            
        # 创建四边形掩码
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        pts = np.array(point_list, np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(mask, [pts], 255)
        
        # 找到四边形内所有点
        y_indices, x_indices = np.where(mask == 255)
        
        # 计算中心点到各点的距离
        cx, cy = center
        distances = np.sqrt((x_indices - cx)**2 + (y_indices - cy)**2)
        
        # 归一化距离 (0-1)
        max_dist = distances.max()
        norm_distances = distances / max_dist
        
        # 根据距离计算灰度值 (中心255，边缘不低于60)
        gray_values = 255 - norm_distances * (255 - 60)  # 线性递减
        gray_values = np.clip(gray_values, 60, 254).astype(np.uint8)  # [60, 254]
        
        # 6. 中心点设为255
        if 0 <= cy < img.shape[0] and 0 <= cx < img.shape[1]:
            if mask[cy, cx] == 255:  # 确保中心点在四边形内
                gray_values[distances == 0] = 255
        
        # 填充到图像
        for i in range(len(y_indices)):
            y, x = y_indices[i], x_indices[i]
            gray = gray_values[i]
            img[y, x] = (gray, gray, gray)

    if len(border_list) > 0 and len(internal_list) > 0: # 输入了边界点与内部点列表，要处理的是不规则图形条带
        # 分离x, y坐标
        internal_array = np.array(internal_list)
        x_indices = internal_array[:, 0]
        y_indices = internal_array[:, 1]
        
        # 计算每个内部点到中心的距离
        cx, cy = center
        distances = np.sqrt((x_indices - cx)**2 + (y_indices - cy)**2)
        
        # 归一化距离
        max_dist = distances.max()
        norm_distances = distances / max_dist
        
        # 计算灰度值（中心亮，边缘暗）
        gray_values = 255 - norm_distances * (255 - 60)
        gray_values = np.clip(gray_values, 60, 254).astype(np.uint8)
        
        # 中心点最亮（找到最近的点）
        min_dist_idx = np.argmin(distances)
        gray_values[min_dist_idx] = 255
        
        # 填充到图像
        for i in range(len(y_indices)):
            y, x = y_indices[i], x_indices[i]
            gray = gray_values[i]
            img[y, x] = (gray, gray, gray)

# 为矩形或一般四边形条带添加锯齿边缘
def add_jagged_border(img, point_list, is_few_bands = True):
    # 若图像中条带数较少，则可将锯齿边缘延长得更多些
    max_extend, corner_extend = 12, 6
    if is_few_bands == True:
        max_extend, corner_extend = 20, 10
    min_extend = max_extend // 3
    
    min_gray_value, max_gray_value = 60, 120 # 锯齿边缘的灰度值范围
    interval = [min_gray_value, max_gray_value]
    
    if len(point_list) == 2: # 输入的是矩形的左上、右下顶点
        corner1_x, corner1_y = point_list[0]
        corner2_x, corner2_y = point_list[1]
        
        # 为条带上侧边添加锯齿边缘
        y = corner1_y
        for x in range(corner1_x, corner2_x):
            if (x - corner1_x) % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
            change_area_gray_value(img, x, x, y - extend, y, interval=interval)
        # 为条带下侧边添加锯齿边缘
        y = corner2_y
        for x in range(corner1_x, corner2_x):
            if (x - corner1_x) % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
            change_area_gray_value(img, x, x, y + extend, y, interval=interval)
        # 为条带左侧边添加锯齿边缘
        x = corner1_x
        for y in range(corner1_y, corner2_y):
            if (y - corner1_y) % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
            change_area_gray_value(img, x - extend, x, y, y, interval=interval)
        # 为条带右侧边添加锯齿边缘
        x = corner2_x
        for y in range(corner1_y, corner2_y):
            if (y - corner1_y) % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
            change_area_gray_value(img, x + extend, x, y, y, interval=interval)
        
        # 处理左上顶点
        change_area_gray_value(img, 
            corner1_x - corner_extend, corner1_x, corner1_y - corner_extend, corner1_y, interval=interval)
        # 处理左下顶点
        change_area_gray_value(img, 
            corner1_x - corner_extend, corner1_x, corner2_y + corner_extend, corner2_y, interval=interval)
        # 处理右上顶点
        change_area_gray_value(img, 
            corner2_x + corner_extend, corner2_x, corner1_y - corner_extend, corner1_y, interval=interval)
        # 处理右下顶点
        change_area_gray_value(img, 
            corner2_x + corner_extend, corner2_x, corner2_y + corner_extend, corner2_y, interval=interval)
    
    if len(point_list) == 4: # 输入的是一般四边形的四个顶点，左上顶点开始沿顺时针顺序排列
        pt1, pt2, pt3, pt4 = point_list
        
        temp = point_list + [pt1] # 尾接pt1便于后续循环
        for i in range(len(point_list)):
            # 求斜率与纵坐标截距
            x1, y1 = temp[i]
            x2, y2 = temp[i + 1]
            k = (y2 - y1) / (x2 - x1)
            b = y1 - k * x1
            
            border_list = [] # 储存在两顶点之间线段上的点（即条带边界点）的列表 
            # 为条带上侧边添加锯齿边缘
            if i == 0:
                # 处理相对水平线时遍历x得到y，int() + 1向上取整保证点在条带内部
                border_list = [(x, int(k * x + b) + 1) for x in range(min(x1, x2), max(x1, x2))]
                for j in range(len(border_list)):
                    x, y = border_list[j]
                    if j % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                        extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
                    change_area_gray_value(img, x, x, y - extend, y, interval=interval)
            # 为条带下侧边添加锯齿边缘
            if i == 2:
                # 处理相对水平线时遍历x得到y，int()向下取整保证点在条带内部
                border_list = [(x, int(k * x + b)) for x in range(min(x1, x2), max(x1, x2))]
                for j in range(len(border_list)):
                    x, y = border_list[j]
                    if j % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                        extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
                    change_area_gray_value(img, x, x, y + extend, y, interval=interval)
            # 为条带右侧边添加锯齿边缘
            if i == 1:
                # 处理相对竖直线时遍历y得到x，int()向下取整保证点在条带内部
                border_list = [(int((y - b) / k), y) for y in range(min(y1, y2), max(y1, y2))] 
                for j in range(len(border_list)):
                    x, y = border_list[j]
                    if j % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                        extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
                    change_area_gray_value(img, x + extend, x, y, y, interval=interval)
            # 为条带左侧边添加锯齿边缘
            if i == 3:
                # 处理相对竖直线时遍历y得到x，int() + 1向上取整保证点在条带内部
                border_list = [(int((y - b) / k) + 1, y) for y in range(min(y1, y2), max(y1, y2))]
                for j in range(len(border_list)):
                    x, y = border_list[j]
                    if j % 5 == 0: # 每5个像素点改变一次锯齿长度，减少尖锐感
                        extend = np.random.randint(min_extend, max_extend) # 向外延伸像素点数
                    change_area_gray_value(img, x - extend, x, y, y, interval=interval)
        
        # 处理左上顶点
        change_area_gray_value(img, 
            pt1[0] - corner_extend, pt1[0], pt1[1] - corner_extend, pt1[1], interval=interval)
        # 处理右上顶点
        change_area_gray_value(img, 
            pt2[0] + corner_extend, pt2[0], pt2[1] - corner_extend, pt2[1], interval=interval)
        # 处理右下顶点
        change_area_gray_value(img, 
            pt3[0] + corner_extend, pt3[0], pt3[1] + corner_extend, pt3[1], interval=interval)
        # 处理左下顶点
        change_area_gray_value(img, 
            pt4[0] - corner_extend, pt4[0], pt4[1] + corner_extend, pt4[1], interval=interval)

# 构建理想情况下的矩形条带，含内部渐变灰度值，锯齿边缘
def build_regular_bands(img, rect_band_corner1_list, rect_band_corner2_list, is_few_bands = True):
    center_list = []
    for i in range(len(rect_band_corner1_list)):
        corner1 = rect_band_corner1_list[i]
        corner2 = rect_band_corner2_list[i]
        point_list = [corner1, corner2]
        corner1_x, corner1_y = corner1
        corner2_x, corner2_y = corner2
        width, height = corner2_x - corner1_x, corner2_y - corner1_y
        
        center_x, center_y = (corner2_x + corner1_x) // 2, (corner2_y  + corner1_y) // 2 # 几何中心点
        multiplier_x, multiplier_y = np.random.uniform(-0.25, 0.25), np.random.uniform(-0.25, 0.25)
        center_x, center_y = int(center_x + width * multiplier_x), int(center_y + height * multiplier_y) # 在条带的中心区域随机定位一个中心点
        center = (center_x, center_y)
        center_list.append(center)
        
        fill_in_band(img, center=center, point_list=point_list)
        add_jagged_border(img, point_list=point_list, is_few_bands=is_few_bands)
    
    if len(center_list) == 1:
        return center_list[0]
    return center_list

# 对含有多个条带的原图画出检测矩形框，条带内部及其锯齿边缘的灰度值不会低于60
def detect_rect_bands(img, center_list, is_guide_line = False):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # 三通道转换为单通道
    
    detect_corner1_list = []
    detect_corner2_list = []
    for i in range(len(center_list)):
        center = center_list[i]
        center_x, center_y = center
        
        # 检测条带上界
        j = 0
        while gray_img[center_y - j - 1, center_x] >= 60:
            j = j + 1
        detect_corner1_y = center_y - j
        # 检测条带下界
        k = 0
        while gray_img[center_y + k + 1, center_x] >= 60:
            k = k + 1
        detect_corner2_y = center_y + k
        # 检测条带左界
        m = 0
        while gray_img[center_y, center_x - m - 1] >= 60:
            m = m + 1
        detect_corner1_x = center_x - m
        # 检测条带右界
        n = 0
        while gray_img[center_y, center_x + n + 1] >= 60:
            n = n + 1
        detect_corner2_x = center_x + n
        
        # 画出指示检测过程的辅助线
        if is_guide_line == True:
            guide_pt1 = (center_x, detect_corner1_y + 5) # 加减5是为了结果图更好看
            guide_pt2 = (detect_corner2_x - 5, center_y)
            guide_pt3 = (center_x, detect_corner2_y - 5)
            guide_pt4 = (detect_corner1_x + 5, center_y)
            
            cv2.arrowedLine(img, center, guide_pt1, (200, 255, 50), 2, tipLength=0.2)
            cv2.arrowedLine(img, center, guide_pt2, (200, 255, 50), 2, tipLength=0.05)
            cv2.arrowedLine(img, center, guide_pt3, (200, 255, 50), 2, tipLength=0.2)
            cv2.arrowedLine(img, center, guide_pt4, (200, 255, 50), 2, tipLength=0.1)
        
        detect_corner1 = (detect_corner1_x, detect_corner1_y)
        detect_corner2 = (detect_corner2_x, detect_corner2_y)
        detect_corner1_list.append(detect_corner1)
        detect_corner2_list.append(detect_corner2)
        
        cv2.rectangle(img, detect_corner1, detect_corner2, (0, 0, 255), 2)
    
    if len(detect_corner1_list) == 1 and len(detect_corner2_list) == 1:
        return detect_corner1_list[0], detect_corner2_list[0]
    return detect_corner1_list, detect_corner2_list

# 依据同泳道内条带的同侧边中点，做线性拟合得到泳道检测线
def fit_lanes(img, detect_corner1_list, detect_corner2_list):
    mid_point1_list = [] # 存储条带左侧边中点的列表
    mid_point2_list = [] # 存储条带右侧边中点的列表
    for i in range(len(detect_corner1_list)):
        corner1_x, corner1_y = detect_corner1_list[i]
        corner2_x, corner2_y = detect_corner2_list[i]
        
        mid_point1 = (corner1_x, (corner1_y + corner2_y) // 2) # 左侧边中点，格式为(x, y)
        mid_point2 = (corner2_x, (corner1_y + corner2_y) // 2) # 右侧边中点，格式为(x, y)
        mid_point1_list.append(mid_point1)
        mid_point2_list.append(mid_point2)

        # 画出两侧边中点，以天蓝色填充
        cv2.circle(img, mid_point1, 4, (255, 191, 0), -1)
        cv2.circle(img, mid_point2, 4, (255, 191, 0), -1)

    # 计算条带纵坐标上下界
    min_y_list = detect_corner1_list[0][1], detect_corner1_list[5][1], detect_corner1_list[7][1]
    max_y_list = detect_corner2_list[4][1], detect_corner2_list[6][1], detect_corner2_list[13][1]
    min_y_list = np.array(min_y_list + min_y_list)
    max_y_list = np.array(max_y_list + max_y_list)
    extend = 15 # 泳道检测线将超出泳道内条带上下界15像素点
    min_y_list = min_y_list - extend # 得到泳道检测线的纵坐标下界
    max_y_list = max_y_list + extend # 得到泳道检测线的纵坐标上界

    # 按5/2/7的从左到右泳道内条带数量划分原列表
    split_points = [5, 5 + 2]
    mid_point_split = np.split(mid_point1_list, split_points) + np.split(mid_point2_list, split_points)

    for i in range(len(mid_point_split)):
        mid_point_list = mid_point_split[i]
        x = mid_point_list[:, 0]
        y = mid_point_list[:, 1]
        k, b = np.polyfit(x, y, 1) # 线性拟合得拟合直线的斜率k、截距b
        
        min_y, max_y = min_y_list[i], max_y_list[i]
        min_x = int((min_y - b) / k) # 泳道检测线最小纵坐标处的横坐标，并非最小横坐标
        max_x = int((max_y - b) / k) # 泳道检测线最大纵坐标处的横坐标，并非最大横坐标
        
        cv2.line(img, (min_x, min_y), (max_x, max_y), (255, 191, 0), 2)

# 构建两个非常规的条带（一般四边形和不规则图形），含内部渐变灰度值
def build_irregular_bands(img, center_pair, point_list, is_few_bands = True):
    # 填充一般四边形条带的内部灰度值
    poly_center = center_pair[0] # 一般四边形条带的中心点
    fill_in_band(img, center=poly_center, point_list=point_list)
    add_jagged_border(img, point_list, is_few_bands=is_few_bands)
    
    # 填充不规则图形条带的内部灰度值
    tooth_center = center_pair[1] # 不规则图形（牙齿状）条带的中心点
    border_list = detect_irregular_bands(is_from_real_gel=True)
    internal_list = get_internal_points(border_list)
    fill_in_band(img, center=tooth_center, border_list=border_list, internal_list=internal_list)

# 提取非矩形条带的有序边界点
def detect_irregular_bands(is_from_real_gel = False, img = None):
    if is_from_real_gel == True: # 是从真实的凝胶电泳图像“1.tif”中提取条带边界点，返回有序边界点列表
        # 读取图像为单通道
        path = "1.tif"
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        img = img[0: 100, 10: 110] # 裁剪原图，方便后续坐标变换
        
        threshold = 200 # 设置阈值，不妨多次尝试
        
        # 得到无序的边界点列表
        border_list = get_border_points(img, threshold)
        # 对边界点进行排序，形成连续轮廓
        border_list = sort_border_points(border_list)
        # 把全部边界点、内部点从原图移至展示图的简单坐标变换
        border_list = [(x * 4, y * 3 + 300) for x, y in border_list]
        
        return border_list
    
    else: # 是从模拟的凝胶电泳图像中提取条带边界点并绘制边界，无返回值
        threshold = 60 # 设置阈值，条带内部与锯齿边缘的灰度值不低于60
        
        # 在高度上对半分原图，得到只含一般四边形条带的img1和只含不规则图形条带的img2
        img1 = img[:300, :, :]
        img2 = img[300:, :, :]
        
        # 得到一般四边形条带的无序的边界点列表
        border_list1 = get_border_points(img1, threshold)
        # 对一般四边形条带的边界点进行排序，形成连续轮廓
        border_list1 = sort_border_points(border_list1)
        # 得到不规则图形条带的无序的边界点列表
        border_list2 = get_border_points(img2, threshold)
        # 对不规则图形条带的边界点进行排序，形成连续轮廓
        border_list2 = sort_border_points(border_list2)
        # 恢复分裂原图导致的坐标偏移
        border_list2 = [(x, y + 300) for x, y in border_list2] 
        
        for pt in border_list1:
            cv2.circle(img, pt, 2, (0, 0, 255, 1))
        for pt in border_list2:
            cv2.circle(img, pt, 2, (0, 0, 255, 1))

# 求出全部不规则图形条带的边界点，尚未排序
def get_border_points(img, threshold):
    if len(img.shape) == 3: # 若输入三通道图像则需转换为单通道图像
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray_img = img.copy()
    
    # 创建内部掩码
    internal_mask = (gray_img >= threshold).astype(np.uint8) * 255
    
    # 获取内部点的坐标
    internal_y_list, internal_x_list = np.where(internal_mask == 255)
    internal_list = list(zip(internal_x_list, internal_y_list))
    internal_set = set(internal_list) # 将内部点转换为集合以便快速查找
    
    # 检测边界点：上下左右至少有一个方向不是内部点
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # 定义四个方向的偏移量
    border_list = []
    for point in internal_list:
        x, y = point
        
        # 检查上下左右是否至少有一个方向没有内部点
        has_external_neighbor = False
        for dx, dy in directions:
            neighbor = (x + dx * 1, y + dy * 1)
            if neighbor not in internal_set:
                has_external_neighbor = True
                break
        
        if has_external_neighbor:
            border_list.append(point)
    
    return border_list

# 对边界点进行排序，形成连续轮廓
def sort_border_points(points):
    # 找到最左边的点作为起点
    start_point = min(points, key=lambda p: p[0])
    
    # 使用最近邻方法排序
    sorted_points = [start_point]
    remaining_points = set(points)
    remaining_points.remove(start_point)
    current_point = start_point
    max_iterations = len(points) * 2
    iteration = 0
    
    # 找到最近的点
    while remaining_points and iteration < max_iterations:
        nearest_point = None
        min_distance = float('inf')
        for point in remaining_points:
            dx = point[0] - current_point[0]
            dy = point[1] - current_point[1]
            dist = dx * dx + dy * dy  # 使用平方距离避免开方
            
            if dist < min_distance and dist > 0:
                min_distance = dist
                nearest_point = point
        
        if nearest_point:
            sorted_points.append(nearest_point)
            remaining_points.remove(nearest_point)
            current_point = nearest_point
        
        iteration += 1
    
    # 添加剩余的点
    sorted_points.extend(list(remaining_points))
    
    return sorted_points

# 由不规则图形条带的边界点列表计算其所有内部点
def get_internal_points(border_list):
    # 创建掩码
    border_array = np.array(border_list)
    min_x, min_y = border_array.min(axis=0)
    max_x, max_y = border_array.max(axis=0)
    
    mask = np.zeros((max_y - min_y + 1, max_x - min_x + 1), dtype=np.uint8)
    
    # 调整坐标并填充
    adjusted = border_array - [min_x, min_y]
    cv2.fillPoly(mask, [adjusted.reshape(-1, 1, 2)], 255)
    
    # 获取内部点
    y_indices, x_indices = np.where(mask == 255)
    internal_list = [(x + min_x, y + min_y) for x, y in zip(x_indices, y_indices)]
    
    return internal_list
