import cv2
import numpy as np
from gel_functions import *

def main():
    # 基础设置
    height, width = 250, 400
    img1 = np.random.randint(0, 40, (height, width, 3), np.uint8) # 存在噪声的黑色背景
    
    rect_point_list = [(80, 80), (350, 175)] # 条带参数，矩形左上、右下顶点
    
    # 绘制原图，只含一个矩形条带
    center = build_regular_bands(img1, [rect_point_list[0]], [rect_point_list[1]])
    cv2.imwrite("images/01_single_band_original.png", img1)
    # 画出中心点至边界的辅助线与条带检测框
    detect_rect_bands(img1, [center], is_guide_line=True)
    cv2.imwrite("images/02_single_band_detected.png", img1)
    
    # 基础设置
    height, width = 600, 600
    img2 = np.random.randint(0, 40, (height, width, 3), np.uint8) # 存在噪声的黑色背景

    # 条带参数
    # 3个泳道中分别有5/2/7个条带，共14个
    # 矩形条带左上顶点列表，格式是(x, y)
    rect_band_corner1_list = [
        (20,  50), (24, 150),  (30, 220),  (32, 275),  (40, 340),  
        (240, 60), (245, 175), 
        (455, 55), (460, 140), (450, 195), (435, 270), (440, 350), (425, 420), (420, 470)]
    # 矩形条带右下顶点列表，格式是(x, y)
    rect_band_corner2_list = [
        (160, 90),  (167, 175), (175, 250), (172, 305), (180, 370), 
        (360, 130), (370, 210), 
        (575, 80),  (565, 165), (565, 225), (550, 300), (555, 370), (535, 445), (545, 540)]
    
    # 构造3个泳道内共14个矩形条带
    center_list = build_regular_bands(img2, rect_band_corner1_list, rect_band_corner2_list)
    cv2.imwrite("images/03_multi_lanes_original.png", img2)
    # 画出条带检测框和泳道检测线
    detect_corner1_list, detect_corner2_list = detect_rect_bands(img2, center_list)
    cv2.imwrite("images/04_multi_bands_detected.png", img2)
    draw_lanes(img2, detect_corner1_list, detect_corner2_list)
    cv2.imwrite("images/05_lanes_reconstructed.png", img2)
    
    # 基础设置
    height, width = 600, 400
    img3 = np.random.randint(0, 40, (height, width, 3), np.uint8) # 存在噪声的黑色背景
    
    poly_point_list = [(90, 65), (335, 115), (355, 250), (75, 195)] # 条带参数，一般四边形条带的四个顶点
    center_pair = [(215, 125), (150, 450)] # 条带参数，一般四边形条带与不规则图形条带的中心点
    
    # 构造一般四边形与不规则图形的条带
    build_irregular_bands(img3, center_pair, poly_point_list)
    img4 = img3.copy() # 创建原图的副本
    cv2.imwrite("images/06_irregular_original.png", img3)
    # 展示阶段1的检测方法
    detect_rect_bands(img3, center_pair, is_guide_line=True)
    cv2.imwrite("images/07_phase1_rect_detection.png", img3)
    #展示阶段2的检测方法
    detect_irregular_bands(is_from_real_gel=False, img=img4)
    cv2.imwrite("images/08_phase2_contour_detection.png", img4)
    
    cv2.waitKey(0)

if __name__ == "__main__":
    main()