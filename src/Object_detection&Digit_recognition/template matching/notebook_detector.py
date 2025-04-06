import os
import cv2
import glob
import numpy as np

def apply_perspective_transform(image, mode='left', offset=20):
    """
    对图像进行透视变换：
      mode: 'left'、'front' 或 'right'
      offset: 透视变换的偏移量
    """
    h, w = image.shape[:2]
    src = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
    
    if mode == 'left':
        dst = np.float32([
            [0 + offset, 0],
            [w - 1, 10],
            [w - 1, h - 10],
            [0 + offset, h - 1]
        ])
    elif mode == 'right':
        dst = np.float32([
            [0, 10],
            [w - 1 - offset, 0],
            [w - 1 - offset, h - 1],
            [0, h - 10]
        ])
    else:  # 'front'：不做变换
        dst = src.copy()
    
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(image, M, (w, h))
    return warped

class DigitRecognizer:
    def __init__(self):
        # 存储每个数字对应的扩增模板列表，键为字符串形式的数字
        self.templates = {}
    
    def load_templates(self, folder_path):
        """
        加载模板图片，并进行透视变换扩增，
        每个模板生成 'left'、'front'、'right' 三个版本。
        模板文件命名为 template_1.png ~ template_9.png
        """
        for i in range(1, 10):
            template_filename = f"template_{i}.png"
            template_path = os.path.join(folder_path, template_filename)
            template = cv2.imread(template_path)
            if template is not None:
                augmented_templates = []
                for mode in ['left', 'front', 'right']:
                    aug = apply_perspective_transform(template, mode=mode, offset=20)
                    augmented_templates.append(aug)
                self.templates[str(i)] = augmented_templates
                print(f"模板 {i} 加载成功：{template_path}，生成 {len(augmented_templates)} 个变换版本")
            else:
                print(f"无法加载模板: {template_path}")
    
    def recognize_digit(self, cv_image):
        """
        利用模板匹配实现数字识别：
          - 转换输入图像为灰度图并直方图均衡化
          - 遍历每个数字的所有扩增模板，在不同尺度下进行匹配，选取最高匹配得分
          - 若得分超过阈值，则返回 (digit, pixel_offset)
            其中 pixel_offset 表示匹配区域中心与图像中心水平方向的偏移
          - 否则返回 None
        """
        if not self.templates:
            print("没有加载模板，无法进行数字识别！")
            return None
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        
        best_score = -1
        best_digit = None
        best_loc = None
        best_template_shape = None
        
        # 遍历每个数字及其扩增模板
        for digit, template_list in self.templates.items():
            for template in template_list:
                # 保证模板为灰度图并直方图均衡化
                if len(template.shape) == 3:
                    template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
                else:
                    template_gray = template.copy()
                template_gray = cv2.equalizeHist(template_gray)
                
                # 在多个尺度上进行匹配
                for scale in np.linspace(0.5, 1.5, 21):
                    resized_template = cv2.resize(template_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                    tH, tW = resized_template.shape[:2]
                    if gray.shape[0] < tH or gray.shape[1] < tW:
                        continue
                    
                    res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                    if max_val > best_score:
                        best_score = max_val
                        best_digit = digit
                        best_loc = max_loc
                        best_template_shape = resized_template.shape
        
        threshold = 0.7
        if best_score < threshold or best_digit is None:
            print("模板匹配未达到阈值，未检测到有效数字。最高分: {:.2f}".format(best_score))
            return None
        
        tH, tW = best_template_shape
        match_center_x = best_loc[0] + tW / 2
        image_center_x = gray.shape[1] / 2
        pixel_offset = match_center_x - image_center_x
        
        print("模板匹配成功：数字 {}, 匹配分数: {:.2f}, 像素偏移: {:.2f}".format(best_digit, best_score, pixel_offset))
        return best_digit, pixel_offset

def recognize_digit_with_bbox(recognizer, cv_image):
    """
    包装 recognize_digit 方法，返回识别数字及匹配区域信息
    返回: (digit, pixel_offset, top_left, bottom_right)
      - top_left: 匹配区域左上角坐标
      - bottom_right: 匹配区域右下角坐标
    如果未检测到有效数字，则返回 None
    """
    if not recognizer.templates:
        print("没有加载模板，无法进行数字识别！")
        return None
    
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    
    best_score = -1
    best_digit = None
    best_loc = None
    best_template_shape = None
    
    for digit, template_list in recognizer.templates.items():
        for template in template_list:
            if len(template.shape) == 3:
                template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            else:
                template_gray = template.copy()
            template_gray = cv2.equalizeHist(template_gray)
            
            for scale in np.linspace(0.4, 1.6, 21):
                resized_template = cv2.resize(template_gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                tH, tW = resized_template.shape[:2]
                if gray.shape[0] < tH or gray.shape[1] < tW:
                    continue
                
                res = cv2.matchTemplate(gray, resized_template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                if max_val > best_score:
                    best_score = max_val
                    best_digit = digit
                    best_loc = max_loc
                    best_template_shape = resized_template.shape
    
    threshold = 0.7
    if best_score < threshold or best_digit is None:
        print("模板匹配未达到阈值，未检测到有效数字。最高分: {:.2f}".format(best_score))
        return None
    
    tH, tW = best_template_shape
    match_center_x = best_loc[0] + tW / 2
    image_center_x = gray.shape[1] / 2
    pixel_offset = match_center_x - image_center_x
    
    top_left = best_loc
    bottom_right = (best_loc[0] + tW, best_loc[1] + tH)
    
    print("模板匹配成功：数字 {}, 匹配分数: {:.2f}, 像素偏移: {:.2f}".format(best_digit, best_score, pixel_offset))
    return best_digit, pixel_offset, top_left, bottom_right

def annotate_and_save_images(recognizer, input_folder, output_folder):
    """
    处理输入文件夹中的所有图片，
    调用 recognize_digit_with_bbox 进行数字识别，
    在识别区域画出边框并标注识别到的数字，
    然后保存到输出文件夹。
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    image_files = sorted(glob.glob(os.path.join(input_folder, "image_*.png")))
    print("找到 {} 张图片".format(len(image_files)))
    
    for img_path in image_files:
        img = cv2.imread(img_path)
        if img is None:
            print("无法读取图片: {}".format(img_path))
            continue
        
        result = recognize_digit_with_bbox(recognizer, img)
        if result is not None:
            digit, pixel_offset, top_left, bottom_right = result
            cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 2)
            cv2.putText(img, str(digit), (top_left[0], max(top_left[1]-10, 10)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        else:
            cv2.putText(img, "No Digit", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (0, 0, 255), 2)
        
        filename = os.path.basename(img_path)
        output_path = os.path.join(output_folder, filename)
        cv2.imwrite(output_path, img)
        print("保存标注图片到: {}".format(output_path))

if __name__ == "__main__":
    # 获取当前工作目录（适用于 Jupyter 或脚本环境）
    script_dir = os.getcwd()
    template_folder = os.path.join(script_dir, "t")
    input_folder = os.path.join(script_dir, "images2")
    output_folder = os.path.join(script_dir, "output11")
    
    recognizer = DigitRecognizer()
    recognizer.load_templates(template_folder)
    
    annotate_and_save_images(recognizer, input_folder, output_folder)
