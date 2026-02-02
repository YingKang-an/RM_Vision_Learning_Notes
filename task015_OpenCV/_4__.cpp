#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// 显示图像信息
void showImageInfo(const cv::Mat& img, const std::string& name) {
    std::cout << name << ": " << img.cols << "×" << img.rows 
              << " (" << img.channels() << " channels)" << std::endl;
}

// 创建颜色测试图像
cv::Mat createColorTestImage(int width, int height) {
    cv::Mat image(height, width, CV_8UC3);
    
    // 创建彩虹渐变
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float ratio = static_cast<float>(x) / width;
            cv::Vec3b color;
            
            if (ratio < 0.2) {
                // 红色到橙色
                float t = ratio / 0.2;
                color = cv::Vec3b(0, 165 * t, 255);  // BGR: (0, 165*t, 255)
            } else if (ratio < 0.4) {
                // 橙色到黄色
                float t = (ratio - 0.2) / 0.2;
                color = cv::Vec3b(0, 165 + 90 * t, 255);  // (0, 165+90*t, 255)
            } else if (ratio < 0.6) {
                // 黄色到绿色
                float t = (ratio - 0.4) / 0.2;
                color = cv::Vec3b(0, 255, 255 - 255 * t);  // (0, 255, 255-255*t)
            } else if (ratio < 0.8) {
                // 绿色到蓝色
                float t = (ratio - 0.6) / 0.2;
                color = cv::Vec3b(255 * t, 255, 0);  // (255*t, 255, 0)
            } else {
                // 蓝色到紫色
                float t = (ratio - 0.8) / 0.2;
                color = cv::Vec3b(255, 255 - 128 * t, 128 * t);  // (255, 255-128*t, 128*t)
            }
            
            // 添加垂直渐变
            float vertical_factor = static_cast<float>(y) / height;
            color = color * (0.5 + 0.5 * vertical_factor);
            
            image.at<cv::Vec3b>(y, x) = color;
        }
    }
    
    // 添加颜色块
    std::vector<cv::Scalar> color_blocks = {
        cv::Scalar(0, 0, 255),     // 红色
        cv::Scalar(0, 255, 0),     // 绿色
        cv::Scalar(255, 0, 0),     // 蓝色
        cv::Scalar(0, 255, 255),   // 黄色
        cv::Scalar(255, 255, 0),   // 青色
        cv::Scalar(255, 0, 255),   // 品红
        cv::Scalar(128, 128, 128), // 灰色
        cv::Scalar(0, 0, 0),       // 黑色
        cv::Scalar(255, 255, 255)  // 白色
    };
    
    int block_size = 40;
    for (size_t i = 0; i < color_blocks.size(); i++) {
        int x = 20 + (i % 3) * (block_size + 10);
        int y = height - 120 + (i / 3) * (block_size + 10);
        cv::rectangle(image, cv::Rect(x, y, block_size, block_size), 
                     color_blocks[i], -1);
        cv::rectangle(image, cv::Rect(x, y, block_size, block_size), 
                     cv::Scalar(255, 255, 255), 1);
    }
    
    return image;
}

int main() {
    std::cout << "=== OpenCV 色彩空间与通道操作 ===\n" << std::endl;
    
    // ============================================
    // 1. 创建测试图像
    // ============================================
    
    std::cout << "1. 创建测试图像..." << std::endl;
    cv::Mat original = createColorTestImage(800, 600);
    showImageInfo(original, "原始图像");
    
    // ============================================
    // 2. 色彩空间转换
    // ============================================
    
    std::cout << "\n2. 色彩空间转换" << std::endl;
    
    // 2.1 RGB/BGR 转灰度
    cv::Mat gray;
    cv::cvtColor(original, gray, cv::COLOR_BGR2GRAY);
    showImageInfo(gray, "   灰度图像");
    
    // 2.2 BGR 转 HSV
    cv::Mat hsv;
    cv::cvtColor(original, hsv, cv::COLOR_BGR2HSV);
    showImageInfo(hsv, "   HSV图像");
    
    // 2.3 BGR 转 LAB
    cv::Mat lab;
    cv::cvtColor(original, lab, cv::COLOR_BGR2Lab);
    showImageInfo(lab, "   Lab图像");
    
    // 2.4 BGR 转 YCrCb
    cv::Mat ycrcb;
    cv::cvtColor(original, ycrcb, cv::COLOR_BGR2YCrCb);
    showImageInfo(ycrcb, "   YCrCb图像");
    
    // 2.5 反向转换
    cv::Mat bgr_from_hsv, bgr_from_lab, bgr_from_ycrcb;
    cv::cvtColor(hsv, bgr_from_hsv, cv::COLOR_HSV2BGR);
    cv::cvtColor(lab, bgr_from_lab, cv::COLOR_Lab2BGR);
    cv::cvtColor(ycrcb, bgr_from_ycrcb, cv::COLOR_YCrCb2BGR);
    
    // ============================================
    // 3. 通道拆分与合并
    // ============================================
    
    std::cout << "\n3. 通道拆分与合并" << std::endl;
    
    // 3.1 拆分BGR通道
    std::vector<cv::Mat> bgr_channels;
    cv::split(original, bgr_channels);
    
    std::cout << "   BGR通道拆分: ";
    std::cout << "B(" << bgr_channels[0].cols << "×" << bgr_channels[0].rows << ") ";
    std::cout << "G(" << bgr_channels[1].cols << "×" << bgr_channels[1].rows << ") ";
    std::cout << "R(" << bgr_channels[2].cols << "×" << bgr_channels[2].rows << ")" << std::endl;
    
    // 3.2 拆分HSV通道
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);
    std::cout << "   HSV通道拆分: H(色调), S(饱和度), V(明度)" << std::endl;
    
    // 3.3 创建单通道图像用于演示
    cv::Mat blue_channel = bgr_channels[0].clone();
    cv::Mat green_channel = bgr_channels[1].clone();
    cv::Mat red_channel = bgr_channels[2].clone();
    
    // 3.4 通道合并
    std::vector<cv::Mat> custom_channels;
    custom_channels.push_back(blue_channel);      // B通道
    custom_channels.push_back(cv::Mat::zeros(original.size(), CV_8UC1));  // G通道设为0
    custom_channels.push_back(cv::Mat::zeros(original.size(), CV_8UC1));  // R通道设为0
    
    cv::Mat blue_only;
    cv::merge(custom_channels, blue_only);
    
    std::cout << "   创建只显示蓝色通道的图像" << std::endl;
    
    // 3.5 交换通道顺序（BGR转RGB）
    std::vector<cv::Mat> rgb_channels;
    rgb_channels.push_back(bgr_channels[2]);  // R
    rgb_channels.push_back(bgr_channels[1]);  // G
    rgb_channels.push_back(bgr_channels[0]);  // B
    
    cv::Mat rgb_image;
    cv::merge(rgb_channels, rgb_image);
    
    // ============================================
    // 4. HSV颜色识别（提取红色区域）
    // ============================================
    
    std::cout << "\n4. HSV颜色识别：提取红色区域" << std::endl;
    
    // 4.1 定义红色范围（HSV空间）
    // 注意：OpenCV中H范围是0-180（不是0-360）
    cv::Mat red_mask1, red_mask2, red_mask;
    
    // 红色在HSV中跨越0度和180度（因为色调是圆形）
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), red_mask1);     // 红色低范围
    cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), red_mask2);  // 红色高范围
    
    // 合并两个范围
    cv::bitwise_or(red_mask1, red_mask2, red_mask);
    
    // 4.2 形态学操作优化掩膜
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);  // 闭运算填充空洞
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);   // 开运算去除噪声
    
    // 4.3 应用掩膜提取红色区域
    cv::Mat red_extracted;
    original.copyTo(red_extracted, red_mask);
    
    std::cout << "   红色区域提取完成" << std::endl;
    
    // 4.4 其他颜色提取示例
    cv::Mat green_mask, blue_mask, yellow_mask;
    
    // 绿色范围
    cv::inRange(hsv, cv::Scalar(35, 70, 50), cv::Scalar(85, 255, 255), green_mask);
    
    // 蓝色范围
    cv::inRange(hsv, cv::Scalar(100, 70, 50), cv::Scalar(130, 255, 255), blue_mask);
    
    // 黄色范围
    cv::inRange(hsv, cv::Scalar(20, 70, 50), cv::Scalar(35, 255, 255), yellow_mask);
    
    // ============================================
    // 5. 通道操作高级应用
    // ============================================
    
    std::cout << "\n5. 通道操作高级应用" << std::endl;
    
    // 5.1 修改特定通道
    cv::Mat enhanced_red = original.clone();
    std::vector<cv::Mat> enhanced_channels;
    cv::split(enhanced_red, enhanced_channels);
    
    // 增强红色通道（提高对比度）
    cv::Mat red_enhanced;
    cv::equalizeHist(enhanced_channels[2], red_enhanced);
    enhanced_channels[2] = red_enhanced;
    
    cv::merge(enhanced_channels, enhanced_red);
    
    // 5.2 创建伪彩色图像（将灰度图映射为彩色）
    cv::Mat pseudocolor;
    cv::applyColorMap(gray, pseudocolor, cv::COLORMAP_JET);
    
    // 5.3 通道混合（创建特殊效果）
    cv::Mat special_effect;
    std::vector<cv::Mat> effect_channels;
    
    // 使用不同通道组合
    effect_channels.push_back(bgr_channels[2]);  // 红色通道作为蓝色
    effect_channels.push_back(bgr_channels[0]);  // 蓝色通道作为绿色
    effect_channels.push_back(bgr_channels[1]);  // 绿色通道作为红色
    
    cv::merge(effect_channels, special_effect);
    
    std::cout << "   通道混合效果创建完成" << std::endl;
    
    // 5.4 计算通道统计信息
    cv::Scalar mean_bgr, stddev_bgr;
    cv::meanStdDev(original, mean_bgr, stddev_bgr);
    
    std::cout << "\n   BGR通道统计:" << std::endl;
    std::cout << "     均值: B=" << mean_bgr[0] << ", G=" << mean_bgr[1] << ", R=" << mean_bgr[2] << std::endl;
    std::cout << "     标准差: B=" << stddev_bgr[0] << ", G=" << stddev_bgr[1] << ", R=" << stddev_bgr[2] << std::endl;
    
    // 5.5 通道直方图（简化显示）
    std::cout << "\n   通道直方图统计:" << std::endl;
    
    for (int i = 0; i < 3; i++) {
        // 计算直方图
        int histSize = 256;
        float range[] = {0, 256};
        const float* histRange = {range};
        
        cv::Mat hist;
        cv::calcHist(&bgr_channels[i], 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
        
        // 找到最大值
        double min_val, max_val;
        cv::minMaxLoc(hist, &min_val, &max_val);
        
        // 简化为10个区间显示
        std::cout << "     通道" << i << ": ";
        int bin_step = histSize / 10;
        for (int j = 0; j < 10; j++) {
            int bin_start = j * bin_step;
            int bin_end = (j + 1) * bin_step;
            float sum = 0;
            
            for (int k = bin_start; k < bin_end && k < histSize; k++) {
                sum += hist.at<float>(k);
            }
            
            // 显示相对频率
            int stars = static_cast<int>(sum / max_val * 20);
            std::cout << std::string(stars, '*');
            if (j < 9) std::cout << "|";
        }
        std::cout << std::endl;
    }
    
    // ============================================
    // 6. 创建显示面板
    // ============================================
    
    std::cout << "\n6. 创建结果显示面板..." << std::endl;
    
    int panel_width = 1600;
    int panel_height = 1000;
    cv::Mat display_panel(panel_height, panel_width, CV_8UC3, cv::Scalar(40, 40, 40));
    
    // 6.1 定义布局
    int thumb_width = 380;
    int thumb_height = 285;
    
    // 6.2 准备所有图像
    struct DisplayItem {
        cv::Mat image;
        std::string label;
        cv::Scalar text_color;
    };
    
    std::vector<DisplayItem> display_items = {
        {original, "原始图像 (BGR)", cv::Scalar(255, 255, 255)},
        {gray, "灰度图像", cv::Scalar(255, 255, 255)},
        {hsv, "HSV色彩空间", cv::Scalar(255, 255, 255)},
        {lab, "Lab色彩空间", cv::Scalar(255, 255, 255)},
        {bgr_from_hsv, "BGR (从HSV转换)", cv::Scalar(255, 255, 255)},
        {red_extracted, "红色区域提取", cv::Scalar(0, 0, 255)},
        {enhanced_red, "增强红色通道", cv::Scalar(255, 255, 255)},
        {pseudocolor, "伪彩色 (JET)", cv::Scalar(255, 255, 255)},
        {special_effect, "通道混合效果", cv::Scalar(255, 255, 255)}
    };
    
    // 6.3 添加通道分离图像
    std::vector<cv::Mat> channel_display;
    for (int i = 0; i < 3; i++) {
        cv::Mat channel_color;
        std::vector<cv::Mat> single_channel(3, cv::Mat::zeros(original.size(), CV_8UC1));
        single_channel[i] = bgr_channels[i];
        cv::merge(single_channel, channel_color);
        
        std::string channel_name;
        cv::Scalar text_color;
        
        switch(i) {
            case 0: channel_name = "蓝色通道"; text_color = cv::Scalar(255, 0, 0); break;
            case 1: channel_name = "绿色通道"; text_color = cv::Scalar(0, 255, 0); break;
            case 2: channel_name = "红色通道"; text_color = cv::Scalar(0, 0, 255); break;
        }
        
        display_items.push_back({channel_color, channel_name, text_color});
    }
    
    // 6.4 添加HSV通道图像
    std::vector<std::string> hsv_names = {"色调 (H)", "饱和度 (S)", "明度 (V)"};
    for (int i = 0; i < 3; i++) {
        // 将单通道转换为伪彩色以便观察
        cv::Mat pseudocolor_channel;
        cv::applyColorMap(hsv_channels[i], pseudocolor_channel, cv::COLORMAP_JET);
        
        display_items.push_back({pseudocolor_channel, hsv_names[i], cv::Scalar(255, 255, 255)});
    }
    
    // 6.5 排列图像
    int cols = 4;
    for (size_t i = 0; i < display_items.size(); i++) {
        int row = i / cols;
        int col = i % cols;
        
        int x = col * (thumb_width + 20) + 20;
        int y = row * (thumb_height + 40) + 40;
        
        // 检查边界
        if (y + thumb_height >= panel_height) {
            // 创建第二个面板
            panel_height += thumb_height + 100;
            cv::Mat new_panel(panel_height, panel_width, CV_8UC3, cv::Scalar(40, 40, 40));
            display_panel.copyTo(new_panel(cv::Rect(0, 0, display_panel.cols, display_panel.rows)));
            display_panel = new_panel;
        }
        
        // 调整图像尺寸
        cv::Mat thumb;
        cv::resize(display_items[i].image, thumb, cv::Size(thumb_width, thumb_height));
        
        // 复制到面板
        cv::Rect roi(x, y, thumb_width, thumb_height);
        thumb.copyTo(display_panel(roi));
        
        // 添加边框
        cv::rectangle(display_panel, roi, cv::Scalar(180, 180, 180), 2);
        
        // 添加标签
        cv::putText(display_panel, display_items[i].label,
                   cv::Point(x + 10, y + thumb_height + 25),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, display_items[i].text_color, 1);
        
        // 添加序号
        cv::putText(display_panel, std::to_string(i + 1),
                   cv::Point(x + 10, y + 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    
    // 6.6 添加标题和说明
    cv::putText(display_panel, "OpenCV 色彩空间与通道操作演示",
               cv::Point(panel_width / 2 - 250, 30),
               cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 255), 2);
    
    // 添加图例
    cv::putText(display_panel, "色彩空间: BGR, HSV, Lab, YCrCb",
               cv::Point(20, panel_height - 60),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    
    cv::putText(display_panel, "操作: 转换, 拆分, 合并, 颜色提取",
               cv::Point(20, panel_height - 40),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    
    cv::putText(display_panel, "按ESC退出, 按's'保存结果",
               cv::Point(panel_width - 300, panel_height - 20),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    
    // ============================================
    // 7. 交互演示：颜色选择器
    // ============================================
    
    std::cout << "\n7. 交互演示：HSV颜色选择器" << std::endl;
    
    // 创建颜色选择器窗口
    cv::namedWindow("颜色选择器", cv::WINDOW_NORMAL);
    cv::resizeWindow("颜色选择器", 600, 400);
    
    // 创建控制参数
    int h_min = 0, h_max = 180;
    int s_min = 0, s_max = 255;
    int v_min = 0, v_max = 255;
    
    // 创建控制面板
    cv::namedWindow("控制面板", cv::WINDOW_NORMAL);
    cv::resizeWindow("控制面板", 400, 500);
    
    // 创建滑动条回调函数
    auto updateColorPicker = [&]() {
        cv::Mat control_panel(500, 400, CV_8UC3, cv::Scalar(50, 50, 50));
        
        // 标题
        cv::putText(control_panel, "=== HSV颜色选择器 ===",
                   cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX,
                   0.7, cv::Scalar(0, 255, 255), 2);
        
        // 显示当前范围
        cv::putText(control_panel, "H: [" + std::to_string(h_min) + ", " + std::to_string(h_max) + "]",
                   cv::Point(20, 70), cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(200, 200, 200), 1);
        
        cv::putText(control_panel, "S: [" + std::to_string(s_min) + ", " + std::to_string(s_max) + "]",
                   cv::Point(20, 100), cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(200, 200, 200), 1);
        
        cv::putText(control_panel, "V: [" + std::to_string(v_min) + ", " + std::to_string(v_max) + "]",
                   cv::Point(20, 130), cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(200, 200, 200), 1);
        
        // 创建滑动条显示
        int bar_width = 360;
        int bar_height = 20;
        int bar_x = 20;
        
        // H滑动条
        cv::rectangle(control_panel, cv::Rect(bar_x, 160, bar_width, bar_height),
                     cv::Scalar(100, 100, 100), -1);
        int h_pos = bar_x + (h_min * bar_width / 180);
        int h_width = (h_max - h_min) * bar_width / 180;
        cv::rectangle(control_panel, cv::Rect(h_pos, 160, h_width, bar_height),
                     cv::Scalar(0, 255, 0), -1);
        cv::putText(control_panel, "Hue (色调)", cv::Point(bar_x, 155),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // S滑动条
        cv::rectangle(control_panel, cv::Rect(bar_x, 200, bar_width, bar_height),
                     cv::Scalar(100, 100, 100), -1);
        int s_pos = bar_x + (s_min * bar_width / 255);
        int s_width = (s_max - s_min) * bar_width / 255;
        cv::rectangle(control_panel, cv::Rect(s_pos, 200, s_width, bar_height),
                     cv::Scalar(0, 255, 0), -1);
        cv::putText(control_panel, "Saturation (饱和度)", cv::Point(bar_x, 195),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // V滑动条
        cv::rectangle(control_panel, cv::Rect(bar_x, 240, bar_width, bar_height),
                     cv::Scalar(100, 100, 100), -1);
        int v_pos = bar_x + (v_min * bar_width / 255);
        int v_width = (v_max - v_min) * bar_width / 255;
        cv::rectangle(control_panel, cv::Rect(v_pos, 240, v_width, bar_height),
                     cv::Scalar(0, 255, 0), -1);
        cv::putText(control_panel, "Value (明度)", cv::Point(bar_x, 235),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // 操作说明
        cv::putText(control_panel, "按键控制:", cv::Point(20, 290),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        
        std::vector<std::string> instructions = {
            "Q/W: 调整H最小值",
            "A/S: 调整H最大值",
            "E/R: 调整S最小值",
            "D/F: 调整S最大值",
            "T/Y: 调整V最小值",
            "G/H: 调整V最大值",
            "空格: 重置所有参数",
            "ESC: 退出程序"
        };
        
        for (size_t i = 0; i < instructions.size(); i++) {
            cv::putText(control_panel, instructions[i],
                       cv::Point(30, 320 + i * 25),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        }
        
        // 显示当前选择的颜色
        cv::Mat color_sample(50, 200, CV_8UC3);
        
        // 计算HSV中间值
        cv::Scalar hsv_color(
            (h_min + h_max) / 2,
            (s_min + s_max) / 2,
            (v_min + v_max) / 2
        );
        
        // 转换为BGR显示
        cv::Mat hsv_mat(1, 1, CV_8UC3, hsv_color);
        cv::Mat bgr_mat;
        cv::cvtColor(hsv_mat, bgr_mat, cv::COLOR_HSV2BGR);
        cv::Vec3b bgr_color = bgr_mat.at<cv::Vec3b>(0, 0);
        
        color_sample.setTo(cv::Scalar(bgr_color[0], bgr_color[1], bgr_color[2]));
        
        // 添加到控制面板
        cv::Rect color_rect(100, 420, 200, 50);
        color_sample.copyTo(control_panel(color_rect));
        cv::rectangle(control_panel, color_rect, cv::Scalar(255, 255, 255), 2);
        
        cv::putText(control_panel, "当前颜色",
                   cv::Point(20, 445), cv::FONT_HERSHEY_SIMPLEX,
                   0.6, cv::Scalar(200, 200, 200), 1);
        
        cv::imshow("控制面板", control_panel);
        
        // 更新颜色选择器显示
        cv::Mat color_mask;
        cv::inRange(hsv, cv::Scalar(h_min, s_min, v_min), 
                   cv::Scalar(h_max, s_max, v_max), color_mask);
        
        cv::Mat color_result;
        original.copyTo(color_result, color_mask);
        
        // 添加轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) > 100) {  // 只显示面积大于100的轮廓
                cv::drawContours(color_result, std::vector<std::vector<cv::Point>>{contour}, 
                               0, cv::Scalar(255, 255, 255), 2);
            }
        }
        
        // 添加信息
        cv::putText(color_result, "HSV范围: [" + std::to_string(h_min) + "," + std::to_string(h_max) + "]",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(color_result, "按ESC退出选择器",
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        cv::imshow("颜色选择器", color_result);
    };
    
    // 初始显示
    updateColorPicker();
    
    // 主循环
    while (true) {
        int key = cv::waitKey(0);
        
        if (key == 27) {  // ESC
            break;
        } else if (key == 'q' || key == 'Q') {
            h_min = std::max(0, h_min - 5);
            updateColorPicker();
        } else if (key == 'w' || key == 'W') {
            h_min = std::min(h_max - 1, h_min + 5);
            updateColorPicker();
        } else if (key == 'a' || key == 'A') {
            h_max = std::max(h_min + 1, h_max - 5);
            updateColorPicker();
        } else if (key == 's' || key == 'S') {
            h_max = std::min(180, h_max + 5);
            updateColorPicker();
        } else if (key == 'e' || key == 'E') {
            s_min = std::max(0, s_min - 5);
            updateColorPicker();
        } else if (key == 'r' || key == 'R') {
            s_min = std::min(s_max - 1, s_min + 5);
            updateColorPicker();
        } else if (key == 'd' || key == 'D') {
            s_max = std::max(s_min + 1, s_max - 5);
            updateColorPicker();
        } else if (key == 'f' || key == 'F') {
            s_max = std::min(255, s_max + 5);
            updateColorPicker();
        } else if (key == 't' || key == 'T') {
            v_min = std::max(0, v_min - 5);
            updateColorPicker();
        } else if (key == 'y' || key == 'Y') {
            v_min = std::min(v_max - 1, v_min + 5);
            updateColorPicker();
        } else if (key == 'g' || key == 'G') {
            v_max = std::max(v_min + 1, v_max - 5);
            updateColorPicker();
        } else if (key == 'h' || key == 'H') {
            v_max = std::min(255, v_max + 5);
            updateColorPicker();
        } else if (key == ' ') {
            // 重置
            h_min = 0; h_max = 180;
            s_min = 0; s_max = 255;
            v_min = 0; v_max = 255;
            updateColorPicker();
        } else if (key == '1') {
            // 红色范围
            h_min = 0; h_max = 10;
            s_min = 100; s_max = 255;
            v_min = 50; v_max = 255;
            updateColorPicker();
        } else if (key == '2') {
            // 绿色范围
            h_min = 35; h_max = 85;
            s_min = 100; s_max = 255;
            v_min = 50; v_max = 255;
            updateColorPicker();
        } else if (key == '3') {
            // 蓝色范围
            h_min = 100; h_max = 130;
            s_min = 100; s_max = 255;
            v_min = 50; v_max = 255;
            updateColorPicker();
        } else if (key == 's' || key == 'S') {
            // 保存当前结果
            cv::imwrite("color_space_demo.jpg", display_panel);
            std::cout << "已保存结果到 color_space_demo.jpg" << std::endl;
        }
    }
    
    cv::destroyAllWindows();
    
    std::cout << "\n=== 色彩空间与通道操作演示完成 ===" << std::endl;
    std::cout << "总结：" << std::endl;
    std::cout << "1. 掌握了色彩空间转换（BGR↔灰度↔HSV↔Lab）" << std::endl;
    std::cout << "2. 学会了通道拆分、合并和单通道处理" << std::endl;
    std::cout << "3. 实现了HSV颜色识别和提取" << std::endl;
    std::cout << "4. 了解了不同色彩空间的特点和应用场景" << std::endl;
    
    return 0;
}