#include "getimg.hpp"

getimg::getimg():it_(nh_), window_name_("Armor Detection"),frame_count_(0),last_print_time_(ros::Time::now())
{
    // 首先，初始化参数
    initParams();
    // 使用TransportHints在代码中指定压缩传输
    image_transport::TransportHints hints("compressed");

    // 订阅相机图像和参数
    cam_sub_ = it_.subscribeCamera("/hk_camera/image_raw", 10, &getimg::imageCallback, this, hints);
    // // 创建OpenCV显示窗口 初期的图像显示，现在移动到rqt中
    // cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
    // cv::moveWindow(window_name_, 100, 100); // 设置窗口位置

    // 初始化Publisher
    pub_result_image_ = it_.advertise("/armor_detection/result_image", 1);
    pub_binary_image_ = it_.advertise("/armor_detection/binary_image", 1);

    ROS_INFO("Armor Detection Node initialized");
    ROS_INFO("Subscribing to /hk_camera/image_raw with compressed transport");
    ROS_INFO("Detect color: %s", (enemy_color_ == 0)?"RED":((enemy_color_ == 1)?"BULE":"TEST"));
    ROS_INFO("Waiting for camera data...");
}

getimg::~getimg()
{
    // 原来的图像移动到rqt中，不需要再销毁了
    // cv::destroyWindow(window_name_);
    ROS_INFO("Armor Detection closed");
}

void getimg::initParams()
{
    // 红色阈值
    red_thresh_.hue_min = nh_.param("threshold/red/hue_min", 0);
    red_thresh_.hue_max = nh_.param("threshold/red/hue_max", 10);
    red_thresh_.sat_min = nh_.param("threshold/red/sat_min", 100);
    red_thresh_.sat_max = nh_.param("threshold/red/sat_max", 255);
    red_thresh_.val_min = nh_.param("threshold/red/val_min", 50);
    red_thresh_.val_max = nh_.param("threshold/red/val_max", 255);
    
    // 蓝色阈值
    blue_thresh_.hue_min = nh_.param("threshold/blue/hue_min", 100);
    blue_thresh_.hue_max = nh_.param("threshold/blue/hue_max", 130);
    blue_thresh_.sat_min = nh_.param("threshold/blue/sat_min", 100);
    blue_thresh_.sat_max = nh_.param("threshold/blue/sat_max", 255);
    blue_thresh_.val_min = nh_.param("threshold/blue/val_min", 50);
    blue_thresh_.val_max = nh_.param("threshold/blue/val_max", 255);
    
    // 测试阈值
    test_thresh_.hue_min = nh_.param("threshold/test/hue_min", 10);
    test_thresh_.hue_max = nh_.param("threshold/test/hue_max", 25);
    test_thresh_.sat_min = nh_.param("threshold/test/sat_min", 150);
    test_thresh_.sat_max = nh_.param("threshold/test/sat_max", 255);
    test_thresh_.val_min = nh_.param("threshold/test/val_min", 180);
    test_thresh_.val_max = nh_.param("threshold/test/val_max", 255);
    
    // 敌方颜色
    enemy_color_ = nh_.param("enemy_color", 0);
    
    // 灯条参数
    light_params_.min_area = nh_.param("light/min_area", 50.0);
    light_params_.max_ratio = nh_.param("light/max_ratio", 0.4);
    light_params_.min_solidity = nh_.param("light/min_solidity", 0.7);
    light_params_.min_height = nh_.param("light/min_height", 20.0);
    
    // 配对参数
    pair_params_.light_max_angle_diff = nh_.param("pair/light_max_angle_diff", 15.0);
    pair_params_.light_max_height_diff_ratio = nh_.param("pair/light_max_height_diff_ratio", 0.5);
    pair_params_.light_max_y_diff_ratio = nh_.param("pair/light_max_y_diff_ratio", 0.7);
    pair_params_.light_min_x_diff_ratio = nh_.param("pair/light_min_x_diff_ratio", 0.5);
    pair_params_.armor_min_aspect_ratio = nh_.param("pair/armor_min_aspect_ratio", 0.8);
    pair_params_.armor_max_aspect_ratio = nh_.param("pair/armor_max_aspect_ratio", 4.5);
    pair_params_.armor_big_armor_ratio = nh_.param("pair/armor_big_armor_ratio", 3.2);
    pair_params_.armor_small_armor_ratio = nh_.param("pair/armor_small_armor_ratio", 2.0);

    // 数字检测参数
    digit_params_.white_threshold = nh_.param("digit/white_threshold", 200);
    digit_params_.min_white_ratio = nh_.param("digit/min_white_ratio", 0.15);
    
    // 预处理参数
    preprocess_params_.brightness_thresh = nh_.param("preprocess/brightness_thresh", 150);
    
    // 形态学参数
    morph_open_size_ = nh_.param("morphology/open_size", 3);
    morph_close_size_ = nh_.param("morphology/close_size", 5);
}

std::string getimg::getColorName(int color_id)
{
    switch(color_id) {
        case 0: return "RED";
        case 1: return "BLUE";
        case 2: return "TEST";
        default: return "UNKNOWN";
    }
}

void getimg::imageCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    frame_count_++;
    ros::Time current_time = ros::Time::now();
    
    // 每秒打印一次帧率信息
    if ((current_time - last_print_time_).toSec() >= 1.0) 
    {
        ROS_INFO("Frame rate: %d Hz", frame_count_);
        frame_count_ = 0;
        last_print_time_ = current_time;
    }
    
    // 1. 转换图像为OpenCV格式
    cv::Mat image;
    if (!convertImage(img_msg, image)) 
    {
        ROS_ERROR("Failed to convert image");
        return;
    }
    
    // 2. 检测装甲板
    std::vector<ArmorDescriptor> detected_armors = detectArmor(image);
    
    // 3. 绘制检测结果
    cv::Mat result_image = image.clone();
    drawDetections(result_image, detected_armors);
    
    // // 4. 显示结果 在rqt中显示
    // cv::imshow(window_name_, display_image);
    // cv::waitKey(1);

    // 4. 将结果发送到rqt中显示（发布topic）
    sensor_msgs::ImagePtr result_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).toImageMsg();
    pub_result_image_.publish(result_msg);
    // 附加：二值化图像
    cv::Mat binary_color;
    if (binary_image_.channels() == 1) 
    {
        cv::cvtColor(binary_image_, binary_color, cv::COLOR_GRAY2BGR);
    } 
    else 
    {
        binary_color = binary_image_;
    }
    sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", binary_color).toImageMsg();
    pub_binary_image_.publish(binary_msg);
    
    // 5. 打印检测信息（每秒一次）
    static int print_counter = 0;
    print_counter++;
    if (print_counter % 30 == 0 && !detected_armors.empty()) 
    {
        printDetectionInfo(detected_armors);
    }
}

bool getimg::convertImage(const sensor_msgs::ImageConstPtr& img_msg, cv::Mat& output_image)
{
    try 
    {
        cv_bridge::CvImagePtr cv_ptr;
        // 转换为BGR8格式
        if (img_msg->encoding == sensor_msgs::image_encodings::BGR8) 
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            output_image = cv_ptr->image;
            return true;
        } 
        else if (img_msg->encoding == sensor_msgs::image_encodings::RGB8) 
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
            cv::cvtColor(cv_ptr->image, output_image, cv::COLOR_RGB2BGR);
            return true;
        }
        else if (img_msg->encoding == sensor_msgs::image_encodings::MONO8) 
        {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
            cv::cvtColor(cv_ptr->image, output_image, cv::COLOR_GRAY2BGR);
            return true;
        }
        else 
        {
            // 尝试使用原始编码
            cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
            ROS_WARN("Unusual encoding: %s", img_msg->encoding.c_str());
            output_image = cv_ptr->image;
            return true;
        }
        
    } catch (cv_bridge::Exception& e) 
    {
        ROS_ERROR("Image conversion error: %s", e.what());
        return false;
    } catch (std::exception& e) 
    {
        ROS_ERROR("Processing error: %s", e.what());
        return false;
    }
}

void getimg::adjustRect(cv::RotatedRect& rect)
{
    // 确保宽度是较小的边（灯条宽度）
    if (rect.size.width > rect.size.height)
    {
        std::swap(rect.size.width, rect.size.height);
        rect.angle += 90.0f;
    }
    
    // 标准化角度到[-90, 90)度
    if (rect.angle < -90.0) rect.angle += 180.0;
    else if (rect.angle > 90.0) rect.angle -= 180.0;
}

cv::Mat getimg::colorSegmentation(const cv::Mat& hsv_image) 
{
    cv::Mat mask;
    
    if (enemy_color_ == 0) 
    { 
        // 检测红色 - 两个范围
        cv::Mat mask1, mask2;
        cv::inRange(hsv_image, 
                   cv::Scalar(red_thresh_.hue_min, red_thresh_.sat_min, red_thresh_.val_min), 
                   cv::Scalar(red_thresh_.hue_max, red_thresh_.sat_max, red_thresh_.val_max), 
                   mask1);
        cv::inRange(hsv_image, 
                   cv::Scalar(170, red_thresh_.sat_min, red_thresh_.val_min), 
                   cv::Scalar(180, red_thresh_.sat_max, red_thresh_.val_max), 
                   mask2);
        mask = mask1 | mask2;
    } 
    else if (enemy_color_ == 1) 
    { 
        // 检测蓝色
        cv::inRange(hsv_image, 
                   cv::Scalar(blue_thresh_.hue_min, blue_thresh_.sat_min, blue_thresh_.val_min), 
                   cv::Scalar(blue_thresh_.hue_max, blue_thresh_.sat_max, blue_thresh_.val_max), 
                   mask);
    }
    else if (enemy_color_ == 2) 
    { 
        // 检测测试颜色
        cv::inRange(hsv_image, 
                   cv::Scalar(test_thresh_.hue_min, test_thresh_.sat_min, test_thresh_.val_min), 
                   cv::Scalar(test_thresh_.hue_max, test_thresh_.sat_max, test_thresh_.val_max), 
                   mask);
    }
    
    // 形态学操作：去除噪声
    if (morph_open_size_ > 0) {
        cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_RECT, 
                                                       cv::Size(morph_open_size_, morph_open_size_));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_open);
    }
    
    if (morph_close_size_ > 0) {
        cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, 
                                                        cv::Size(morph_close_size_, morph_close_size_));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_close);
    }
    
    return mask;
}

std::vector<getimg::ArmorDescriptor> getimg::detectArmor(const cv::Mat& image) 
{
    std::vector<ArmorDescriptor> final_armors;
    std::vector<LightDescriptor> light_infos;
    
    // --- 方法1: 使用通道相减法（来自CSDN）---
    if (enemy_color_ == 0 || enemy_color_ == 1) 
    {
        // 红色装甲板：红色通道减蓝色通道，蓝色相反
        std::vector<cv::Mat> channels;
        cv::split(image, channels);
        cv::Mat gray_img = (enemy_color_ == 0)?(channels.at(2) - channels.at(0)):(channels.at(0) - channels.at(2));
        
        // 二值化 - 尝试过动态阈值，和预期不一样，画面有点诡异
        cv::Mat bin_img;
        cv::threshold(gray_img, bin_img, preprocess_params_.brightness_thresh, 255, cv::THRESH_BINARY);
        
        // 形态学操作
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::dilate(bin_img, bin_img, element);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> light_contours;
        cv::findContours(bin_img.clone(), light_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 筛选灯条轮廓
        for (const auto& contour : light_contours)
        {
            float light_contour_area = cv::contourArea(contour);
            if (light_contour_area < light_params_.min_area) continue;
            
            // min矩形拟合得到旋转矩形 - 原来的是fitEllipse，出现了灯条缺失的情况(不能完整地识别出所有灯条)
            cv::RotatedRect light_rec = cv::minAreaRect(contour);
            adjustRect(light_rec);
            
            // 宽高比筛选
            float ratio = light_rec.size.width / light_rec.size.height;
            if (ratio > light_params_.max_ratio) continue;
            
            // 凸度筛选
            float solidity = light_contour_area / (light_rec.size.width * light_rec.size.height);
            if (solidity < light_params_.min_solidity) continue;
            
            // 高度筛选
            if (light_rec.size.height < light_params_.min_height) continue;
            
            // 保存灯条
            light_infos.push_back(LightDescriptor(light_rec));
        }

        // 保存二值化图像 P3必做
        binary_image_ = bin_img.clone();
    }
    else
    {
        // --- 方法2: 使用HSV颜色分割（其他颜色）---
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        cv::Mat color_mask = colorSegmentation(hsv_image);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> light_contours;
        cv::findContours(color_mask.clone(), light_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // 筛选灯条轮廓
        for (const auto& contour : light_contours)
        {
            float light_contour_area = cv::contourArea(contour);
            if (light_contour_area < light_params_.min_area) continue;
            
            cv::RotatedRect light_rec = cv::fitEllipse(contour);
            adjustRect(light_rec);
            
            // 宽高比筛选
            float ratio = light_rec.size.width / light_rec.size.height;
            if (ratio > light_params_.max_ratio) continue;
            
            // 凸度筛选
            float solidity = light_contour_area / (light_rec.size.width * light_rec.size.height);
            if (solidity < light_params_.min_solidity) continue;
            
            // 高度筛选
            if (light_rec.size.height < light_params_.min_height) continue;
            
            light_infos.push_back(LightDescriptor(light_rec));
        }

        // 保存二值化图像
        binary_image_ = color_mask.clone();
    }
    
    // 如果没有找到足够灯条，直接返回
    if (light_infos.size() < 2) 
    {
        return final_armors;
    }
    
    // 按灯条中心x坐标排序
    std::sort(light_infos.begin(), light_infos.end(), 
              [](const LightDescriptor& ld1, const LightDescriptor& ld2) 
              {return ld1.center.x < ld2.center.x;});
    
    // 灯条配对
    for (size_t i = 0; i < light_infos.size(); i++) 
    {
        for (size_t j = i + 1; j < light_infos.size(); j++) 
        {
            const LightDescriptor& left_light = light_infos[i];
            const LightDescriptor& right_light = light_infos[j];
 
            // 条件1: 角度差（平行度）
            float angle_diff = std::abs(left_light.angle - right_light.angle);
            if (angle_diff > pair_params_.light_max_angle_diff) continue;
            
            // 条件2: 长度差比率
            float len_diff_ratio = std::abs(left_light.length - right_light.length) / 
                                   std::max(left_light.length, right_light.length);
            if (len_diff_ratio > pair_params_.light_max_height_diff_ratio) continue;
            
            // 条件3: y方向距离
            float y_diff = std::abs(left_light.center.y - right_light.center.y);
            float y_diff_ratio = y_diff / std::max(left_light.length, right_light.length);
            if (y_diff_ratio > pair_params_.light_max_y_diff_ratio) continue;
            
            // 条件4: x方向距离（不应太近）
            float x_diff = std::abs(left_light.center.x - right_light.center.x);
            float x_diff_ratio = x_diff / std::max(left_light.length, right_light.length);
            if (x_diff_ratio < pair_params_.light_min_x_diff_ratio) continue;
            
            // 条件5: 装甲板长宽比
            float distance = cv::norm(left_light.center - right_light.center);
            float mean_len = (left_light.length + right_light.length) / 2;
            float aspect_ratio = distance / mean_len;
            
            if (aspect_ratio < pair_params_.armor_min_aspect_ratio || 
                aspect_ratio > pair_params_.armor_max_aspect_ratio) continue;

            // 确定装甲板类型（大小装甲板）
            int armor_type = (aspect_ratio > pair_params_.armor_big_armor_ratio) ? 1 : 0;

            // 添加到最终结果
            final_armors.push_back(ArmorDescriptor(left_light, right_light, armor_type));
        }
    }
    return final_armors;
}


void getimg::drawDetections(cv::Mat& image, const std::vector<ArmorDescriptor>& armors) 
{
    // 绘制检测结果
    for (size_t i = 0; i < armors.size(); i++) 
    {
        const ArmorDescriptor& armor = armors[i];
        
        // 1. 画出左右灯条（紫色）
        cv::ellipse(image, armor.left_light.rect, cv::Scalar(255, 0, 255), 2);
        cv::ellipse(image, armor.right_light.rect, cv::Scalar(255, 0, 255), 2);
        
        // 2. 画出整个装甲板的外接矩形（绿色）
        cv::rectangle(image, armor.bounding_rect, cv::Scalar(0, 255, 0), 3);
        
        // 3. 连接左右灯条的中心点（黄色）
        cv::line(image, armor.left_light.center, armor.right_light.center, 
                cv::Scalar(0, 255, 255), 2);
        
        // 4. 绘制装甲板中心点（红色）
        cv::Point armor_center(
            armor.bounding_rect.x + armor.bounding_rect.width / 2,
            armor.bounding_rect.y + armor.bounding_rect.height / 2
        );
        cv::circle(image, armor_center, 5, cv::Scalar(0, 0, 255), -1);
        
        // 5. 添加标签
        std::string label = "Armor " + std::to_string(i) + 
                           (armor.type == 1 ? " (BIG)" : " (SMALL)");
        cv::putText(image, label, 
                   cv::Point(armor.bounding_rect.x, armor.bounding_rect.y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
    
    // 在图像左上角添加统计信息
    std::string info_text = "Detected: " + std::to_string(armors.size()) + " armors";
    cv::putText(image, info_text, 
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    
    // 添加颜色信息
    std::string color_text = "Target: " + getColorName(enemy_color_);
    cv::Scalar text_color;
    if (enemy_color_ == 0) text_color = cv::Scalar(0, 0, 255);
    else if (enemy_color_ == 1) text_color = cv::Scalar(255, 0, 0);
    else text_color = cv::Scalar(0, 165, 255);
    
    cv::putText(image, color_text,
                cv::Point(10, 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);
}

void getimg::printDetectionInfo(const std::vector<ArmorDescriptor>& armors) 
{
    ROS_INFO("=== Armor Detection Results ===");
    ROS_INFO("Target Color: %s", getColorName(enemy_color_).c_str());
    ROS_INFO("Detected %zu armor(s)", armors.size());
    
    for (size_t i = 0; i < armors.size(); i++) 
    {
        const ArmorDescriptor& armor = armors[i];
        cv::Rect rect = armor.bounding_rect;
        ROS_INFO("Armor %zu: Position(%d, %d), Size(%dx%d), Type: %s", 
                 i, rect.x, rect.y, rect.width, rect.height,
                 (armor.type == 1) ? "BIG" : "SMALL");
    }
    
    ROS_INFO("----------------------------------------");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "armor_detection_node");
    
    // 设置ROS日志级别
    if (ros::console::set_logger_level
        (ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ROS_INFO("Starting Armor Detector Node");
    ROS_INFO("check at rqt: result_image/binary_image");
    
    getimg node;
    
    ros::spin();
    
    ROS_INFO("Armor Detector Node shutting down");
    return 0;
}