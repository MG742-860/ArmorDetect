#pragma once
#ifndef _GET_IMG_
#define _GET_IMG_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

// 用于识别装甲板
class getimg
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber cam_sub_;
    
    // 显示窗口名称
    std::string window_name_;
    
    // 统计变量
    int frame_count_;
    ros::Time last_print_time_;
    
    // 颜色阈值参数
    struct ColorThreshold {
        int hue_min, hue_max;
        int sat_min, sat_max;
        int val_min, val_max;
    } red_thresh_, blue_thresh_, test_thresh_;
    
    // 识别到的颜色 - 对面(敌方， 0-红色， 1-蓝色， 2-测试)
    int enemy_color_;
    
    // 装甲板参数
    struct ArmorParams {
        double min_area, max_area;
        double min_aspect_ratio, max_aspect_ratio;
        double min_contour_area;
        bool has_number;
    } armor_params_;
    
    // 形态学操作参数
    int morph_open_size_;
    int morph_close_size_;
    
    // 灯条参数
    struct LightParams {
        double min_area;
        double max_ratio; // 宽高比（宽/高）
        double min_solidity; // 凸度
        double min_height;
    } light_params_;
    
    // 配对参数
    struct PairParams {
        double light_max_angle_diff;
        double light_max_height_diff_ratio;
        double light_max_y_diff_ratio;
        double light_min_x_diff_ratio;
        double armor_min_aspect_ratio;
        double armor_max_aspect_ratio;
        double armor_big_armor_ratio;
        double armor_small_armor_ratio;
    } pair_params_;
    
    // 预处理参数
    struct PreprocessParams {
        int brightness_thresh;
    } preprocess_params_;
    
    // 灯条描述结构体
    struct LightDescriptor {
        cv::RotatedRect rect;  // 旋转矩形
        float length;          // 灯条长度（较长的一边）
        cv::Point2f center;    // 中心点
        float angle;           // 角度（标准化到[-90, 90)度）
        
        LightDescriptor(const cv::RotatedRect& light_rect) {
            rect = light_rect;
            length = std::max(light_rect.size.width, light_rect.size.height);
            center = light_rect.center;
            
            // 标准化角度到[-90, 90)度
            angle = light_rect.angle;
            if (angle < -90.0) angle += 180.0;
            else if (angle > 90.0) angle -= 180.0;
        }
    };
    
    // 装甲板描述结构体
    struct ArmorDescriptor {
        LightDescriptor left_light;
        LightDescriptor right_light;
        cv::Rect bounding_rect;
        int type; // 0:小装甲板，1:大装甲板
        ArmorDescriptor(const LightDescriptor& l, const LightDescriptor& r, int armor_type) 
            : left_light(l), right_light(r), type(armor_type) {
            cv::Rect left_rect = l.rect.boundingRect();
            cv::Rect right_rect = r.rect.boundingRect();
            bounding_rect = left_rect | right_rect;
        }
    };

    // 数字检测相关参数
    struct DigitCheckParams {
        int white_threshold;          // 白色像素阈值
        float min_white_ratio;        // 最小白色像素比例
    } digit_params_;

    // 图像发布器
    image_transport::Publisher pub_result_image_;     // 发布绘制结果的图像
    image_transport::Publisher pub_binary_image_;     // 发布二值化图像
    // 二值化图像
    cv::Mat binary_image_;
    /* image_transport：
        这个样子发布后还有附加很多子话题：compressed、theora、compressedDepth
        不用管，这是 image_transport 自动创建的，每个子话题里面的内容是不同形式的图片信息
    */
    
public:
    // 构造函数和析构函数
    getimg();
    ~getimg();
    
    // 参数初始化
    void initParams();
    
    // 颜色名称获取
    std::string getColorName(int color_id);
    
    // 回调函数
    void imageCallback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    // 图像转换
    bool convertImage(const sensor_msgs::ImageConstPtr& img_msg, cv::Mat& output_image);
    
    // 颜色分割
    cv::Mat colorSegmentation(const cv::Mat& hsv_image);
    
    // 核心检测函数 - 现在返回装甲板向量
    // 建议阅读：https://blog.csdn.net/u010750137/article/details/96428059
    std::vector<ArmorDescriptor> detectArmor(const cv::Mat& image);

    // 绘制检测结果
    void drawDetections(cv::Mat& image, const std::vector<ArmorDescriptor>& armors);
    
    // 打印检测信息
    void printDetectionInfo(const std::vector<ArmorDescriptor>& armors);
    
    // 辅助函数：调整旋转矩形方向
    void adjustRect(cv::RotatedRect& rect);
};

#endif