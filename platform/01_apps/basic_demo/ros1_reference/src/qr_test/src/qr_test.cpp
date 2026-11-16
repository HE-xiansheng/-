#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <unordered_set>
#include <deque>
#include <iostream>

ros::Subscriber sub_img;
ros::Publisher  pub_qr_imgresult;

zbar::ImageScanner scanner;

// 已输出过的码（带上限）
static std::unordered_set<std::string> printed_qr;
static std::deque<std::string> qr_queue;
static const size_t MAX_QR_CACHE = 100;

// 帧计数
static int frame_count = 0;
static const int FRAME_SKIP = 3;   // 每3帧识别一次

// 四边形有效性检查
static inline bool validQuad(const std::vector<cv::Point>& pts)
{
    if (pts.size() < 4) return false;

    std::vector<cv::Point2f> p2;
    p2.reserve(4);
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite((double)pts[i].x) || !std::isfinite((double)pts[i].y))
            return false;
        p2.emplace_back((float)pts[i].x, (float)pts[i].y);
    }
    double a = cv::contourArea(p2);
    return std::isfinite(a) && a > 1.0;
}

void readStream(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 安全复制一份可修改的图像
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
        ROS_WARN_STREAM_THROTTLE(1.0, "cv_bridge error: " << e.what());
        return;
    }

    cv::Mat &img = cv_ptr->image;
    if (img.empty()) return;

    frame_count++;

    // 每 FRAME_SKIP 帧做一次识别
    if (frame_count % FRAME_SKIP == 0)
    {
        // 缩小图像以减轻计算量
        const double scale = 0.5;  // 1280x720 -> 640x360
        cv::Mat small_bgr, small_gray;
        cv::resize(img, small_bgr, cv::Size(), scale, scale, cv::INTER_AREA);
        cv::cvtColor(small_bgr, small_gray, cv::COLOR_BGR2GRAY);

        // 构造 zbar 图像，使用 Y800（单通道灰度）格式
        zbar::Image zimg(
            small_gray.cols,
            small_gray.rows,
            "Y800",
            small_gray.data,
            small_gray.cols * small_gray.rows
        );

        // 扫描二维码
        int n = scanner.scan(zimg);
        if (n > 0) {
            for (auto it = zimg.symbol_begin(); it != zimg.symbol_end(); ++it) {
                const zbar::Symbol &symbol = *it;

                // 只关心 QR-Code（如需所有类型可去掉这个判断）
                if (symbol.get_type() != zbar::ZBAR_QRCODE)
                    continue;

                std::string decoded_text = symbol.get_data();
                if (decoded_text.empty())
                    continue;

                // 去重 + 限制缓存大小
                if (printed_qr.insert(decoded_text).second) {
                    qr_queue.push_back(decoded_text);
                    if (qr_queue.size() > MAX_QR_CACHE) {
                        printed_qr.erase(qr_queue.front());
                        qr_queue.pop_front();
                    }
                    std::cout << "QR: " << decoded_text << std::endl;
                }

                // 取出角点坐标（在缩小图上），放大回原图
                int loc_size = symbol.get_location_size();
                std::vector<cv::Point> pts;
                pts.reserve(loc_size);
                for (int i = 0; i < loc_size; ++i) {
                    int x = symbol.get_location_x(i);
                    int y = symbol.get_location_y(i);
                    pts.emplace_back(
                        static_cast<int>(x / scale),
                        static_cast<int>(y / scale)
                    );
                }

                if (validQuad(pts)) {
                    // 画框
                    for (size_t i = 0; i < pts.size(); ++i) {
                        cv::line(img,
                                 pts[i],
                                 pts[(i + 1) % pts.size()],
                                 cv::Scalar(0, 255, 0),
                                 3);
                    }
                    // 写文字
                    cv::Point org = pts[0];
                    org.y = std::max(30, org.y - 10);
                    cv::putText(img, decoded_text, org,
                                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                                cv::Scalar(0, 255, 0), 2);
                }
            }
        }

        // zbar::Image 的数据指针指向 small_gray，不会被 zbar 释放，
        // zimg 析构即可，不需要手动 free。
    }

    // 发布带标注的图像
    pub_qr_imgresult = pub_qr_imgresult; // 防止未使用警告（有些编译器）
    pub_qr_imgresult.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qr_code_zbar");
    ros::NodeHandle nh;

    // 限制 OpenCV 线程，避免把所有核跑满
    cv::setNumThreads(2);

    // 初始化 zbar 扫描器
    scanner.set_config(zbar::ZBAR_NONE,   zbar::ZBAR_CFG_ENABLE, 0); // 先关掉全部
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1); // 只启用 QR

    sub_img = nh.subscribe("/camera/color/image_raw", 1, readStream);
    pub_qr_imgresult = nh.advertise<sensor_msgs::Image>("/qr_imgresult", 1);

    ROS_INFO("qr_code_zbar running ...");
    ros::spin();
    return 0;
}
