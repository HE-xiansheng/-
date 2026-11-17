#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <cmath>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cstring>
#include <csignal>
#include "frame_rate_gate.hpp"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/time.h>
}

#include "rk_mpi.h"
#include "mpp_buffer.h"
#include "mpp_frame.h"
#include "mpp_packet.h"

#define mpp_err printf

// A frame may be between gray/color publications when a signal arrives. Keep
// DDS alive until that work and reliable acknowledgments have finished.
static volatile std::sig_atomic_t stop_requested = 0;
static void request_stop(int) { stop_requested = 1; }

// ---- 解码器上下文 ----
typedef struct {
    MppCtx ctx;
    MppApi *mpi;
    MppPacket packet;
    MppFrame frame;
    MppBufferGroup buf_mgr;
    RK_U32 width, height, hor_stride, ver_stride;
    MppCodingType coding_type;
    MppFrameFormat output_format;
    int info_changed;
    int decoder_initialized;
    AVCodecParserContext *parser;
    AVCodecContext *parser_ctx;
    uint8_t *extradata;
    int extradata_size;
    int extradata_sent;
} DecoderContext;

// ---- MPP 缓冲区管理 ----
static MPP_RET dec_buf_mgr_init(MppBufferGroup *buf_mgr) {
    MppBufferGroup group = NULL;
    MPP_RET ret = mpp_buffer_group_get_internal(&group, MPP_BUFFER_TYPE_ION);
    if (ret != MPP_OK) mpp_err("buffer group init failed\n");
    *buf_mgr = group;
    return ret;
}

static void dec_buf_mgr_deinit(MppBufferGroup buf_mgr) {
    if (buf_mgr) mpp_buffer_group_put(buf_mgr);
}

// ---- 解码器初始化 / 销毁 ----
static int send_extradata_to_decoder(DecoderContext *dec) {
    if (!dec->extradata || dec->extradata_sent) return 0;
    if (dec->packet) mpp_packet_deinit(&dec->packet);
    MPP_RET ret = mpp_packet_init(&dec->packet, dec->extradata, dec->extradata_size);
    if (ret != MPP_OK) { mpp_err("extradata packet init fail\n"); return -1; }
    ret = dec->mpi->decode_put_packet(dec->ctx, dec->packet);
    if (ret != MPP_OK) { mpp_err("send extradata fail\n"); return -1; }
    dec->extradata_sent = 1;
    return 0;
}

static int init_decoder(DecoderContext *dec, MppCodingType type, RK_U32 width, RK_U32 height,
                        uint8_t *extradata, int extradata_size) {
    memset(dec, 0, sizeof(*dec));
    dec->coding_type = type;
    dec->width = width;
    dec->height = height;
    dec->output_format = MPP_FMT_YUV420SP;

    if (extradata && extradata_size > 0) {
        dec->extradata = (uint8_t*)malloc(extradata_size);
        if (!dec->extradata) return -1;
        memcpy(dec->extradata, extradata, extradata_size);
        dec->extradata_size = extradata_size;
    }

    if (mpp_create(&dec->ctx, &dec->mpi) != MPP_OK) { mpp_err("mpp_create fail\n"); return -1; }
    if (mpp_init(dec->ctx, MPP_CTX_DEC, dec->coding_type) != MPP_OK) { mpp_err("mpp_init fail\n"); return -1; }
    dec->mpi->control(dec->ctx, MPP_DEC_SET_OUTPUT_FORMAT, &dec->output_format);

    if (dec_buf_mgr_init(&dec->buf_mgr) != MPP_OK) return -1;
    if (dec->buf_mgr) dec->mpi->control(dec->ctx, MPP_DEC_SET_EXT_BUF_GROUP, dec->buf_mgr);
    if (mpp_packet_init(&dec->packet, NULL, 0) != MPP_OK) return -1;

    dec->parser = av_parser_init(type == MPP_VIDEO_CodingHEVC ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264);
    dec->parser_ctx = avcodec_alloc_context3(NULL);
    if (!dec->parser || !dec->parser_ctx) return -1;
    if (dec->parser_ctx && extradata && extradata_size > 0) {
        dec->parser_ctx->extradata = (uint8_t*)av_mallocz(extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
        if (!dec->parser_ctx->extradata) return -1;
        memcpy(dec->parser_ctx->extradata, extradata, extradata_size);
        dec->parser_ctx->extradata_size = extradata_size;
    }

    dec->decoder_initialized = 1;
    return 0;
}

static void destroy_decoder(DecoderContext *dec) {
    if (!dec) return;
    if (dec->packet) mpp_packet_deinit(&dec->packet);
    if (dec->frame) mpp_frame_deinit(&dec->frame);
    // The decoder must release its references before the external group is put.
    if (dec->ctx) { dec->mpi->reset(dec->ctx); mpp_destroy(dec->ctx); dec->ctx = nullptr; }
    if (dec->buf_mgr) { dec_buf_mgr_deinit(dec->buf_mgr); dec->buf_mgr = nullptr; }
    if (dec->parser) { av_parser_close(dec->parser); dec->parser = nullptr; }
    // avcodec_free_context owns and releases parser_ctx->extradata.
    avcodec_free_context(&dec->parser_ctx);
    free(dec->extradata);
    dec->extradata = nullptr;
    dec->decoder_initialized = 0;
}

// ---- 硬件解码核心 ----
static void handle_info_change(DecoderContext *dec, MppFrame frame) {
    RK_U32 w = mpp_frame_get_width(frame), h = mpp_frame_get_height(frame);
    dec->width = w; dec->height = h;
    dec->hor_stride = mpp_frame_get_hor_stride(frame);
    dec->ver_stride = mpp_frame_get_ver_stride(frame);
    dec->mpi->control(dec->ctx, MPP_DEC_SET_OUTPUT_FORMAT, &dec->output_format);
    if (dec->buf_mgr) {
        size_t buf_size = dec->hor_stride * dec->ver_stride * 3 / 2;
        mpp_buffer_group_limit_config(dec->buf_mgr, buf_size * 24, 24);
        dec->mpi->control(dec->ctx, MPP_DEC_SET_EXT_BUF_GROUP, dec->buf_mgr);
    }
    dec->mpi->control(dec->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
}

static int decode_frame(DecoderContext *dec, uint8_t *data, size_t len,
                        void **frame_data, int *frame_width, int *frame_height) {
    if (!dec->decoder_initialized) return 0;

    uint8_t *parse_data = data;
    int parse_len = len, decoded = 0;

    if (dec->parser) {
        while (parse_len > 0) {
            uint8_t *out_data = NULL;
            int out_size = 0;
            int consumed = av_parser_parse2(dec->parser, dec->parser_ctx, &out_data, &out_size,
                                            parse_data, parse_len, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if (consumed < 0 || (consumed == 0 && out_size == 0)) break;
            parse_data += consumed;
            parse_len -= consumed;
            if (out_size > 0) {
                if (dec->packet) mpp_packet_deinit(&dec->packet);
                if (mpp_packet_init(&dec->packet, out_data, out_size) == MPP_OK) {
                    dec->mpi->decode_put_packet(dec->ctx, dec->packet);
                    decoded = 1;
                }
            }
        }
    }

    if (!decoded) return 0;

    if (dec->frame) { mpp_frame_deinit(&dec->frame); dec->frame = NULL; }
    if (dec->mpi->decode_get_frame(dec->ctx, &dec->frame) != MPP_OK) return 0;
    if (!dec->frame) return 0;

    if (mpp_frame_get_info_change(dec->frame)) {
        handle_info_change(dec, dec->frame);
        return 0;
    }

    MppBuffer buffer = mpp_frame_get_buffer(dec->frame);
    if (buffer) {
        *frame_data = mpp_buffer_get_ptr(buffer);
        *frame_width = mpp_frame_get_width(dec->frame);
        *frame_height = mpp_frame_get_height(dec->frame);
        return 1;
    }
    return 0;
}

// ============================================================
// 主函数：RTSP 硬解 + ROS 发布
//   /rtsp_cam/image_raw    mono8  30 Hz  （检测用，直接取 NV12 的 Y 平面）
//   /rtsp_cam/image_color  bgr8   color_hz （预览用，降采样后转换）
// 旧的 bgr8 image_raw 每帧要做一次 1080p YUV→BGR 并拷贝 6 MB，占 1.3 个核；
// 改成 mono8 后这两项都省掉，检测节点也无需再做 BGR→灰度。
// ============================================================
int main(int argc, char **argv) {
    // Scope the large-frame transport to this publisher process. Keep an explicit
    // caller profile and any selected non-Fast-DDS RMW unchanged.
    const char* rmw = std::getenv("RMW_IMPLEMENTATION");
    const bool fastdds = !rmw || std::string(rmw) == "rmw_fastrtps_cpp" ||
        std::string(rmw) == "rmw_fastrtps_dynamic_cpp";
    if (fastdds && !std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE") &&
        !std::getenv("FASTDDS_DEFAULT_PROFILES_FILE")) {
        const auto profile = ament_index_cpp::get_package_share_directory("rtsp_test") +
            "/config/fastdds_large_images.xml";
        if (!std::ifstream(profile).good() || setenv("FASTRTPS_DEFAULT_PROFILES_FILE", profile.c_str(), 0) != 0)
            throw std::runtime_error("Cannot select RTSP large-image DDS profile: " + profile);
    }
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    // Small conversions only; the default 8-thread pool spins more than it helps.
    cv::setNumThreads(2);
    auto nh = std::make_shared<rclcpp::Node>("rtsp_hw_pub_ultra");
    if (const char* profile = std::getenv("FASTRTPS_DEFAULT_PROFILES_FILE"))
        RCLCPP_INFO(nh->get_logger(), "Fast DDS profile: %s", profile);
    auto gray_pub = nh->create_publisher<sensor_msgs::msg::Image>("/rtsp_cam/image_raw", rclcpp::QoS(1).reliable());
    auto color_pub = nh->create_publisher<sensor_msgs::msg::Image>("/rtsp_cam/image_color", rclcpp::QoS(1).reliable());

    std::string url = "rtsp://192.168.144.108";
    std::string frame_id = "gimbal_camera_optical_frame";
    bool publish_gray = true;
    double color_hz = 15.0;
    int color_width = 960;
    url = nh->declare_parameter("rtsp_url", url);
    frame_id = nh->declare_parameter("frame_id", frame_id);
    publish_gray = nh->declare_parameter("publish_gray", publish_gray);
    color_hz = nh->declare_parameter("color_hz", color_hz);
    color_width = nh->declare_parameter("color_width", color_width);
    FrameRateGate color_gate(color_hz);
    // RTSP URL is a declared ROS2 parameter; --ros-args is never treated as a URL.

    const double connect_timeout_s = nh->declare_parameter("connect_timeout_s", 5.0);
    const double read_timeout_s = nh->declare_parameter("read_timeout_s", 3.0);
    const double reconnect_delay_s = nh->declare_parameter("reconnect_delay_s", 1.0);
    const double frame_timeout_s = nh->declare_parameter("frame_timeout_s", 3.0);
    for (double value : {connect_timeout_s, read_timeout_s, reconnect_delay_s, frame_timeout_s})
        if (!std::isfinite(value) || value < 0.1 || value > 60.0)
            throw std::invalid_argument("RTSP timeouts and reconnect delay must be finite, within [0.1,60] seconds");

    using Clock = std::chrono::steady_clock;
    auto after = [](double seconds) {
        return Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    };
    struct Deadline { Clock::time_point at; } io_deadline{after(connect_timeout_s)};
    AVFormatContext *fmt_ctx = nullptr;
    AVPacket *pkt = av_packet_alloc();
    if (!pkt) throw std::bad_alloc();
    DecoderContext dec{};
    int video_idx = -1;
    avformat_network_init();
    auto close_stream = [&]() {
        // Every new connection owns a fresh parser, MPP context and stride state.
        av_packet_unref(pkt);
        avformat_close_input(&fmt_ctx);
        destroy_decoder(&dec);
        dec = DecoderContext{};
        video_idx = -1;
    };
    auto open_stream = [&]() {
        io_deadline.at = after(connect_timeout_s);
        fmt_ctx = avformat_alloc_context();
        if (!fmt_ctx) return false;
        fmt_ctx->interrupt_callback = {[](void* opaque) -> int {
            const auto* deadline = static_cast<Deadline*>(opaque);
            return stop_requested || !rclcpp::ok() || Clock::now() >= deadline->at;
        }, &io_deadline};
        AVDictionary *opts = nullptr;
        av_dict_set(&opts, "rtsp_transport", "tcp", 0);
        av_dict_set(&opts, "max_delay", "50000", 0);
        av_dict_set(&opts, "buffer_size", "512000", 0);
        av_dict_set(&opts, "fflags", "nobuffer", 0);
        av_dict_set(&opts, "flags", "low_delay", 0);
        // Live RTSP already provides codec parameters in SDP. Bound probing so
        // it finishes before the connection deadline on a healthy stream.
        av_dict_set(&opts, "analyzeduration", "1000000", 0);
        const int opened = avformat_open_input(&fmt_ctx, url.c_str(), nullptr, &opts);
        av_dict_free(&opts);
        if (opened < 0 || avformat_find_stream_info(fmt_ctx, nullptr) < 0 ||
            stop_requested || !rclcpp::ok() || Clock::now() >= io_deadline.at ||
            (fmt_ctx->pb && fmt_ctx->pb->error < 0)) {
            // FFmpeg may return partial metadata after an interrupt. Such a
            // context has a latched IO error and must not enter the read loop.
            close_stream(); return false;
        }
        for (unsigned i=0; i<fmt_ctx->nb_streams; ++i)
            if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_idx = static_cast<int>(i); break;
            }
        if (video_idx < 0) { close_stream(); return false; }
        AVCodecParameters *cp = fmt_ctx->streams[video_idx]->codecpar;
        if (cp->codec_id != AV_CODEC_ID_H264 && cp->codec_id != AV_CODEC_ID_HEVC) {
            RCLCPP_WARN(nh->get_logger(), "RTSP stream is not H264/H265");
            close_stream(); return false;
        }
        const MppCodingType type = cp->codec_id == AV_CODEC_ID_HEVC ? MPP_VIDEO_CodingHEVC : MPP_VIDEO_CodingAVC;
        if (init_decoder(&dec, type, cp->width, cp->height, cp->extradata, cp->extradata_size) ||
            send_extradata_to_decoder(&dec)) {
            close_stream(); return false;
        }
        color_gate = FrameRateGate(color_hz);
        RCLCPP_INFO(nh->get_logger(), "RTSP stream connected: %dx%d", cp->width, cp->height);
        return true;
    };

    // 仅启用子logger的debug时记录真实发布消息源戳，便于区分订阅漏帧与重写。
    const auto frame_logger = nh->get_logger().get_child("frames");
    unsigned long gray_frames = 0, color_frames = 0;
    rclcpp::Time rate_start = nh->now();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh);
    auto next_connect = Clock::now();
    auto last_frame = Clock::now();
    while (rclcpp::ok() && !stop_requested) {
        if (!fmt_ctx) {
            executor.spin_some();
            if (Clock::now() < next_connect) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (!open_stream()) {
                if (rclcpp::ok() && !stop_requested) RCLCPP_WARN(nh->get_logger(), "RTSP connect failed or timed out; retrying");
                next_connect = after(reconnect_delay_s);
                continue;
            }
            last_frame = Clock::now();
        }
        io_deadline.at = after(read_timeout_s);
        if (av_read_frame(fmt_ctx, pkt) < 0) {
            if (rclcpp::ok() && !stop_requested) RCLCPP_WARN(nh->get_logger(), "RTSP read failed or timed out; reconnecting");
            close_stream(); next_connect = after(reconnect_delay_s); continue;
        }
        if (pkt->stream_index == video_idx) {
            const auto process_begin = Clock::now();
            void *data = NULL;
            int fw = 0, fh = 0;
            if (decode_frame(&dec, pkt->data, pkt->size, &data, &fw, &fh) == 1 && dec.frame) {
                last_frame = Clock::now();
                if (!rclcpp::ok()) break;
                const auto decoded_at = Clock::now();
                const rclcpp::Time stamp = nh->now();
                std_msgs::msg::Header header;
                header.stamp = stamp;
                header.frame_id = frame_id;

                MppBuffer buffer = mpp_frame_get_buffer(dec.frame);
                void *plane = buffer ? mpp_buffer_get_ptr(buffer) : NULL;
                const RK_U32 width = dec.width, height = dec.height;
                const RK_U32 hor_stride = dec.hor_stride ? dec.hor_stride : width;
                const RK_U32 ver_stride = dec.ver_stride ? dec.ver_stride : height;

                if (plane && publish_gray && gray_pub->get_subscription_count() > 0) {
                    // The NV12 Y plane is already the greyscale image: no
                    // conversion, just a 1/3-size copy into the message. The
                    // message is filled row by row so a padded MPP stride does
                    // not need an extra clone.
                    sensor_msgs::msg::Image::SharedPtr message(new sensor_msgs::msg::Image);
                    message->header = header;
                    message->height = height;
                    message->width = width;
                    message->encoding = "mono8";
                    message->is_bigendian = 0;
                    message->step = width;
                    message->data.resize(size_t(width) * height);
                    if (hor_stride == width) {
                        std::memcpy(message->data.data(), plane, message->data.size());
                    } else {
                        for (RK_U32 row = 0; row < height; ++row)
                            std::memcpy(message->data.data() + size_t(row) * width,
                                        (uint8_t*)plane + size_t(row) * hor_stride, width);
                    }
                    gray_pub->publish(*message);
                    RCLCPP_DEBUG(frame_logger, "published_raw stamp_ns=%lld",
                        static_cast<long long>(rclcpp::Time(message->header.stamp).nanoseconds()));
                    ++gray_frames;
                }
                const auto gray_done = Clock::now();
                if (plane && color_hz > 0 && color_pub->get_subscription_count() > 0 &&
                    color_gate.ready(FrameRateGate::Clock::now())) {
                    const int cw = std::max(16, std::min<int>(color_width, int(width)));
                    const int ch = std::max(2, (int(height) * cw / int(width)) & ~1);
                    cv::Mat y_full(ver_stride, hor_stride, CV_8UC1, plane);
                    cv::Mat uv_full(ver_stride / 2, hor_stride / 2, CV_8UC2,
                                    (uint8_t*)plane + size_t(hor_stride) * ver_stride);
                    cv::Mat y_small, uv_small, bgr;
                    cv::Mat y_roi = y_full(cv::Rect(0, 0, width, height));
                    cv::Mat uv_roi = uv_full(cv::Rect(0, 0, width / 2, height / 2));
                    // Fractional INTER_AREA scaling repeatedly reads source pixels.
                    // MPP DMA memory is expensive for those accesses: copy once to
                    // cached storage, preserving the existing interpolation result.
                    if (width % cw || height % ch) {
                        y_roi = y_roi.clone();
                        uv_roi = uv_roi.clone();
                    }
                    cv::resize(y_roi, y_small,
                               cv::Size(cw, ch), 0, 0, cv::INTER_AREA);
                    cv::resize(uv_roi, uv_small,
                               cv::Size(cw / 2, ch / 2), 0, 0, cv::INTER_AREA);
                    cv::cvtColorTwoPlane(y_small, uv_small, bgr, cv::COLOR_YUV2BGR_NV12);
                    const auto color_message = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
                    color_pub->publish(*color_message);
                    RCLCPP_DEBUG(frame_logger, "published_color stamp_ns=%lld",
                        static_cast<long long>(rclcpp::Time(color_message->header.stamp).nanoseconds()));
                    ++color_frames;
                }
                RCLCPP_DEBUG(frame_logger, "frame_work width=%u decode_ms=%.3f gray_ms=%.3f color_ms=%.3f",
                    width, std::chrono::duration<double, std::milli>(decoded_at-process_begin).count(),
                    std::chrono::duration<double, std::milli>(gray_done-decoded_at).count(),
                    std::chrono::duration<double, std::milli>(Clock::now()-gray_done).count());
                mpp_frame_deinit(&dec.frame);
                dec.frame = NULL;
            }
        }
        av_packet_unref(pkt);
        if (std::chrono::duration<double>(Clock::now() - last_frame).count() > frame_timeout_s) {
            RCLCPP_WARN(nh->get_logger(), "RTSP decoder produced no frame within timeout; reconnecting");
            close_stream(); next_connect = after(reconnect_delay_s);
        }
        if (rclcpp::ok()) executor.spin_some();
        const double elapsed = (nh->now() - rate_start).seconds();
        if (elapsed >= 10.0) {
            RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 10000, "RTSP decode: gray %.1f Hz, color %.1f Hz, subscribers gray=%zu color=%zu",
                              gray_frames / elapsed, color_frames / elapsed,
                              gray_pub->get_subscription_count(), color_pub->get_subscription_count());
            gray_frames = color_frames = 0;
            rate_start = nh->now();
        }
    }

    close_stream();
    av_packet_free(&pkt);
    avformat_network_deinit();
    const bool gray_acked = gray_pub->wait_for_all_acked(std::chrono::seconds(1));
    const bool color_acked = color_pub->wait_for_all_acked(std::chrono::seconds(1));
    if (!gray_acked || !color_acked)
        RCLCPP_WARN(nh->get_logger(), "RTSP shutdown acknowledgment timeout: gray=%d color=%d", gray_acked, color_acked);
    rclcpp::shutdown();
    return gray_acked && color_acked ? 0 : 1;
}
