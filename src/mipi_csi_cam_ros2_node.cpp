/*
 * mipi_csi_cam_ros2_node.cpp
 * ROS 2 camera node for MIPI CSI cameras.
 *
 * Backends:
 *   - libcamera: Raspberry Pi 5 via GStreamer libcamerasrc.
 *   - v4l2: RK3588/RK3588S boards via rkisp1 V4L2 capture nodes.
 *
 * Publishes:
 *   <topic>                   sensor_msgs/Image (bgr8)
 *   <namespace>/camera_info   sensor_msgs/CameraInfo (latched)
 *
 * Parameters:
 *   topic            string  /camera_1/image_raw
 *   source_type      string  libcamera        (libcamera | v4l2)
 *   sensor           string  imx219           (sensor name, used for CameraInfoManager)
 *   fps              int     30
 *   width            int     1280
 *   height           int     960
 *   resize_w         int     0                (0 = native)
 *   resize_h         int     0                (0 = native)
 *   video_device     string  /dev/video11     (V4L2 only)
 *   format           string  NV12             (V4L2 caps format)
 *   io_mode          string  dmabuf           (V4L2 only; empty/auto disables explicit io-mode)
 *   v4l2_use_framerate_caps bool false        (add framerate caps for V4L2)
 *   qos_reliable     bool    true             (RELIABLE for compat with ros2 tools)
 *   calibration_file string  ""               (path to .yaml; empty = no calibration)
 *   frame_id         string  camera_optical_1
 *   camera_name      string  ""               (libcamera camera-name prop, empty = auto)
 */

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <pthread.h>
#include <sched.h>
#include <signal.h>

// ---------------------------------------------------------------------------

static int64_t now_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void try_set_realtime(int prio = 15)
{
    sched_param sp{}; sp.sched_priority = prio;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0)
        fprintf(stdout, "[RT] SCHED_FIFO prio=%d OK\n", prio);
    else
        fprintf(stderr, "[RT] SCHED_FIFO failed (need root or CAP_SYS_NICE)\n");
}

// ---------------------------------------------------------------------------

class MipiCameraNode : public rclcpp::Node
{
public:
    explicit MipiCameraNode()
        : Node("mipi_csi_cam_ros2")
    {
        declare_parameter("topic",            "/camera_1/image_raw");
        declare_parameter("source_type",      "libcamera");
        declare_parameter("sensor",           "imx219");
        declare_parameter("fps",              30);
        declare_parameter("width",            1280);
        declare_parameter("height",           960);
        declare_parameter("resize_w",         0);
        declare_parameter("resize_h",         0);
        declare_parameter("video_device",     "/dev/video11");
        declare_parameter("format",           "NV12");
        declare_parameter("io_mode",          "dmabuf");
        declare_parameter("v4l2_use_framerate_caps", false);
        declare_parameter("qos_reliable",     false);  // BEST_EFFORT: no back-pressure from slow consumers
        declare_parameter("calibration_file", "");
        declare_parameter("frame_id",         "camera_optical_1");
        declare_parameter("camera_name",      "");     // libcamera camera-name property

        topic_      = get_parameter("topic").as_string();
        source_type_ = get_parameter("source_type").as_string();
        sensor_     = get_parameter("sensor").as_string();
        fps_        = get_parameter("fps").as_int();
        cap_w_      = get_parameter("width").as_int();
        cap_h_      = get_parameter("height").as_int();
        resize_w_   = get_parameter("resize_w").as_int();
        resize_h_   = get_parameter("resize_h").as_int();
        video_device_ = get_parameter("video_device").as_string();
        format_     = get_parameter("format").as_string();
        io_mode_    = get_parameter("io_mode").as_string();
        v4l2_use_framerate_caps_ = get_parameter("v4l2_use_framerate_caps").as_bool();
        frame_id_   = get_parameter("frame_id").as_string();
        cam_name_   = get_parameter("camera_name").as_string();
        cal_file_   = get_parameter("calibration_file").as_string();
        bool reliable = get_parameter("qos_reliable").as_bool();

        if (source_type_ != "libcamera" && source_type_ != "v4l2") {
            RCLCPP_WARN(get_logger(),
                "Unknown source_type '%s'; falling back to libcamera",
                source_type_.c_str());
            source_type_ = "libcamera";
        }

        out_w_ = (resize_w_ > 0) ? resize_w_ : cap_w_;
        out_h_ = (resize_h_ > 0) ? resize_h_ : cap_h_;

        // /camera_1/image_raw  →  /camera_1/camera_info
        std::string info_topic;
        auto slash = topic_.rfind('/');
        if (slash != std::string::npos && slash > 0)
            info_topic = topic_.substr(0, slash) + "/camera_info";
        else
            info_topic = topic_ + "_info";

        auto qos = rclcpp::QoS(10);
        reliable ? qos.reliable() : qos.best_effort();

        img_pub_  = create_publisher<sensor_msgs::msg::Image>(topic_, qos);
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            info_topic, rclcpp::QoS(1).reliable().transient_local());

        std::string cal_url = cal_file_.empty() ? "" : "file://" + cal_file_;
        cinfo_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this, sensor_, cal_url);

        auto ci = cinfo_mgr_->getCameraInfo();
        ci.header.stamp    = now();
        ci.header.frame_id = frame_id_;
        if (ci.width  == 0) ci.width  = (uint32_t)out_w_;
        if (ci.height == 0) ci.height = (uint32_t)out_h_;
        info_pub_->publish(ci);

        RCLCPP_INFO(get_logger(),
            "\n=================================================="
            "\n  mipi_csi_cam_ros2"
            "\n=================================================="
            "\n  backend:     %s"
            "\n  sensor:      %s"
            "\n  capture:     %dx%d @ %d fps"
            "\n  output:      %dx%d"
            "\n  device:      %s"
            "\n  format:      %s"
            "\n  io_mode:     %s"
            "\n  topic:       %s"
            "\n  info topic:  %s"
            "\n  QoS:         %s"
            "\n  frame_id:    %s"
            "\n  calibration: %s"
            "\n==================================================",
            source_type_.c_str(),
            sensor_.c_str(),
            cap_w_, cap_h_, fps_,
            out_w_, out_h_,
            source_type_ == "v4l2" ? video_device_.c_str() : "n/a",
            source_type_ == "v4l2" ? format_.c_str() : "n/a",
            source_type_ == "v4l2" ? io_mode_.c_str() : "n/a",
            topic_.c_str(),
            info_topic.c_str(),
            reliable ? "RELIABLE" : "BEST_EFFORT",
            frame_id_.c_str(),
            cal_file_.empty() ? "none" : cal_file_.c_str());

        running_ = true;
        capture_thread_ = std::thread(&MipiCameraNode::capture_loop, this);
    }

    ~MipiCameraNode()
    {
        stop();
    }

    void stop()
    {
        // Guard against double-call (signal handler + destructor)
        if (!running_.exchange(false)) return;

        // Release the capture to unblock cap->read() in the capture thread
        {
            std::lock_guard<std::mutex> lk(cap_mtx_);
            if (cap_) cap_->release();
        }

        if (capture_thread_.joinable())
            capture_thread_.join();
    }

private:
    std::string build_pipeline() const
    {
        if (source_type_ == "v4l2")
            return build_v4l2_pipeline();
        return build_libcamera_pipeline();
    }

    std::string build_libcamera_pipeline() const
    {
        std::ostringstream p;
        p << "libcamerasrc";
        if (!cam_name_.empty())
            p << " camera-name=\"" << cam_name_ << "\"";
        p << " ! video/x-raw,format=BGR"
          << ",width="     << cap_w_
          << ",height="    << cap_h_
          << ",framerate=" << fps_ << "/1"
          << " ! videoconvert"
          << " ! video/x-raw,format=BGR"
          << " ! appsink max-buffers=4 drop=true wait-on-eos=false";
        return p.str();
    }

    std::string build_v4l2_pipeline() const
    {
        std::ostringstream p;
        p << "v4l2src device=" << video_device_
          << " do-timestamp=true";
        if (!io_mode_.empty() && io_mode_ != "auto")
            p << " io-mode=" << io_mode_;
        p << " ! video/x-raw,format=" << format_
          << ",width=" << cap_w_
          << ",height=" << cap_h_;
        if (v4l2_use_framerate_caps_ && fps_ > 0)
            p << ",framerate=" << fps_ << "/1";
        p << " ! videoconvert"
          << " ! video/x-raw,format=BGR"
          << " ! appsink max-buffers=4 drop=true wait-on-eos=false";
        return p.str();
    }

    void capture_loop()
    {
        try_set_realtime(15);

        while (running_) {
            std::string pipeline = build_pipeline();
            RCLCPP_INFO(get_logger(), "Pipeline: %s", pipeline.c_str());

            auto cap = std::make_unique<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);
            if (!cap->isOpened()) {
                RCLCPP_ERROR(get_logger(),
                    "Camera open failed (backend=%s, %dx%d @ %d fps), retrying in 2s...",
                    source_type_.c_str(), cap_w_, cap_h_, fps_);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            // Store pointer so stop() can release the capture to unblock cap->read()
            {
                std::lock_guard<std::mutex> lk(cap_mtx_);
                cap_ = cap.get();
            }

            RCLCPP_INFO(get_logger(), "Streaming: %dx%d -> %dx%d @ %d fps",
                cap_w_, cap_h_, out_w_, out_h_, fps_);

            uint64_t frame_count = 0;
            int64_t  t0 = now_ms();
            cv::Mat  frame, out_frame;

            while (running_) {
                if (!cap->read(frame) || frame.empty()) {
                    if (!running_) break;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Empty frame, waiting...");
                    continue;
                }

                cv::Mat* pub_frame = &frame;
                if (out_w_ != cap_w_ || out_h_ != cap_h_) {
                    cv::resize(frame, out_frame, cv::Size(out_w_, out_h_));
                    pub_frame = &out_frame;
                }

                auto stamp = now();
                auto img_msg = cv_bridge::CvImage(
                    std_msgs::msg::Header{}, "bgr8", *pub_frame).toImageMsg();
                img_msg->header.stamp    = stamp;
                img_msg->header.frame_id = frame_id_;
                img_pub_->publish(*img_msg);

                auto ci = cinfo_mgr_->getCameraInfo();
                ci.header.stamp    = stamp;
                ci.header.frame_id = frame_id_;
                ci.width  = (uint32_t)out_w_;
                ci.height = (uint32_t)out_h_;
                info_pub_->publish(ci);

                ++frame_count;

                int64_t elapsed = now_ms() - t0;
                if (elapsed >= 5000) {
                    RCLCPP_INFO(get_logger(),
                        "%.1f fps | frames=%lu | %dx%d",
                        frame_count * 1000.0 / elapsed, frame_count, out_w_, out_h_);
                    frame_count = 0;
                    t0 = now_ms();
                }
            }

            {
                std::lock_guard<std::mutex> lk(cap_mtx_);
                cap_ = nullptr;
            }
            cap->release();
        }
    }

    std::string topic_, source_type_, sensor_, video_device_, format_, io_mode_;
    std::string frame_id_, cam_name_, cal_file_;
    int         fps_, cap_w_, cap_h_, out_w_, out_h_, resize_w_, resize_h_;
    bool        v4l2_use_framerate_caps_{false};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager>    cinfo_mgr_;

    std::thread        capture_thread_;
    std::atomic<bool>  running_{false};
    std::mutex         cap_mtx_;
    cv::VideoCapture*  cap_{nullptr};
};

// ---------------------------------------------------------------------------

static std::shared_ptr<MipiCameraNode> g_node;

// Signal handler: only trigger ROS shutdown.
// Actual thread cleanup happens after rclcpp::spin() returns in main().
// This avoids the libcamera/GStreamer race condition that caused SIGSEGV
// when stop() was called from the signal handler context.
static void sig_handler(int)
{
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    g_node = std::make_shared<MipiCameraNode>();
    rclcpp::spin(g_node);

    // spin() returned (shutdown was triggered), now cleanly stop the capture thread
    if (g_node) {
        g_node->stop();
        g_node.reset();
    }

    return 0;
}
