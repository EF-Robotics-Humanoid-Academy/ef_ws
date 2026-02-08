// src/g1_video_publisher.cpp
/*
@author    Salman Omar Sohail <support@mybotshop.de>
@copyright (c) 2025, MYBOTSHOP GmbH
[License text same as before...]
*/

#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// GStreamer
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <glib.h>

// OpenCV
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

class GStreamerImagePublisher : public rclcpp::Node
{
public:
  GStreamerImagePublisher()
      : Node("g1_video_publisher"),
        loop_(nullptr),
        pipeline_(nullptr),
        bus_(nullptr),
        running_(false)
  {
    // ---------------- Params ----------------
    group_ = declare_parameter<std::string>("group", "230.1.1.1");
    port_ = declare_parameter<int>("port", 1720);
    iface_ = declare_parameter<std::string>("iface", "eth0");
    decoder_ = declare_parameter<std::string>("decoder", "avdec_h264"); // try HW: nvh264dec / nvv4l2decoder / vaapih264dec
    image_topic_ = declare_parameter<std::string>("image_topic", "image_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera");

    // OpenCV params
    publish_compressed_ = declare_parameter<bool>("publish_compressed", true);
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 85); // 1..100

    // Low-latency knobs
    drop_old_ = declare_parameter<bool>("drop_old", true);
    sync_sink_ = declare_parameter<bool>("sync", false);    // keep false for live streaming
    latency_ms_ = declare_parameter<int>("latency_ms", 10); // rtpjitterbuffer target latency (ms), 0..30 typical

    int depth = declare_parameter<int>("qos_depth", 1);

    pub_ = create_publisher<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort());

    if (publish_compressed_)
    {
      compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
          image_topic_ + "/compressed",
          rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort());
    }

    RCLCPP_INFO(get_logger(),
                "Publishing on '%s'%s (compressed=%s q=%d)",
                image_topic_.c_str(),
                publish_raw_ ? " (raw enabled)" : "",
                publish_compressed_ ? "yes" : "no",
                jpeg_quality_);

    RCLCPP_INFO(get_logger(),
                "Multicast H.264 → ROS Image | group=%s port=%d iface=%s decoder=%s topic=%s latency=%dms",
                group_.c_str(), port_, iface_.c_str(), decoder_.c_str(), image_topic_.c_str(), latency_ms_);

    // ---------------- GStreamer init ----------------
    if (!gst_is_initialized())
    {
      int argc = 0;
      char **argv = nullptr;
      gst_init(&argc, &argv);
    }

    // Elements (low-latency chain)
    GstElement *udpsrc = gst_element_factory_make("udpsrc", "src");
    GstElement *caps_rtp = gst_element_factory_make("capsfilter", "caps_rtp");
    GstElement *jitter = gst_element_factory_make("rtpjitterbuffer", "jitter");
    GstElement *depay = gst_element_factory_make("rtph264depay", "depay");
    GstElement *q1 = gst_element_factory_make("queue", "q1");
    GstElement *parse = gst_element_factory_make("h264parse", "parse");
    GstElement *q2 = gst_element_factory_make("queue", "q2");
    GstElement *decoder = gst_element_factory_make(decoder_.c_str(), "decoder");
    GstElement *convert = gst_element_factory_make("videoconvert", "convert");
    GstElement *caps_bgr = gst_element_factory_make("capsfilter", "caps_bgr");
    GstElement *q3 = gst_element_factory_make("queue", "q3");
    GstElement *appsink = gst_element_factory_make("appsink", "appsink");

    if (!udpsrc || !caps_rtp || !jitter || !depay || !q1 || !parse || !q2 ||
        !decoder || !convert || !caps_bgr || !q3 || !appsink)
    {
      RCLCPP_FATAL(get_logger(), "Failed to create one or more GStreamer elements. "
                                 "Ensure gstreamer1.0, -base, -good, -bad and chosen decoder are installed.");
      throw std::runtime_error("GStreamer element creation failed");
    }

    // udpsrc: join multicast; increase socket buffer
    g_object_set(G_OBJECT(udpsrc),
                 "address", group_.c_str(),
                 "port", port_,
                 "multicast-iface", iface_.c_str(),
                 "buffer-size", 4194304, // 4 MB socket rcv buffer
                 nullptr);

    // RTP caps (H264)
    {
      GstCaps *caps = gst_caps_new_simple(
          "application/x-rtp",
          "media", G_TYPE_STRING, "video",
          "encoding-name", G_TYPE_STRING, "H264",
          nullptr);
      g_object_set(G_OBJECT(caps_rtp), "caps", caps, nullptr);
      gst_caps_unref(caps);
    }

    // rtpjitterbuffer: minimal latency; drop late/lost packets
    g_object_set(G_OBJECT(jitter),
                 "latency", latency_ms_, // ms
                 "drop-on-late", TRUE,
                 "do-lost", TRUE,
                 nullptr);

    // Tiny, leaky queues (avoid pipeline buildup)
    for (GstElement *q : {q1, q2, q3})
    {
      g_object_set(G_OBJECT(q),
                   "leaky", 2, // downstream
                   "max-size-buffers", 1,
                   "max-size-bytes", 0,
                   "max-size-time", 0,
                   nullptr);
    }

    // Force raw BGR for straight copy to sensor_msgs/Image
    {
      GstCaps *caps = gst_caps_new_simple(
          "video/x-raw",
          "format", G_TYPE_STRING, "BGR",
          nullptr);
      g_object_set(G_OBJECT(caps_bgr), "caps", caps, nullptr);
      gst_caps_unref(caps);
    }

    // appsink: live, no preroll, keep latest only
    g_object_set(G_OBJECT(appsink),
                 "emit-signals", TRUE,
                 "sync", sync_sink_, // keep false for live
                 "async", FALSE,
                 "enable-last-sample", FALSE,
                 "max-buffers", 1,
                 "drop", drop_old_,
                 "max-lateness", (gint64)0, // drop anything late immediately
                 nullptr);

    // Build pipeline
    pipeline_ = gst_pipeline_new("h264_to_rosimg");
    gst_bin_add_many(GST_BIN(pipeline_),
                     udpsrc, caps_rtp, jitter, depay, q1, parse, q2, decoder, convert, caps_bgr, q3, appsink, nullptr);

    if (!gst_element_link(udpsrc, caps_rtp) ||
        !gst_element_link(caps_rtp, jitter) ||
        !gst_element_link(jitter, depay) ||
        !gst_element_link(depay, q1) ||
        !gst_element_link(q1, parse) ||
        !gst_element_link(parse, q2) ||
        !gst_element_link(q2, decoder) ||
        !gst_element_link(decoder, convert) ||
        !gst_element_link(convert, caps_bgr) ||
        !gst_element_link(caps_bgr, q3) ||
        !gst_element_link(q3, appsink))
    {
      RCLCPP_FATAL(get_logger(), "Failed to link GStreamer elements.");
      throw std::runtime_error("Element linking failed");
    }

    // Bus + main loop
    bus_ = gst_element_get_bus(pipeline_);
    loop_ = g_main_loop_new(nullptr, FALSE);
    if (!bus_ || !loop_)
    {
      RCLCPP_FATAL(get_logger(), "Failed to set up GStreamer bus or main loop.");
      throw std::runtime_error("Bus/main loop setup failed");
    }

    gst_bus_add_watch(bus_, &GStreamerImagePublisher::bus_cb_static, this);
    g_signal_connect(appsink, "new-sample",
                     G_CALLBACK(&GStreamerImagePublisher::on_new_sample_static), this);

    // Start
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
      RCLCPP_FATAL(get_logger(), "Unable to set pipeline to PLAYING.");
      throw std::runtime_error("Pipeline PLAYING failed");
    }

    running_.store(true);
    gst_thread_ = std::thread([this]()
                              { g_main_loop_run(loop_); });

    RCLCPP_INFO(get_logger(),
                "Publishing ROS images on '%s' (frame_id='%s') from %s:%d (%s).",
                image_topic_.c_str(), frame_id_.c_str(), group_.c_str(), port_, iface_.c_str());
  }

  ~GStreamerImagePublisher() override { shutdown_pipeline(); }

private:
  // -------- ROS 2 pubsub --------
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  bool publish_compressed_{true};
  bool publish_raw_{true};
  int jpeg_quality_{85};

  // -------- appsink callback: sample → sensor_msgs/Image --------
  static GstFlowReturn on_new_sample_static(GstAppSink *sink, gpointer user_data)
  {
    return static_cast<GStreamerImagePublisher *>(user_data)->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstAppSink *sink)
  {
    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (!sample)
      return GST_FLOW_OK;

    GstCaps *caps = gst_sample_get_caps(sample);
    GstVideoInfo vinfo;
    if (!caps || !gst_video_info_from_caps(&vinfo, caps))
    {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    const int width = GST_VIDEO_INFO_WIDTH(&vinfo);
    const int height = GST_VIDEO_INFO_HEIGHT(&vinfo);
    const int stride = GST_VIDEO_INFO_PLANE_STRIDE(&vinfo, 0); // bytes/row
    const size_t step = static_cast<size_t>(width) * 3;        // BGR8

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map{};
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
    {
      gst_sample_unref(sample);
      return GST_FLOW_OK;
    }

    // Timestamp & header (use node time if PTS not useful)
    std::chrono::nanoseconds stamp_ns;
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (pts != GST_CLOCK_TIME_NONE)
    {
      stamp_ns = std::chrono::nanoseconds(static_cast<int64_t>(pts));
    }
    else
    {
      stamp_ns = std::chrono::nanoseconds(this->now().nanoseconds());
    }

    // ---- OPTIONAL RAW ----
    if (publish_raw_)
    {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      msg.height = static_cast<uint32_t>(height);
      msg.width = static_cast<uint32_t>(width);
      msg.encoding = "bgr8";
      msg.is_bigendian = false;
      msg.step = static_cast<uint32_t>(step);
      msg.data.resize(static_cast<size_t>(height) * step);

      const uint8_t *src = map.data;
      uint8_t *dst = msg.data.data();

      if (stride == static_cast<int>(step))
      {
        std::memcpy(dst, src, msg.data.size());
      }
      else
      {
        for (int y = 0; y < height; ++y)
        {
          std::memcpy(dst + y * step, src + y * stride, step);
        }
      }
      pub_->publish(std::move(msg));
    }

    // ---- COMPRESSED JPEG ----
    if (compressed_pub_)
    {
      // Wrap mapped BGR frame without copy; respect stride
      const uint8_t *src = map.data;
      cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8_t *>(src), stride);

      std::vector<uint8_t> encoded;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, std::min(std::max(jpeg_quality_, 1), 100)};
      bool ok = cv::imencode(".jpg", bgr, encoded, params);

      if (ok)
      {
        sensor_msgs::msg::CompressedImage cmsg;
        cmsg.header.stamp = this->now();
        cmsg.header.frame_id = frame_id_;
        cmsg.format = "jpeg";
        cmsg.data = std::move(encoded);
        compressed_pub_->publish(std::move(cmsg));
      }
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  // -------- Bus callback --------
  static gboolean bus_cb_static(GstBus *bus, GstMessage *msg, gpointer user_data)
  {
    return static_cast<GStreamerImagePublisher *>(user_data)->bus_cb(bus, msg);
  }

  gboolean bus_cb(GstBus *, GstMessage *msg)
  {
    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_ERROR:
    {
      GError *err = nullptr;
      gchar *dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      RCLCPP_ERROR(get_logger(), "GStreamer ERROR: %s (debug: %s)",
                   err ? err->message : "unknown", dbg ? dbg : "none");
      if (err)
        g_error_free(err);
      if (dbg)
        g_free(dbg);
      shutdown_pipeline();
      rclcpp::shutdown();
      break;
    }
    case GST_MESSAGE_EOS:
      RCLCPP_INFO(get_logger(), "EOS received.");
      shutdown_pipeline();
      rclcpp::shutdown();
      break;
    default:
      break;
    }
    return TRUE;
  }

  void shutdown_pipeline()
  {
    if (!running_.exchange(false))
      return;

    if (pipeline_)
    {
      gst_element_send_event(pipeline_, gst_event_new_eos());
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (loop_)
      g_main_loop_quit(loop_);
    if (gst_thread_.joinable())
      gst_thread_.join();

    if (bus_)
    {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    if (pipeline_)
    {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    if (loop_)
    {
      g_main_loop_unref(loop_);
      loop_ = nullptr;
    }

    RCLCPP_INFO(get_logger(), "GStreamer pipeline shut down.");
  }

  // -------- Params --------
  std::string group_;
  int port_;
  std::string iface_;
  std::string decoder_;
  std::string image_topic_;
  std::string frame_id_;
  bool drop_old_;
  bool sync_sink_;
  int latency_ms_;

  // -------- ROS --------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  // -------- GStreamer infra --------
  GMainLoop *loop_;
  GstElement *pipeline_;
  GstBus *bus_;
  std::thread gst_thread_;
  std::atomic<bool> running_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<GStreamerImagePublisher>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
