/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Rosbag1Dataset.cpp
 * @brief  RawDataSource for datasets in ROS1 bag format, without a ROS1 install
 * @author Jose Luis Blanco Claraco
 * @date   May 20, 2025
 */

/** \defgroup mola_input_rosbag1_grp mola_input_rosbag1_grp
 * RawDataSource for datasets in ROS1 rosbag (.bag) format.
 *
 * Portions of this program source code are based on
 * rosbag2rawlog (MRPT project), Hunter Laux, 2018, JLBC, 2018-2024.
 */

#include <mola_input_rosbag1/Rosbag1Dataset.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/filesystem.h>

// MRPT <-> ROS1 message conversions (vendored mrpt_ros1bridge sub-library):
#include <mrpt/ros1bridge/gps.h>
#include <mrpt/ros1bridge/imu.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/time.h>

// Vendored ROS1 message definitions and rosbag reader:
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_msgs/TFMessage.h>

// ROS2 tf2 for the transform tree (geometry2 package, available in the build env):
#include <algorithm>
#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(Rosbag1Dataset, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_Rosbag1Dataset)  // NOLINT(misc-use-anonymous-namespace)
{
  MOLA_REGISTER_MODULE(Rosbag1Dataset);
}

namespace
{
/** Converts a vendored ROS1 geometry_msgs::TransformStamped into the ROS2
 *  message type expected by tf2::BufferCore. */
geometry_msgs::msg::TransformStamped toRos2Transform(const geometry_msgs::TransformStamped& in)
{
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp.sec        = static_cast<int32_t>(in.header.stamp.sec);
  out.header.stamp.nanosec    = in.header.stamp.nsec;
  out.header.frame_id         = in.header.frame_id;
  out.child_frame_id          = in.child_frame_id;
  out.transform.translation.x = in.transform.translation.x;
  out.transform.translation.y = in.transform.translation.y;
  out.transform.translation.z = in.transform.translation.z;
  out.transform.rotation.x    = in.transform.rotation.x;
  out.transform.rotation.y    = in.transform.rotation.y;
  out.transform.rotation.z    = in.transform.rotation.z;
  out.transform.rotation.w    = in.transform.rotation.w;
  return out;
}

/** Manual conversion sensor_msgs/Image -> mrpt::img::CImage, so we do not
 *  depend on cv_bridge (which would require its ROS2 message types). */
mrpt::img::CImage imageFromROS(const sensor_msgs::Image& image)
{
  namespace enc = sensor_msgs::image_encodings;

  const unsigned int w = image.width;
  const unsigned int h = image.height;
  ASSERT_GT_(w, 0U);
  ASSERT_GT_(h, 0U);

  const std::string& encoding = image.encoding;

  bool         color       = false;
  bool         swapRedBlue = false;
  unsigned int channels    = 0;
  if (encoding == enc::MONO8)
  {
    color    = false;
    channels = 1;
  }
  else if (encoding == enc::BGR8)
  {
    color       = true;
    channels    = 3;
    swapRedBlue = false;
  }
  else if (encoding == enc::RGB8)
  {
    color       = true;
    channels    = 3;
    swapRedBlue = true;
  }
  else if (encoding == enc::MONO16)
  {
    // 16-bit grayscale: scale the high byte to produce an 8-bit image.
    mrpt::img::CImage    out;
    std::vector<uint8_t> buf(static_cast<size_t>(w) * h);
    for (unsigned int row = 0; row < h; row++)
    {
      const auto* srcRow = reinterpret_cast<const uint16_t*>(
          image.data.data() + static_cast<size_t>(row) * image.step);
      uint8_t* dstRow = buf.data() + static_cast<size_t>(row) * w;
      for (unsigned int col = 0; col < w; col++)
        dstRow[col] = static_cast<uint8_t>(srcRow[col] >> 8);
    }
    out.loadFromMemoryBuffer(w, h, false /*grayscale*/, buf.data());
    return out;
  }
  else if (
      encoding == enc::BAYER_RGGB8 || encoding == enc::BAYER_BGGR8 ||
      encoding == enc::BAYER_GBRG8 || encoding == enc::BAYER_GRBG8)
  {
    // Debayer to BGR using OpenCV.
    // Mapping: ROS name → OpenCV code (matches cv_bridge convention)
    int code = cv::COLOR_BayerBG2BGR;
    if (encoding == enc::BAYER_BGGR8)
      code = cv::COLOR_BayerRG2BGR;
    else if (encoding == enc::BAYER_GBRG8)
      code = cv::COLOR_BayerGR2BGR;
    else if (encoding == enc::BAYER_GRBG8)
      code = cv::COLOR_BayerGB2BGR;
    // else BAYER_RGGB8 → COLOR_BayerBG2BGR (already default)

    cv::Mat src(
        static_cast<int>(h), static_cast<int>(w), CV_8UC1,
        const_cast<unsigned char*>(image.data.data()), image.step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, code);
    mrpt::img::CImage out;
    out.loadFromMemoryBuffer(w, h, true /*color*/, bgr.data, false /*already BGR*/);
    return out;
  }
  else if (encoding == enc::RGBA8)
  {
    cv::Mat src(
        static_cast<int>(h), static_cast<int>(w), CV_8UC4,
        const_cast<unsigned char*>(image.data.data()), image.step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_RGBA2BGR);
    mrpt::img::CImage out;
    out.loadFromMemoryBuffer(w, h, true, bgr.data, false);
    return out;
  }
  else if (encoding == enc::BGRA8)
  {
    cv::Mat src(
        static_cast<int>(h), static_cast<int>(w), CV_8UC4,
        const_cast<unsigned char*>(image.data.data()), image.step);
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
    mrpt::img::CImage out;
    out.loadFromMemoryBuffer(w, h, true, bgr.data, false);
    return out;
  }
  else
  {
    THROW_EXCEPTION_FMT(
        "Unsupported image encoding '%s'. Supported: mono8, mono16, rgb8, bgr8, rgba8, bgra8, "
        "bayer_rggb8, bayer_bggr8, bayer_gbrg8, bayer_grbg8.",
        encoding.c_str());
  }

  const unsigned int expectedStride = w * channels;
  ASSERT_GE_(image.data.size(), static_cast<size_t>(image.step) * h);

  mrpt::img::CImage out;

  if (image.step == expectedStride)
  {
    // Contiguous: load directly (CImage copies the buffer):
    out.loadFromMemoryBuffer(
        w, h, color, const_cast<unsigned char*>(image.data.data()), swapRedBlue);
  }
  else
  {
    // Row stride has padding: repack into a contiguous buffer first:
    std::vector<unsigned char> packed(static_cast<size_t>(expectedStride) * h);
    for (unsigned int row = 0; row < h; row++)
    {
      std::memcpy(
          packed.data() + static_cast<size_t>(row) * expectedStride,
          image.data.data() + static_cast<size_t>(row) * image.step, expectedStride);
    }
    out.loadFromMemoryBuffer(w, h, color, packed.data(), swapRedBlue);
  }

  return out;
}
}  // namespace

struct Rosbag1Dataset::BagInfo
{
  BagInfo() = default;

  std::vector<std::shared_ptr<rosbag::Bag>> bags;
  rosbag::View                              full_view;

  // Sequential read cursor over all messages in the bag(s), in time order:
  rosbag::View::iterator iter;
  rosbag::View::iterator end;
  bool                   iter_initialized = false;
};

Rosbag1Dataset::Rosbag1Dataset() : bag_reader_(std::make_shared<BagInfo>())
{
  this->setLoggerName("Rosbag1Dataset");
  tfBuffer_ = std::make_shared<tf2::BufferCore>();
}

void Rosbag1Dataset::initialize_rds(const Yaml& c)
{
  using namespace std::string_literals;

  // ROS1 datatypes (note: no "/msg/" infix, unlike ROS2) -> MOLA classes:
  const std::map<std::string, std::string> mapTopic2Class = {
      {"sensor_msgs/Imu", "CObservationIMU"},
      {"sensor_msgs/Image", "CObservationImage"},
      {"sensor_msgs/CompressedImage", "CObservationImage"},
      {"sensor_msgs/PointCloud2", "CObservationPointCloud"},
      {"livox_ros_driver/CustomMsg", "CObservationPointCloud"},
      {"livox_ros_driver2/CustomMsg", "CObservationPointCloud"},
      {"sensor_msgs/LaserScan", "CObservation2DRangeScan"},
      {"sensor_msgs/NavSatFix", "CObservationGPS"},
      {"nav_msgs/Odometry", "CObservationOdometry"},
      {"geometry_msgs/PoseStamped", "CObservationRobotPose"},
  };

  MRPT_START
  ProfilerEntry tle(profiler_, "initialize");

  // Mandatory parameters:
  ENSURE_YAML_ENTRY_EXISTS(c, "params");
  const auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

  // 'rosbag_filename' may be either a single scalar path, or a YAML sequence
  // of paths, so that several .bag files (e.g. a sensors bag plus a separate
  // ground-truth-only bag) can be merged and replayed jointly, in time order.
  ENSURE_YAML_ENTRY_EXISTS(cfg, "rosbag_filename");
  const auto rosbagFilenameNode = cfg["rosbag_filename"];
  if (rosbagFilenameNode.isSequence())
  {
    const auto seq = rosbagFilenameNode.asSequence();
    for (const auto& f : seq)
    {
      // Skip empty entries, e.g. coming from an unset "${OPTIONAL_BAG|}"
      // mola-cli environment-variable placeholder, so that a second
      // (ground-truth) bag can be made optional in a launch file.
      if (auto s = f.as<std::string>(); !s.empty()) rosbag_filenames_.push_back(s);
    }
  }
  else
  {
    rosbag_filenames_.push_back(rosbagFilenameNode.as<std::string>());
  }
  ASSERT_(!rosbag_filenames_.empty());
  rosbag_filename_ = rosbag_filenames_.front();

  YAML_LOAD_MEMBER_OPT(time_warp_scale, double);
  YAML_LOAD_MEMBER_OPT(base_link_frame_id, std::string);
  YAML_LOAD_MEMBER_OPT(read_ahead_length, size_t);
  YAML_LOAD_MEMBER_OPT(ground_truth_topic, std::string);
  paused_ = cfg.getOrDefault<bool>("start_paused", paused_);

  // Open input ros bag(s), merging them into one single chronological View:
  for (const auto& file : rosbag_filenames_)
  {
    ASSERT_FILE_EXISTS_(file);
    MRPT_LOG_INFO_STREAM("Opening: " << file);
    auto bag = std::make_shared<rosbag::Bag>();
    bag->open(file, rosbag::bagmode::Read);
    bag_reader_->bags.push_back(bag);
    bag_reader_->full_view.addQuery(*bag);
  }

  // Message count:
  bagMessageCount_ = bag_reader_->full_view.size();

  MRPT_LOG_INFO_STREAM("List of topics found in the bag (" << bagMessageCount_ << " msgs)");

  // Build map: topic name -> type
  std::map<std::string, std::string> topic2type;

  const std::vector<const rosbag::ConnectionInfo*>& connections =
      bag_reader_->full_view.getConnections();

  for (const auto& connection : connections)
  {
    topic2type[connection->topic] = connection->datatype;
    MRPT_LOG_INFO_STREAM(" " << connection->topic << " (" << connection->datatype << ")");
  }

  read_ahead_.clear();
  read_ahead_.resize(bagMessageCount_);
  rosbag_next_idx_ = 0;

  // Pre-scan all /tf_static messages and populate the tf buffer now, before
  // sequential playback starts. Static transforms are time-independent, so
  // pre-loading them ensures sensor poses are available even when /tf_static
  // appears after the first sensor messages in bag recording order.
  {
    rosbag::View tfStaticView;
    for (const auto& bag : bag_reader_->bags)
      tfStaticView.addQuery(*bag, rosbag::TopicQuery(std::vector<std::string>({"/tf_static"})));

    int nTfStatic = 0;
    for (const auto& rosmsg : tfStaticView)
    {
      const auto tfs = rosmsg.instantiate<tf2_msgs::TFMessage>();
      if (!tfs) continue;
      for (const auto& tf : tfs->transforms)
      {
        try
        {
          tfBuffer_->setTransform(toRos2Transform(tf), "bagfile", true /*isStatic*/);
          nTfStatic++;
        }
        catch (const tf2::TransformException& ex)
        {
          MRPT_LOG_ERROR_STREAM("Pre-scan /tf_static: " << ex.what());
        }
      }
    }
    if (nTfStatic > 0)
      MRPT_LOG_INFO_STREAM(
          "Pre-scanned " << nTfStatic << " static transform(s) from /tf_static. "
                         << "Known frames: " << tfBuffer_->allFramesAsString());
    else
      MRPT_LOG_WARN("No /tf_static messages found in the bag. Sensor poses will rely on /tf only.");
  }

  // Pre-scan the ground-truth topic (if any), e.g. messages coming from a
  // separate, GT-only bag file merged above, to build a full trajectory_t
  // exposed via the mola::OfflineDatasetSource ground-truth API
  // (hasGroundTruthTrajectory() / getGroundTruthTrajectory()), in addition to
  // the normal per-timestep publishing as a regular observation (if the same
  // topic is also listed under "sensors").
  if (!ground_truth_topic_.empty())
  {
    if (topic2type.count(ground_truth_topic_) == 0)
    {
      MRPT_LOG_WARN_STREAM(
          "ground_truth_topic '" << ground_truth_topic_
                                 << "' was given but does not exist in the input bag(s).");
    }
    else
    {
      const std::string& gtType = topic2type.at(ground_truth_topic_);

      rosbag::View gtView;
      for (const auto& bag : bag_reader_->bags)
      {
        gtView.addQuery(*bag, rosbag::TopicQuery(std::vector<std::string>({ground_truth_topic_})));
      }

      size_t nGtPoses = 0;
      for (const auto& rosmsg : gtView)
      {
        mrpt::poses::CPose3D    pose;
        mrpt::Clock::time_point tim;

        if (gtType == "geometry_msgs/PoseStamped")
        {
          const auto m = rosmsg.instantiate<geometry_msgs::PoseStamped>();
          if (!m) continue;
          const auto& q = m->pose.orientation;
          if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
              !std::isfinite(q.w))
          {
            // Some datasets contain a few malformed GT entries (e.g. NaN
            // quaternion from a degenerate pose-graph node): skip them
            // rather than aborting the whole pre-scan.
            MRPT_LOG_THROTTLE_WARN_FMT(
                5.0, "Skipping ground-truth pose with non-finite quaternion on topic '%s'.",
                ground_truth_topic_.c_str());
            continue;
          }
          pose = mrpt::ros1bridge::fromROS(m->pose);
          tim  = mrpt::ros1bridge::fromROS(m->header.stamp);
        }
        else if (gtType == "nav_msgs/Odometry")
        {
          const auto m = rosmsg.instantiate<nav_msgs::Odometry>();
          if (!m) continue;
          const auto& q = m->pose.pose.orientation;
          if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
              !std::isfinite(q.w))
          {
            MRPT_LOG_THROTTLE_WARN_FMT(
                5.0, "Skipping ground-truth pose with non-finite quaternion on topic '%s'.",
                ground_truth_topic_.c_str());
            continue;
          }
          pose = mrpt::ros1bridge::fromROS(m->pose).mean;
          tim  = mrpt::ros1bridge::fromROS(m->header.stamp);
        }
        else
        {
          MRPT_LOG_THROTTLE_WARN_FMT(
              5.0,
              "ground_truth_topic '%s' has unsupported message type '%s' "
              "(supported: geometry_msgs/PoseStamped, nav_msgs/Odometry).",
              ground_truth_topic_.c_str(), gtType.c_str());
          break;
        }

        groundTruthTrajectory_.insert(tim, pose);
        nGtPoses++;
      }

      if (nGtPoses > 0)
        MRPT_LOG_INFO_STREAM(
            "Pre-scanned " << nGtPoses << " ground-truth pose(s) from '" << ground_truth_topic_
                           << "' into the GT trajectory.");
      else
        MRPT_LOG_WARN_STREAM(
            "ground_truth_topic '" << ground_truth_topic_ << "' yielded no usable GT poses.");
    }
  }

  // Begin of code adapted from "Transcriber" class from rosbag2rawlog:

  // Either follow the user-provided "sensors" YAML list, or build it
  // automatically from the list of sensors:
  mrpt::containers::yaml sensorsYaml;

  if (cfg.has("sensors"))
  {
    // Get from the user config:
    ASSERT_(cfg["sensors"].isSequence());

    std::stringstream ss;
    cfg["sensors"].printAsYAML(ss);
    sensorsYaml = mrpt::containers::yaml::FromStream(ss);
  }
  else
  {
    MRPT_LOG_INFO("Automatically building list of mapped topics:");

    // create list automatically:
    sensorsYaml = mrpt::containers::yaml::Sequence();

    for (const auto& [topic, topicType] : topic2type)
    {
      auto itType = mapTopic2Class.find(topicType);
      if (itType == mapTopic2Class.end())
      {
        MRPT_LOG_INFO_FMT(
            "- Skipped %25s (%30s): no known mapping to MOLA", topic.c_str(), topicType.c_str());
        continue;
      }

      mrpt::containers::yaml s = mrpt::containers::yaml::Map();

      s["topic"] = topic;
      s["type"]  = itType->second;

      sensorsYaml.push_back(s);

      MRPT_LOG_INFO_FMT(
          "- ADDED   %25s (%30s): as %s", topic.c_str(), topicType.c_str(), itType->second.c_str());
    }
  }

  // Start creating topic observers for /tf and /tf_static:
  lookup_["/tf"].emplace_back([this](const rosbag::MessageInstance& rosmsg)
                              { return toTf<false>(rosmsg); });
  lookup_["/tf_static"].emplace_back([this](const rosbag::MessageInstance& rosmsg)
                                     { return toTf<true>(rosmsg); });

  for (auto& sensorNode : sensorsYaml.asSequence())
  {
    const auto&       sensor = sensorNode.asMap();
    const std::string topic  = sensor.at("topic").as<std::string>();

    std::string sensorLabel = topic;
    if (sensor.count("sensorLabel") != 0)
    {
      sensorLabel = sensor.at("sensorLabel").as<std::string>();
    }

    // Map to MOLA class: auto or manual:
    std::string sensorType;

    if (sensor.count("type") != 0)
    {
      sensorType = sensor.at("type").as<std::string>();
    }
    else
    {
      if (topic2type.count(topic) == 0)
      {
        MRPT_LOG_INFO_FMT(
            "'sensors' contains topic '%s' with no explicit 'type' field, but there are no such "
            "messages in the rosbag: it will be ignored.",
            topic.c_str());
      }
      else
      {
        auto itType = mapTopic2Class.find(topic2type.at(topic));
        if (itType == mapTopic2Class.end())
        {
          THROW_EXCEPTION_FMT(
              "'sensors' contains topic '%s' without a 'type' entry, but could not automatically "
              "determine its mapping to mrpt::obs classes.",
              topic.c_str());
        }
        sensorType = itType->second;
      }
    }

    // Optional: fixed sensorPose (then ignores/don't need "tf" data):
    std::optional<mrpt::poses::CPose3D> fixedSensorPose;
    if (sensor.count("fixed_sensor_pose") != 0 && (sensor.count("use_fixed_sensor_pose") == 0 ||
                                                   sensor.at("use_fixed_sensor_pose").as<bool>()))
    {
      fixedSensorPose = mrpt::poses::CPose3D::FromString(
          "["s + sensor.at("fixed_sensor_pose").as<std::string>() + "]"s);
    }

    // Optional: some drivers stamp messages with an internal clock never
    // synced to the recording PC's wall clock, which breaks GT time lookups.
    // See the doc comment on toPointCloud2() for details.
    bool useBagRecordTime = false;
    if (sensor.count("use_bag_record_time") != 0)
    {
      useBagRecordTime = sensor.at("use_bag_record_time").as<bool>();
    }

    if (sensorType == "CObservationPointCloud")
    {
      // Both sensor_msgs/PointCloud2 and livox_ros_driver(2)/CustomMsg map here;
      // pick the right converter by checking the actual ROS type in the bag.
      const std::string rosType = topic2type.count(topic) ? topic2type.at(topic) : "";
      if (rosType == "livox_ros_driver/CustomMsg" || rosType == "livox_ros_driver2/CustomMsg")
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        {
          return catchExceptions(
              [=]()
              { return toLivoxCustomMsg(sensorLabel, m, fixedSensorPose, useBagRecordTime); });
        };
        lookup_[topic].emplace_back(callback);
      }
      else
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        {
          return catchExceptions(
              [=]() { return toPointCloud2(sensorLabel, m, fixedSensorPose, useBagRecordTime); });
        };
        lookup_[topic].emplace_back(callback);
      }
    }
    else if (sensorType == "CObservationImage")
    {
      // Both sensor_msgs/Image and sensor_msgs/CompressedImage map here;
      // pick the right converter by checking the actual ROS type in the bag.
      const std::string rosType = topic2type.count(topic) ? topic2type.at(topic) : "";
      if (rosType == "sensor_msgs/CompressedImage")
      {
        auto callback = [=](const rosbag::MessageInstance& m) {
          return catchExceptions([=]()
                                 { return toCompressedImage(sensorLabel, m, fixedSensorPose); });
        };
        lookup_[topic].emplace_back(callback);
      }
      else
      {
        auto callback = [=](const rosbag::MessageInstance& m)
        { return catchExceptions([=]() { return toImage(sensorLabel, m, fixedSensorPose); }); };
        lookup_[topic].emplace_back(callback);
      }
    }

    else if (sensorType == "CObservation2DRangeScan")
    {
      auto callback = [=](const rosbag::MessageInstance& m)
      { return catchExceptions([=]() { return toLidar2D(sensorLabel, m, fixedSensorPose); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationRotatingScan")
    {
      auto callback = [=](const rosbag::MessageInstance& m) {
        return catchExceptions([=]() { return toRotatingScan(sensorLabel, m, fixedSensorPose); });
      };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationIMU")
    {
      auto callback = [=](const rosbag::MessageInstance& m)
      { return catchExceptions([=]() { return toIMU(sensorLabel, m, fixedSensorPose); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationGPS")
    {
      auto callback = [=](const rosbag::MessageInstance& m)
      { return catchExceptions([=]() { return toGPS(sensorLabel, m, fixedSensorPose); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationOdometry")
    {
      auto callback = [=](const rosbag::MessageInstance& m)
      { return catchExceptions([=]() { return toOdometry(sensorLabel, m); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (sensorType == "CObservationRobotPose")
    {
      auto callback = [=](const rosbag::MessageInstance& m)
      { return catchExceptions([=]() { return toPoseStamped(sensorLabel, m); }); };
      lookup_[topic].emplace_back(callback);
    }
    else if (!sensorType.empty())
    {
      THROW_EXCEPTION_FMT(
          "Unsupported sensor type '%s' for topic '%s'", sensorType.c_str(), topic.c_str());
    }

    MRPT_LOG_INFO_FMT(
        "Installing handler for topic '%s' as '%s'", topic.c_str(), sensorType.c_str());

  }  // end for each "sensor"

  // Initialize the sequential read cursor:
  bag_reader_->iter             = bag_reader_->full_view.begin();
  bag_reader_->end              = bag_reader_->full_view.end();
  bag_reader_->iter_initialized = true;

  initialized_ = true;
  MRPT_END
}  // end initialize()

void Rosbag1Dataset::spinOnce()
{
  using mrpt::system::timeDifference;

  ASSERTMSG_(initialized_, "You must call initialize() first");

  MRPT_START
  ProfilerEntry tleg(profiler_, "spinOnce");

  const auto tNow = mrpt::Clock::now();

  // Starting time:
  if (!last_play_wallclock_time_)
  {
    last_play_wallclock_time_ = tNow;
  }

  // get current replay time:
  auto         lckUIVars       = mrpt::lockHelper(dataset_ui_mtx_);
  const double time_warp_scale = time_warp_scale_;
  const bool   paused          = paused_;
  const auto   teleport_here   = teleport_here_;
  teleport_here_.reset();
  lckUIVars.unlock();

  double dt = mrpt::system::timeDifference(*last_play_wallclock_time_, tNow) * time_warp_scale;
  last_play_wallclock_time_ = tNow;

  if (!rosbag_begin_time_ && bagMessageCount_ > 0)
  {
    doReadAhead(0, true /* skip read ahead buffer */);
    rosbag_begin_time_ = read_ahead_.at(0)->timestamp;
  }

  // override by an special teleport order?
  if (teleport_here.has_value() && *teleport_here < bagMessageCount_)
  {
    if (*teleport_here > rosbag_next_idx_write_)
    {
      MRPT_LOG_INFO_STREAM("Request to fast-forward ('teleport') to timestep: " << *teleport_here);

      rosbag_next_idx_ = *teleport_here;
      doReadAhead(rosbag_next_idx_, true /* skip read ahead buffer */);

      // this will force a reset with the first valid timestamp.
      last_dataset_time_ = 0;
    }
    else
    {
      MRPT_LOG_WARN_STREAM(
          "IGNORING order to go backwards in time to index="
          << *teleport_here << " due to limitation of the sequential rosbag reader.");
    }
  }
  else
  {
    if (paused)
    {
      return;
    }
    // move forward replayed dataset time:
    last_dataset_time_ += dt;
  }

  if (rosbag_next_idx_ >= read_ahead_.size())
  {
    onDatasetPlaybackEnds();  // notify base class

    MRPT_LOG_THROTTLE_INFO(
        10.0,
        "End of dataset reached! Nothing else to publish (CTRL+C to "
        "quit)");
    return;
  }

  MRPT_LOG_THROTTLE_INFO_FMT(
      5.0, "Dataset replay progress: %lu / %lu  (%4.02f%%)",
      static_cast<unsigned long>(rosbag_next_idx_), static_cast<unsigned long>(bagMessageCount_),
      (100.0 * rosbag_next_idx_) / bagMessageCount_);

  // Publish observations up to current time:
  for (;;)
  {
    if (rosbag_next_idx_ >= rosbag_next_idx_write_)
    {
      doReadAhead(rosbag_next_idx_);
    }

    // EOF?
    if (rosbag_next_idx_ >= read_ahead_.size())
    {
      break;
    }

    // current dataset entry:
    auto& de = read_ahead_.at(rosbag_next_idx_);
    ASSERT_(de.has_value());

    // Already past the time?
    // First rawlog timestamp?
    if (auto& de_tim = de->timestamp; de_tim)
    {
      if (!rosbag_begin_time_)
      {
        rosbag_begin_time_ = de_tim.value();
      }

      double thisTim = timeDifference(*rosbag_begin_time_, de_tim.value());

      // mechanism to detect mis-timestamped datasets:
      // e.g. good sensors mixed with LiDARs with timestamps starting
      //      in UNIX epoch.
      if (std::abs(thisTim - last_dataset_time_) > 1e9)
      {
        rosbag_begin_time_ = de_tim.value();
        thisTim            = .0;
        last_dataset_time_ = thisTim;

        MRPT_LOG_THROTTLE_WARN(
            2.0,
            "Apparently mis-timestamped sensors: resetting time "
            "reference. Please, fix your sensor timestamps.");
      }

      // Reset time after a "teleport"?
      if (last_dataset_time_ == 0)
      {
        last_dataset_time_ = thisTim;
      }

      // end of playback for now?
      if (last_dataset_time_ < thisTim)
      {
        break;
      }
    }

    // Send observations out:
    if (SF::Ptr sf = de->obs; sf)
    {
      for (const auto& obs : *sf)
      {
        this->sendObservationsToFrontEnds(obs);

        if (already_pub_sensor_labels_.count(obs->sensorLabel) == 0)
        {
          already_pub_sensor_labels_.insert(obs->sensorLabel);
          MRPT_LOG_INFO_STREAM(
              "Starting streaming of '" << obs->sensorLabel << "' ("
                                        << obs->GetRuntimeClass()->className
                                        << ") from the rosbag");
        }

        MRPT_LOG_DEBUG_STREAM(
            "Publishing " << obs->GetRuntimeClass()->className
                          << " sensorLabel: " << obs->sensorLabel << " for t=" << last_dataset_time_
                          << " observation timestamp="
                          << mrpt::system::dateTimeLocalToString(obs->timestamp));
      }
    }

    // Move on:
    rosbag_next_idx_++;
  }

  {
    auto lck = mrpt::lockHelper(dataset_ui_mtx_);

    last_used_tim_index_ = rosbag_next_idx_;
  }

  MRPT_END
}

void Rosbag1Dataset::doReadAhead(const std::optional<size_t>& requestedIndex, bool skipBufferAhead)
{
  MRPT_START

  ASSERT_(initialized_);

  // ensure we have observation data at the desired read point, plus a few
  // more:
  const auto startIdx = rosbag_next_idx_write_;

  ASSERT_GT_(read_ahead_length_, 0);

  // End of read segment:
  size_t endIdx = 0;
  if (requestedIndex)
  {
    if (skipBufferAhead)
    {
      endIdx = *requestedIndex;
    }
    else
    {
      endIdx = *requestedIndex + read_ahead_length_;
    }
  }
  else
  {
    endIdx = rosbag_next_idx_ + read_ahead_length_;
  }

  mrpt::saturate<size_t>(endIdx, 0, read_ahead_.size() - 1);

  for (size_t idx = startIdx; idx <= endIdx; idx++)
  {
    unload_queue_.push_back(idx);  // mark as recently accessed

    if (read_ahead_.at(idx).has_value())
    {
      continue;  // already read:
    }

    // The sequential reader can only move forward; idx must be the next one:
    ASSERT_EQUAL_(rosbag_next_idx_write_, idx);
    rosbag_next_idx_write_++;

    ASSERT_(bag_reader_->iter_initialized);
    ASSERT_(bag_reader_->iter != bag_reader_->end);

    const rosbag::MessageInstance rosmsg = *bag_reader_->iter;
    ++bag_reader_->iter;

    if (skipBufferAhead && idx != endIdx)
    {
      // Still process tf messages even when fast-forwarding so the transform
      // buffer stays populated regardless of skip distance:
      const auto topic = rosmsg.getTopic();
      if (topic == "/tf" || topic == "/tf_static") to_mrpt(rosmsg);
      continue;
    }

    SF::Ptr sf = to_mrpt(rosmsg);
    ASSERT_(sf);

    DatasetEntry& de = read_ahead_.at(idx).emplace();

    de.obs = sf;

    if (!sf->empty())
    {
      de.timestamp = sf->getObservationByIndex(0)->timestamp;
    }
  }

  // and also, unload() very old observations.
  autoUnloadOldEntries();

  MRPT_END
}

// See docs in base class:
size_t Rosbag1Dataset::datasetSize() const
{
  ASSERTMSG_(initialized_, "You must call initialize() first");

  return bagMessageCount_;
}

mrpt::obs::CSensoryFrame::Ptr Rosbag1Dataset::datasetGetObservations(size_t timestep) const
{
  ASSERTMSG_(initialized_, "You must call initialize() first");

  {
    auto lck             = mrpt::lockHelper(dataset_ui_mtx_);
    last_used_tim_index_ = timestep;
  }

  auto& me = const_cast<Rosbag1Dataset&>(*this);

  me.doReadAhead(timestep);

  ASSERT_(read_ahead_.at(timestep).has_value());

  return read_ahead_.at(timestep)->obs;
}

bool Rosbag1Dataset::findOutSensorPose(
    mrpt::poses::CPose3D& des, const std::string& frame, const std::string& referenceFrame,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose, const std::string_view label)
{
  if (fixedSensorPose)
  {
    des = fixedSensorPose.value();
    return true;
  }

  try
  {
    geometry_msgs::msg::TransformStamped ref_to_trgFrame =
        tfBuffer_->lookupTransform(referenceFrame, frame, {} /*latest value*/);

    tf2::Transform tf;
    tf2::fromMsg(ref_to_trgFrame.transform, tf);
    des = mrpt::ros1bridge::fromROS(tf);

    MRPT_LOG_DEBUG_FMT(
        "[findOutSensorPose] Found pose %s -> %s: %s", referenceFrame.c_str(), frame.c_str(),
        des.asString().c_str());

    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    // This is expected for messages that arrive before their /tf data, or when
    // the configured 'base_link_frame_id' does not match the bag's frames.
    // Avoid throwing here (it would be very slow due to backtrace generation
    // when it happens for many messages): just warn (throttled) and let the
    // caller drop this single observation.
    MRPT_LOG_THROTTLE_WARN_FMT(
        5.0,
        "[findOutSensorPose] Could not look up transform '%s' <- '%s' (label='%s'): %s\n"
        "Dropping affected observations until the transform becomes available. "
        "Currently known tf frames:\n%s",
        referenceFrame.c_str(), frame.c_str(), std::string(label).c_str(), ex.what(),
        tfBuffer_->allFramesAsString().c_str());
    return false;
  }
}

Rosbag1Dataset::Obs Rosbag1Dataset::toPointCloud2(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose, bool useBagRecordTime)
{
  const auto pts = rosmsg.instantiate<sensor_msgs::PointCloud2>();
  ASSERT_(pts);

  auto ptsObs         = mrpt::obs::CObservationPointCloud::Create();
  ptsObs->sensorLabel = label;
  ptsObs->timestamp   = useBagRecordTime ? mrpt::ros1bridge::fromROS(rosmsg.getTime())
                                         : mrpt::ros1bridge::fromROS(pts->header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      ptsObs->sensorPose, pts->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};  // tf not yet available: drop this observation (warning already logged)
  }

  // Convert points:
  std::set<std::string> fields = mrpt::ros1bridge::extractFields(*pts);

  // We need X Y Z:
  if (!fields.count("x") || !fields.count("y") || !fields.count("z"))
  {
    return {};
  }

  if (fields.count("ring") || fields.count("time") || fields.count("timestamp") ||
      fields.count("t"))
  {
    // XYZIRT
    auto mrptPts       = mrpt::maps::CGenericPointsMap::Create();
    ptsObs->pointcloud = mrptPts;

    if (!mrpt::ros1bridge::fromROS(*pts, *mrptPts))
    {
      THROW_EXCEPTION("Could not convert pointcloud from ROS to CGenericPointsMap");
    }

    // Fix timestamps for Livox driver:
    // It uses doubles for timestamps, but they are actually nanoseconds!
    auto ts =
        mrptPts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
    ASSERT_(ts);
    if (!ts->empty())
    {
      const auto [minIt, maxIt] = std::minmax_element(ts->begin(), ts->end());
      const float time_span     = *maxIt - *minIt;
      if (time_span > 1e5F)
      {
        // they must be nanoseconds, convert to seconds:
        for (auto& t : *ts)
        {
          t *= 1e-9F;
        }
      }
    }

    // converted ok:
    return {ptsObs};
  }

  if (fields.count("intensity"))
  {
    // XYZI
    auto mrptPts       = mrpt::maps::CGenericPointsMap::Create();
    ptsObs->pointcloud = mrptPts;

    if (!mrpt::ros1bridge::fromROS(*pts, *mrptPts))
    {
      MRPT_LOG_ONCE_WARN(
          "Could not convert pointcloud from ROS to "
          "CGenericPointsMap. Trying with XYZ");
    }
    else
    {  // converted ok:
      return {ptsObs};
    }
  }

  {
    // XYZ
    auto mrptPts       = mrpt::maps::CSimplePointsMap::Create();
    ptsObs->pointcloud = mrptPts;

    if (!mrpt::ros1bridge::fromROS(*pts, *mrptPts))
    {
      THROW_EXCEPTION("Could not convert pointcloud from ROS to CSimplePointsMap");
    }
  }

  return {ptsObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toLivoxCustomMsg(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose, bool useBagRecordTime)
{
  // instantiate<>() matches by MD5 sum, not by type name, and
  // livox_ros_driver2/CustomMsg shares the exact same field layout and MD5
  // sum as livox_ros_driver/CustomMsg, so this vendored struct deserializes
  // both message types.
  const auto msg = rosmsg.instantiate<livox_ros_driver::CustomMsg>();
  ASSERT_(msg);

  auto ptsObs         = mrpt::obs::CObservationPointCloud::Create();
  ptsObs->sensorLabel = label;
  ptsObs->timestamp   = useBagRecordTime ? mrpt::ros1bridge::fromROS(rosmsg.getTime())
                                         : mrpt::ros1bridge::fromROS(msg->header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      ptsObs->sensorPose, msg->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};  // tf not yet available: drop this observation (warning already logged)
  }

  auto mrptPts       = mrpt::maps::CGenericPointsMap::Create();
  ptsObs->pointcloud = mrptPts;

  mrptPts->registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  mrptPts->registerField_uint16(mrpt::maps::CPointsMap::POINT_FIELD_RING_ID);
  mrptPts->registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  const size_t numPoints = msg->points.size();
  mrptPts->resize(numPoints);

  for (size_t i = 0; i < numPoints; i++)
  {
    const auto& pt = msg->points[i];

    mrptPts->setPointFast(i, pt.x, pt.y, pt.z);
    mrptPts->setPointField_float(i, mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY, pt.reflectivity);
    mrptPts->setPointField_uint16(i, mrpt::maps::CPointsMap::POINT_FIELD_RING_ID, pt.line);
    // offset_time is in nanoseconds, relative to the scan's header.stamp:
    mrptPts->setPointField_float(
        i, mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP, pt.offset_time * 1e-9F);
  }

  return {ptsObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toLidar2D(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  const auto scan = rosmsg.instantiate<sensor_msgs::LaserScan>();
  ASSERT_(scan);

  auto scanObs = mrpt::obs::CObservation2DRangeScan::Create();

  // Extract sensor pose from tf frames, if enabled:
  mrpt::poses::CPose3D sensorPose;
  mrpt::ros1bridge::fromROS(*scan, sensorPose, *scanObs);

  scanObs->sensorLabel = label;
  scanObs->timestamp   = mrpt::ros1bridge::fromROS(scan->header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      scanObs->sensorPose, scan->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};  // tf not yet available: drop this observation (warning already logged)
  }

  return {scanObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toRotatingScan(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  const auto pts = rosmsg.instantiate<sensor_msgs::PointCloud2>();
  ASSERT_(pts);

  // Convert points:
  std::set<std::string> fields = mrpt::ros1bridge::extractFields(*pts);

  // We need X Y Z and ring:
  if (!fields.count("x") || !fields.count("y") || !fields.count("z") || !fields.count("ring"))
  {
    return {};
  }

  // As a structured 2D range image, if we have ring numbers:
  auto                       obsRotScan = mrpt::obs::CObservationRotatingScan::Create();
  const mrpt::poses::CPose3D sensorPose;

  if (!mrpt::ros1bridge::fromROS(*pts, *obsRotScan, sensorPose))
  {
    THROW_EXCEPTION(
        "Could not convert pointcloud from ROS to "
        "CObservationRotatingScan. Trying another format.");
  }

  obsRotScan->sensorLabel = label;
  obsRotScan->timestamp   = mrpt::ros1bridge::fromROS(pts->header.stamp);

  bool sensorPoseOK = findOutSensorPose(
      obsRotScan->sensorPose, pts->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};  // tf not yet available: drop this observation (warning already logged)
  }

  return {obsRotScan};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toIMU(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  const auto imu = rosmsg.instantiate<sensor_msgs::Imu>();
  ASSERT_(imu);

  auto imuObs = mrpt::obs::CObservationIMU::Create();

  imuObs->sensorLabel = label;
  imuObs->timestamp   = mrpt::ros1bridge::fromROS(imu->header.stamp);

  // Convert data:
  mrpt::ros1bridge::fromROS(*imu, *imuObs);

  bool sensorPoseOK = findOutSensorPose(
      imuObs->sensorPose, imu->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};  // tf not yet available: drop this observation (warning already logged)
  }

  return {imuObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toGPS(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  const auto gps = rosmsg.instantiate<sensor_msgs::NavSatFix>();
  ASSERT_(gps);

  auto gpsObs = mrpt::obs::CObservationGPS::Create();

  gpsObs->sensorLabel = label;
  gpsObs->timestamp   = mrpt::ros1bridge::fromROS(gps->header.stamp);

  // Convert data:
  mrpt::ros1bridge::fromROS(*gps, *gpsObs);

  bool sensorPoseOK = findOutSensorPose(
      gpsObs->sensorPose, gps->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};  // tf not yet available: drop this observation (warning already logged)
  }

  return {gpsObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toOdometry(
    std::string_view label, const rosbag::MessageInstance& rosmsg)
{
  const auto odo = rosmsg.instantiate<nav_msgs::Odometry>();
  ASSERT_(odo);

  auto mrptObs = mrpt::obs::CObservationOdometry::Create();

  mrptObs->sensorLabel = label;
  mrptObs->timestamp   = mrpt::ros1bridge::fromROS(odo->header.stamp);

  // Convert data:
  const auto pose   = mrpt::ros1bridge::fromROS(odo->pose);
  mrptObs->odometry = {pose.mean.x(), pose.mean.y(), pose.mean.yaw()};

  mrptObs->hasVelocities       = true;
  mrptObs->velocityLocal.vx    = odo->twist.twist.linear.x;
  mrptObs->velocityLocal.vy    = odo->twist.twist.linear.y;
  mrptObs->velocityLocal.omega = odo->twist.twist.angular.z;

  return {mrptObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toPoseStamped(
    std::string_view label, const rosbag::MessageInstance& rosmsg)
{
  const auto poseMsg = rosmsg.instantiate<geometry_msgs::PoseStamped>();
  ASSERT_(poseMsg);

  auto mrptObs = mrpt::obs::CObservationRobotPose::Create();

  mrptObs->sensorLabel = label;
  mrptObs->timestamp   = mrpt::ros1bridge::fromROS(poseMsg->header.stamp);

  mrptObs->pose.mean = mrpt::ros1bridge::fromROS(poseMsg->pose);

  return {mrptObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toImage(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  const auto image = rosmsg.instantiate<sensor_msgs::Image>();
  ASSERT_(image);

  auto imgObs = mrpt::obs::CObservationImage::Create();

  imgObs->sensorLabel = label;
  imgObs->timestamp   = mrpt::ros1bridge::fromROS(image->header.stamp);

  // Manual conversion sensor_msgs/Image -> mrpt::img::CImage, so we do not
  // depend on cv_bridge (which would require its ROS2 message types):
  imgObs->image = imageFromROS(*image);

  bool sensorPoseOK = findOutSensorPose(
      imgObs->cameraPose, image->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};  // tf not yet available: drop this observation (warning already logged)
  }

  return {imgObs};
}

Rosbag1Dataset::Obs Rosbag1Dataset::toCompressedImage(
    std::string_view label, const rosbag::MessageInstance& rosmsg,
    const std::optional<mrpt::poses::CPose3D>& fixedSensorPose)
{
  const auto image = rosmsg.instantiate<sensor_msgs::CompressedImage>();
  ASSERT_(image);

  // cv::imdecode handles JPEG, PNG, and most other formats automatically.
  const cv::Mat compressed(
      1, static_cast<int>(image->data.size()), CV_8UC1,
      const_cast<unsigned char*>(image->data.data()));
  cv::Mat decoded = cv::imdecode(compressed, cv::IMREAD_ANYCOLOR);

  if (decoded.empty())
  {
    THROW_EXCEPTION_FMT(
        "cv::imdecode failed for CompressedImage on topic '%s' (format='%s')",
        std::string(label).c_str(), image->format.c_str());
  }

  // imdecode returns BGR; convert to the channel count MRPT expects:
  const bool isColor = (decoded.channels() == 3);
  if (decoded.channels() == 4)
  {
    cv::cvtColor(decoded, decoded, cv::COLOR_BGRA2BGR);
  }

  auto imgObs         = mrpt::obs::CObservationImage::Create();
  imgObs->sensorLabel = label;
  imgObs->timestamp   = mrpt::ros1bridge::fromROS(image->header.stamp);
  imgObs->image.loadFromMemoryBuffer(
      static_cast<unsigned int>(decoded.cols), static_cast<unsigned int>(decoded.rows), isColor,
      decoded.data, false /*already BGR*/);

  bool sensorPoseOK = findOutSensorPose(
      imgObs->cameraPose, image->header.frame_id, base_link_frame_id_, fixedSensorPose, label);
  if (!sensorPoseOK)
  {
    return {};
  }

  return {imgObs};
}

template <bool isStatic>
Rosbag1Dataset::Obs Rosbag1Dataset::toTf(const rosbag::MessageInstance& rosmsg)
{
  const auto tfs = rosmsg.instantiate<tf2_msgs::TFMessage>();
  if (!tfs)
  {
    return {};
  }

  for (const auto& tf : tfs->transforms)
  {
    try
    {
      tfBuffer_->setTransform(toRos2Transform(tf), "bagfile", isStatic);
    }
    catch (const tf2::TransformException& ex)
    {
      MRPT_LOG_ERROR_STREAM(ex.what());
    }
  }
  return {};
}

Rosbag1Dataset::SF::Ptr Rosbag1Dataset::to_mrpt(const rosbag::MessageInstance& rosmsg)
{
  auto rets = Rosbag1Dataset::SF::Create();

  const auto topic = rosmsg.getTopic();

  if (auto search = lookup_.find(topic); search != lookup_.end())
  {
    for (const auto& callback : search->second)
    {
      auto obs = callback(rosmsg);

      for (const auto& o : obs)
      {  // insert observation:
        rets->insert(o);
      }
    }
  }
  else
  {
    if (unhandledTopics_.count(topic) == 0)
    {
      unhandledTopics_.insert(topic);
      MRPT_LOG_WARN_STREAM("Warning: unhandled topic '" << topic << "'");
    }
  }
  return rets;
}  // end to_mrpt()

Rosbag1Dataset::Obs Rosbag1Dataset::catchExceptions(const std::function<Obs()>& f)
{
  try
  {
    return f();
  }
  catch (const std::exception& e)
  {
    MRPT_LOG_ERROR_STREAM(
        "Exception while processing topic message (ignore if the error "
        "stops later on, e.g. missing /tf):\n"
        << e.what());
    return {};
  }
}

void Rosbag1Dataset::autoUnloadOldEntries() const
{
  const size_t MAX_UNLOAD_LEN = std::max<size_t>(10, 2 * read_ahead_length_);

  // unload() very old observations.
  while (unload_queue_.size() > MAX_UNLOAD_LEN)
  {
    const auto idx = unload_queue_.front();
    unload_queue_.erase(unload_queue_.begin());

    // Free memory in read-ahead buffer:
    read_ahead_.at(idx).reset();
  }
}
