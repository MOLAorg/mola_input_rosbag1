/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Rosbag1Dataset.h
 * @brief  RawDataSource for datasets in ROS1 bag format, without a full ROS1 installation
 * @author Jose Luis Blanco Claraco
 * @date   May 20, 2025
 */
#pragma once

#include <mola_kernel/interfaces/Dataset_UI.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>

#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

// Forward declarations to isolate the vendored ROS1 / tf2 build dependencies,
// so that downstream code including this header does not need them:
namespace tf2
{
class BufferCore;
}
namespace rosbag
{
class MessageInstance;
}

namespace mola
{
/** RawDataSource for datasets in ROS1 bag format.
 *
 *  It reads a ROS1 bag file, and exposes it as a dataset
 *  with N entries, N being the number of messages in the bag.
 *  Reading them via the offline API (OfflineDatasetSource)
 *  returns empty shared_ptr observations for those messages
 *  that do not have a direct mapping to mrpt::obs classes.
 *  The dataset can be also played in an online (real-time, or with a custom
 *  time wrapping) fashion via the RawDataSourceBase API.
 *
 *  See example configuration files to see how to define what topics
 *  to publish, and how to optionally override the sensor poses in the local
 *  robot frame.
 *
 * \ingroup mola_input_rosbag1_grp */
class Rosbag1Dataset : public RawDataSourceBase, public OfflineDatasetSource, public Dataset_UI
{
  DEFINE_MRPT_OBJECT(Rosbag1Dataset, mola)

 public:
  Rosbag1Dataset();

  // See docs in base class
  void spinOnce() override;

  // See docs in base class:
  size_t datasetSize() const override;

  mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(size_t timestep) const override;

  // Virtual interface of Dataset_UI (see docs in derived class)
  size_t datasetUI_size() const override { return datasetSize(); }
  size_t datasetUI_lastQueriedTimestep() const override
  {
    auto lck = mrpt::lockHelper(dataset_ui_mtx_);
    return last_used_tim_index_;
  }
  double datasetUI_playback_speed() const override
  {
    auto lck = mrpt::lockHelper(dataset_ui_mtx_);
    return time_warp_scale_;
  }
  void datasetUI_playback_speed(double speed) override
  {
    auto lck         = mrpt::lockHelper(dataset_ui_mtx_);
    time_warp_scale_ = speed;
  }
  bool datasetUI_paused() const override
  {
    auto lck = mrpt::lockHelper(dataset_ui_mtx_);
    return paused_;
  }
  void datasetUI_paused(bool paused) override
  {
    auto lck = mrpt::lockHelper(dataset_ui_mtx_);
    paused_  = paused;
  }
  void datasetUI_teleport(size_t timestep) override
  {
    auto lck       = mrpt::lockHelper(dataset_ui_mtx_);
    teleport_here_ = timestep;
  }

 protected:
  // See docs in base class
  void initialize_rds(const Yaml& cfg) override;

 private:
  bool        initialized_ = false;
  std::string rosbag_filename_;
  std::string base_link_frame_id_ = "base_link";

  std::optional<mrpt::Clock::time_point> rosbag_begin_time_;
  size_t                                 read_ahead_length_ = 15;

  std::optional<mrpt::Clock::time_point> last_play_wallclock_time_;
  double                                 last_dataset_time_ = 0;

  struct BagInfo;
  std::shared_ptr<BagInfo> bag_reader_;
  size_t                   bagMessageCount_ = 0;

  using SF = mrpt::obs::CSensoryFrame;

  SF::Ptr to_mrpt(const rosbag::MessageInstance& rosmsg);

  void doReadAhead(
      const std::optional<size_t>& requestedIndex = std::nullopt, bool skipBufferAhead = false);

  // timestep in this class is just the index of the message in the rosbag:
  struct DatasetEntry
  {
    SF::Ptr obs;

    /// empty if obs == nullptr
    std::optional<mrpt::Clock::time_point> timestamp;
  };

  /** At initialization
   *
   */
  mutable std::vector<std::optional<DatasetEntry>> read_ahead_;
  size_t                                           rosbag_next_idx_       = 0;
  size_t                                           rosbag_next_idx_write_ = 0;
  std::set<std::string>                            already_pub_sensor_labels_;
  mutable std::deque<size_t>                       unload_queue_;  //!< read_ahead_ indices

  // Methods and variables for the ROS->MRPT conversion
  // -------------------------------------------------------
  using Obs = std::vector<mrpt::obs::CObservation::Ptr>;

  using CallbackFunction = std::function<Obs(const rosbag::MessageInstance&)>;

  std::map<std::string, std::vector<CallbackFunction>> lookup_;
  std::set<std::string>                                unhandledTopics_;

  std::shared_ptr<tf2::BufferCore> tfBuffer_;

  template <bool isStatic>
  Obs toTf(const rosbag::MessageInstance& rosmsg);

  Obs toPointCloud2(
      std::string_view label, const rosbag::MessageInstance& rosmsg,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

  Obs toLidar2D(
      std::string_view msg, const rosbag::MessageInstance& rosmsg,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

  Obs toRotatingScan(
      std::string_view msg, const rosbag::MessageInstance& rosmsg,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

  Obs toIMU(
      std::string_view msg, const rosbag::MessageInstance& rosmsg,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

  Obs toGPS(
      std::string_view msg, const rosbag::MessageInstance& rosmsg,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

  Obs toOdometry(std::string_view msg, const rosbag::MessageInstance& rosmsg);

  Obs toImage(
      std::string_view msg, const rosbag::MessageInstance& rosmsg,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

  Obs toCompressedImage(
      std::string_view msg, const rosbag::MessageInstance& rosmsg,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose);

  Obs catchExceptions(const std::function<Obs()>& f);

  void autoUnloadOldEntries() const;

  bool findOutSensorPose(
      mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
      const std::optional<mrpt::poses::CPose3D>& fixedSensorPose, const std::string_view label);

  mutable timestep_t    last_used_tim_index_ = 0;
  bool                  paused_              = false;
  double                time_warp_scale_     = 1.0;
  std::optional<size_t> teleport_here_;
  mutable std::mutex    dataset_ui_mtx_;
};

}  // namespace mola