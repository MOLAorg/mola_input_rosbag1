/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   rosbag1-info.cpp
 * @brief  CLI tool: an equivalent to ROS1's "rosbag info" for ROS1 .bag files,
 *         reusing the vendored rosbag_storage reader so it runs in a pure
 *         ROS2 environment (no ROS1 install required).
 * @author Jose Luis Blanco Claraco
 * @date   Jun 8, 2026
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// Vendored ROS1 rosbag reader (no ROS1 install required):
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace
{
/// Human-readable byte size, e.g. "1.3 GB".
std::string humanSize(uint64_t bytes)
{
  const char*  units[] = {"B", "KB", "MB", "GB", "TB"};
  double       v       = static_cast<double>(bytes);
  unsigned int u       = 0;
  while (v >= 1024.0 && u < 4)
  {
    v /= 1024.0;
    u++;
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(u == 0 ? 0 : 1) << v << " " << units[u];
  return ss.str();
}

/// Human-readable duration, e.g. "1:23s (83.0s)".
std::string humanDuration(double seconds)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1) << seconds << "s";
  if (seconds >= 60.0)
  {
    const int total = static_cast<int>(seconds);
    const int h     = total / 3600;
    const int m     = (total % 3600) / 60;
    const int s     = total % 60;
    ss << " (";
    if (h > 0)
    {
      ss << h << "hr ";
    }
    if (h > 0 || m > 0)
    {
      ss << m << "min ";
    }
    ss << s << "s)";
  }
  return ss.str();
}

/// Wall-clock date string for a UNIX epoch time (local time).
std::string humanTime(double epochSeconds)
{
  const auto secs = static_cast<std::time_t>(epochSeconds);
  std::tm    tmv{};
  localtime_r(&secs, &tmv);
  char buf[64];
  std::strftime(buf, sizeof(buf), "%b %d %Y %H:%M:%S", &tmv);
  const double       frac = epochSeconds - static_cast<double>(secs);
  std::ostringstream ss;
  ss << buf << "." << std::setw(2) << std::setfill('0')
     << static_cast<int>(std::round(frac * 100.0)) << " (" << std::fixed << std::setprecision(2)
     << epochSeconds << ")";
  return ss.str();
}

struct TopicStats
{
  std::string datatype;
  uint64_t    count   = 0;
  double      tFirst  = 0;
  double      tLast   = 0;
  bool        hasTime = false;
};
}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " <input.bag>\n"
              << "\n"
              << "Prints summary information (topics, types, timing, size) of a\n"
              << "ROS 1 .bag file, similar to ROS 1's `rosbag info`. Works in a\n"
              << "pure ROS 2 / non-ROS environment (the ROS 1 bag reader is\n"
              << "vendored into this package).\n";
    return 1;
  }

  const std::string filename = argv[1];

  try
  {
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Read);

    rosbag::View view;
    view.addQuery(bag);

    const uint32_t totalMsgs = view.size();

    // Per-topic statistics, plus per-datatype md5sum (from connections):
    std::map<std::string, TopicStats>  topics;
    std::map<std::string, std::string> type2md5;

    const std::vector<const rosbag::ConnectionInfo*> connections = view.getConnections();
    for (const auto* c : connections)
    {
      topics[c->topic].datatype = c->datatype;
      type2md5[c->datatype]     = c->md5sum;
    }

    // Iterate (metadata only: no message body deserialization) to count
    // messages per topic and gather first/last timestamps:
    double begin = 0, end = 0;
    bool   hasGlobalTime = false;
    for (const rosbag::MessageInstance& m : view)
    {
      const double t = m.getTime().toSec();
      auto&        s = topics[m.getTopic()];
      if (s.datatype.empty())
      {
        s.datatype = m.getDataType();
      }
      if (!s.hasTime)
      {
        s.tFirst = s.tLast = t;
        s.hasTime          = true;
      }
      else
      {
        s.tFirst = std::min(s.tFirst, t);
        s.tLast  = std::max(s.tLast, t);
      }
      s.count++;

      if (!hasGlobalTime)
      {
        begin = end   = t;
        hasGlobalTime = true;
      }
      else
      {
        begin = std::min(begin, t);
        end   = std::max(end, t);
      }
    }

    const double duration = hasGlobalTime ? (end - begin) : 0.0;

    // ---- Print, mimicking `rosbag info` ----
    std::cout << std::left;
    std::cout << std::setw(13) << "path:" << bag.getFileName() << "\n";
    std::cout << std::setw(13) << "version:" << bag.getMajorVersion() << "."
              << bag.getMinorVersion() << "\n";
    std::cout << std::setw(13) << "duration:" << humanDuration(duration) << "\n";
    if (hasGlobalTime)
    {
      std::cout << std::setw(13) << "start:" << humanTime(begin) << "\n";
      std::cout << std::setw(13) << "end:" << humanTime(end) << "\n";
    }
    std::cout << std::setw(13) << "size:" << humanSize(bag.getSize()) << "\n";
    std::cout << std::setw(13) << "messages:" << totalMsgs << "\n";

    {
      const char* comp = "none";
      switch (bag.getCompression())
      {
        case rosbag::compression::Uncompressed:
          comp = "none";
          break;
        case rosbag::compression::BZ2:
          comp = "bz2";
          break;
        case rosbag::compression::LZ4:
          comp = "lz4";
          break;
      }
      std::cout << std::setw(13) << "compression:" << comp << "\n";
    }

    // types:
    {
      bool first = true;
      for (const auto& [type, md5] : type2md5)
      {
        std::cout << std::setw(13) << (first ? "types:" : "") << type << " [" << md5 << "]\n";
        first = false;
      }
    }

    // topics: aligned columns (name, count, frequency, type)
    {
      // Compute column widths:
      size_t wName = 0, wCount = 0;
      for (const auto& [name, s] : topics)
      {
        wName  = std::max(wName, name.size());
        wCount = std::max(wCount, std::to_string(s.count).size());
      }

      bool first = true;
      for (const auto& [name, s] : topics)
      {
        std::ostringstream line;
        line << std::left << std::setw(static_cast<int>(wName)) << name << "   " << std::right
             << std::setw(static_cast<int>(wCount)) << s.count << " msgs";

        // frequency, only meaningful with >1 timestamped messages:
        const double span = s.hasTime ? (s.tLast - s.tFirst) : 0.0;
        if (s.count > 1 && span > 0.0)
        {
          const double hz = static_cast<double>(s.count - 1) / span;
          line << " @ " << std::fixed << std::setprecision(1) << std::setw(7) << hz << " Hz";
        }
        else
        {
          line << "          ";  // keep the type column aligned
        }
        line << " : " << s.datatype;

        std::cout << std::left << std::setw(13) << (first ? "topics:" : "") << line.str() << "\n";
        first = false;
      }
    }

    bag.close();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error reading bag '" << filename << "':\n" << e.what() << "\n";
    return 2;
  }

  return 0;
}
