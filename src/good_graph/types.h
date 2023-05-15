/**
 * @file types.h
 * @author Yanwei Du (duyanwei0702@gmail.com)
 * @brief None
 * @version 0.1
 * @date 05-08-2023
 * @copyright Copyright (c) 2023
 */

#include "good_graph/stats.h"

namespace colmap {
namespace good_graph {

struct GraphMetaInfo {
  double timecost = 0.0;
  double cam_num = 0;
  double lmk_num = 0;
  double obs_num = 0;

  GraphMetaInfo(double _timecost = 0.0, double _cam_num = 0,
                double _lmk_num = 0, double _obs_num = 0)
      : timecost(_timecost),
        cam_num(_cam_num),
        lmk_num(_lmk_num),
        obs_num(_obs_num) {}

  bool operator<(const GraphMetaInfo& other) const {
    return this->timecost < other.timecost;
  }

  GraphMetaInfo& operator+=(const GraphMetaInfo& other) {
    this->timecost += other.timecost;
    this->cam_num += other.cam_num;
    this->lmk_num += other.lmk_num;
    this->obs_num += other.obs_num;
    return *this;
  }

  GraphMetaInfo operator/(double value) const {
    return {this->timecost / value, this->cam_num / value,
            this->lmk_num / value, this->obs_num / value};
  }

  friend std::ostream& operator<<(std::ostream& os, const GraphMetaInfo& g) {
    os << std::setw(10) << std::setprecision(10);
    os << g.timecost << " " << g.cam_num << " " << g.lmk_num << " "
       << g.obs_num;
    return os;
  }
};

using GraphStats = StatsT<GraphMetaInfo>;
using GraphStatsSummary = SummaryT<GraphMetaInfo>;
using GraphStatsSummaryPtr = GraphStatsSummary::Ptr;

}  // namespace good_graph

}  // namespace colmap

namespace std {
template <>
class numeric_limits<colmap::good_graph::GraphMetaInfo> {
 public:
  static colmap::good_graph::GraphMetaInfo lowest() {
    return {-1, -1, -1, -1};
  };
  static colmap::good_graph::GraphMetaInfo max() {
    return {1e8, 1e8, 1e8, 1e8};
  };
};
}  // namespace std
