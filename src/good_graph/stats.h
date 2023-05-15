/**
 * @file stats.h
 * @author Yanwei Du (duyanwei0702@gmail.com)
 * @brief None
 * @version 0.1
 * @date 05-08-2023
 * @copyright Copyright (c) 2023
 */

#ifndef COLMAP_GOOD_GRAPH_STATS_H_
#define COLMAP_GOOD_GRAPH_STATS_H_

#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace colmap {
namespace good_graph {

/**
 * @brief
 *
 * @tparam T
 */
template <typename T>
class StatsT {
 public:
  using StatsType = StatsT<T>;

  /**
   * @brief Construct a new Stats T object
   *
   */
  StatsT(const std::string& name = "Stats", const std::string& file = "")
      : name_(name),
        file_(file),
        count_(0u),
        sum_(T(0)),
        min_(std::numeric_limits<T>::max()),
        max_(std::numeric_limits<T>::lowest()),
        last_(T(0)) {
    data_.reserve(1000000);
  }

  /**
   * @brief Destroy the Stats T object
   *
   */
  ~StatsT() {
    if (!file_.empty()) {
      std::ofstream myfile(file_);
      myfile << *this;
      myfile.close();
    }
  }

  /**
   * @brief
   *
   * @param v
   */
  void add(const T& v) {
    ++count_;
    sum_ += v;
    min_ = std::min(min_, v);
    max_ = std::max(max_, v);
    last_ = v;
    data_.emplace_back(v);
  }

  /**
   * @brief
   *
   * @return uint64_t
   */
  uint64_t count() const { return count_; }

  /**
   * @brief
   *
   * @return T
   */
  T sum() const { return sum_; }

  /**
   * @brief
   *
   * @return T
   */
  T min() const { return min_; }

  /**
   * @brief
   *
   * @return T
   */
  T max() const { return max_; }

  /**
   * @brief
   *
   * @return T
   */
  T mean() const {
    return count_ > 0u ? sum_ / static_cast<double>(count_) : T(0);
  }

  /**
   * @brief
   *
   * @return T
   */
  T last() const { return last_; }

  /**
   * @brief
   *
   * @param other
   * @return StatsType&
   */
  StatsType& operator+=(const StatsType& other) {
    if (other.count() > 0u) {
      count_ += other.count();
      sum_ += other.sum();
      min_ = std::min(min_, other.min());
      max_ = std::max(max_, other.max());
      last_ = other.last();
      std::copy(other.data_.cbegin(), other.data_.cend(),
                std::back_inserter(data_));
    }
    return *this;
  }

  /**
   * @brief
   *
   * @param lhs
   * @param rhs
   * @return StatsType
   */
  friend StatsType operator+(StatsType& lhs, const StatsType& rhs) {
    return lhs += rhs;
  }

  /**
   * @brief
   *
   * @param os
   * @param s
   * @return std::ostream&
   */
  friend std::ostream& operator<<(std::ostream& os, const StatsType& s) {
    os << std::setprecision(10);
    os << "# --- " << s.name_ << " ---\n"
       << "# "
       << "count: " << s.count_ << "\t"
       << "sum: " << s.sum_ << "\t"
       << "mean: " << s.mean() << "\t"
       << "min: " << s.min() << "\t"
       << "max: " << s.max();
    return os;
  }

  void write(const std::string& file) const {
    std::ofstream myfile(file);
    myfile << *this << "\n";
    myfile << std::setprecision(10);
    for (auto e : data_) {
      myfile << e << "\n";
    }
    myfile.close();
  }

 private:
  std::string name_;
  std::string file_;
  uint64_t count_;
  T sum_;
  T min_;
  T max_;
  T last_;
  std::vector<T> data_;
};

using Statsf = StatsT<float>;
using Statsd = StatsT<double>;

/**
 * @brief
 *
 */
template <typename T = double>
class SummaryT {
 public:
  using Ptr = std::shared_ptr<SummaryT>;
  using StatsType = StatsT<T>;

  /**
   * @brief
   *
   * @param name
   * @param t
   */
  void update(const std::string& name, T t) {
    if (dict_.count(name) == 0) {
      dict_[name] = StatsType(name);
    }
    dict_[name].add(t);
  }

  /**
   * @brief
   *
   * @return std::string
   */
  std::string toString() const {
    std::stringstream ss;
    ss << "# --- StatsSummary ---\n";
    for (const auto& d : dict_) {
      ss << d.second << "\n";
    }
    return ss.str();
  }

  /**
   * @brief
   *
   */
  void report() const { std::cout << toString() << std::endl; }

  /**
   * @brief
   *
   * @param prefix
   */
  void write(const std::string& prefix) const {
    std::ofstream myfile(prefix + "_summary.txt");
    myfile << toString();
    myfile.close();

    for (const auto& e : dict_) {
      e.second.write(prefix + "_" + e.first + ".txt");
    }
  }

 private:
  std::unordered_map<std::string, StatsType> dict_;
};

}  // namespace good_graph

}  // namespace colmap

#endif  // COLMAP_GOOD_GRAPH_STATS_H_
