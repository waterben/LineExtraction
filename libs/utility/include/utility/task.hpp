#pragma once


#include <opencv2/core.hpp>
#include <utility/string_table.hpp>
#include <utility/value_manager.hpp>

#include <filesystem>
#include <memory>


namespace lsfm {

constexpr int TASK_SQR = 1;
constexpr int TASK_RGB = 2;
constexpr int TASK_NO_3 = 4;
constexpr int TASK_NO_5 = 8;

struct TaskData {
  TaskData(const std::string& n = std::string(), const cv::Mat& s = cv::Mat()) : name(n), src(s) {}
  virtual ~TaskData() {}

  std::string name;
  cv::Mat src;
};

//! Base class for tasks
struct TaskBase {
  TaskBase(const std::string& taskName, int f = 0) : name(taskName), flags(f) {}
  virtual ~TaskBase() {}

  virtual void run(const TaskData& data, int loops, bool verbose) = 0;
  virtual void saveResults(bool verbose) {}

  // name of task
  std::string name;
  int flags;

  inline bool rgb() const { return (flags & TASK_RGB) != 0; }

  inline bool sqr() const { return (flags & TASK_SQR) != 0; }

  virtual int border() const {
    if (flags & TASK_NO_3) return -2;
    if (flags & TASK_NO_5) return -3;
    return 0;
  }

  virtual void value(const std::string& name, const lsfm::Value& value) {}
};

//! Data provider for performance test
struct DataProviderBase {
  DataProviderBase(const std::string& dataName) : name(dataName) {}
  virtual ~DataProviderBase() {}

  //! get next image data, return false if source is depleted
  virtual bool get(std::string& src_name, cv::Mat& src) = 0;
  //! rewind provider to initial data
  virtual void rewind() = 0;
  //! clear data
  virtual void clear() = 0;

  std::string name;
};

typedef std::shared_ptr<DataProviderBase> DataProviderPtr;
typedef std::vector<DataProviderPtr> DataProviderList;

//! Image data set for performance test
struct FileDataProvider : public DataProviderBase {
  FileDataProvider(std::string& name) : DataProviderBase(name) {}
  FileDataProvider(const std::filesystem::path& p, const std::string& name, bool recursive = true)
      : DataProviderBase(name), pos_(0) {
    parse(p, recursive);
  }
  FileDataProvider(const std::vector<std::filesystem::path>& f, const std::string& name, bool recursive = true)
      : DataProviderBase(name), pos_(0) {
    parse(f, recursive);
  }

  void parse(const std::vector<std::filesystem::path>& folders, bool recursive = true) {
    for_each(folders.begin(), folders.end(), [&, this](const std::filesystem::path& f) { this->parse(f, recursive); });
  }

  void parse(const std::filesystem::path& folder, bool recursive = true);

  //! get next image data, return false if source is depleted
  virtual bool get(std::string& src_name, cv::Mat& src);

  //! rewind provider to initial data
  virtual void rewind() { pos_ = 0; }

  virtual void clear() {
    files_.clear();
    pos_ = 0;
  }

 private:
  size_t pos_;
  std::vector<std::filesystem::path> files_;
};

//! Task loader
struct TaskLoader {
  TaskLoader(const std::string& n = std::string()) : name(n) {}
  virtual ~TaskLoader() {}

  std::string name;

  // run tasks
  virtual void run(int runs = 10, bool verbose = true) = 0;

  // get results
  virtual StringTable resultTable(bool fullReport = false) = 0;
};

typedef std::shared_ptr<TaskLoader> TaskLoaderPtr;
}  // namespace lsfm
