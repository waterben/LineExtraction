#include <utility/performance.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

namespace lsfm {

    PerformanceData::PerformanceData(const std::string& n, const cv::Mat& s) : TaskData(n, s) {
        if (src.channels() == 3)
          cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
        else {
            src_gray = src;
            cv::cvtColor(src_gray, src, CV_GRAY2RGB);
        }
    }

    PerformanceResult PerformanceMeasure::computeResult(const std::vector<uint64>& data) {
        PerformanceResult ret;
        if (data.empty())
            return ret;
        uint64 sum = 0, sqrSum = 0;
        for_each(data.begin(), data.end(), [&](uint64 measure) {
            sum += measure;
            sqrSum += measure * measure;
        });
        ret.total = static_cast<double>(sum * 1000) / cv::getTickFrequency();
        ret.mean = static_cast<double>(sum) / data.size();
        ret.stddev = std::sqrt(static_cast<double>(sqrSum) / data.size() - ret.mean * ret.mean);
        ret.mean = (ret.mean * 1000) / cv::getTickFrequency();
        ret.stddev = (ret.stddev * 1000) / cv::getTickFrequency();
        return ret;
    }

    PerformanceMeasure PerformanceTaskBase::accumulatedMeasure(const PerformanceMeasureVector& measure, const std::string sourceName, const std::string filterName) {
        PerformanceMeasure ret(sourceName, filterName);
        if (measure.empty())
            return ret;

        for_each(measure.begin(), measure.end(), [&](const PerformanceMeasure &pd) {
            ret.height += pd.height;
            ret.width += pd.width;
            ret.measures.insert(ret.measures.end(), pd.measures.begin(), pd.measures.end());
        });
        ret.width /= measure.size();
        ret.height /= measure.size();
        return ret;
    }

    void PerformanceTaskDefault::run(TaskData* data, int loops, bool verbose) {
        PerformanceData* pdata = dynamic_cast<PerformanceData*>(data);
        run(pdata->name, this->rgb() ? pdata->src : pdata->src_gray, loops, verbose);
    }

    TaskData* PerformanceTest::prepareTaskData(const std::string& src_name, cv::Mat& src) {
        prepareSource(src);
        return new PerformanceData(src_name, src);
    }

    void PerformanceTest::run(int runs, bool verbose) {
        std::cout << "Starting performance test: " << name << std::endl;
        uint64 start = cv::getTickCount();
        results_.clear();
        providerRecords_ = 0;
        std::string src_name;
        cv::Mat src;
        // make sure data is at beginning
        for_each(data.begin(), data.end(), [&](DataProviderPtr& d) {
            d->rewind();
        });

        size_t pos = 0;
        while (pos < data.size()) {
            // make sure task data is empty
            for_each(tasks.begin(), tasks.end(), [&](PerformanceTaskPtr& t) {
                t->measure.clear();
            });

            while (data[pos]->get(src_name, src)) {
                try {
                    if (verbose)
                        std::cout << "  Process source: " << src_name << std::endl;
                    std::unique_ptr<TaskData> data(prepareTaskData(src_name, src));
                    ++providerRecords_;
                    for_each(tasks.begin(), tasks.end(), [&, this](PerformanceTaskPtr& task) {
                        task->run(data.get(), runs, verbose);
                        if (this->visualResults)
                            task->saveResults(verbose);
                    });
                }
                catch (std::exception& e) {
                    std::cout << "\t\tException occured: " << e.what() << std::endl;
                }
                catch (...) {
                    std::cout << "\t\tUnknown exception occured!" << std::endl;
                }
            }
            for_each(tasks.begin(), tasks.end(), [&](PerformanceTaskPtr& t) {
                results_.push_back(DataProviderTaskMeasure(data[pos]->name, t->name, t->measure));
            });
            ++pos;
        }

        std::cout << "Performance test " << name << " done: " << static_cast<double>((cv::getTickCount() - start)) / cv::getTickFrequency() << "s" << std::endl;
    }

    void PerformanceTest::writeMeasure(const PerformanceMeasure& pm, StringTable& StringTable, size_t col, size_t row) {
        std::string name;
        if (col == 1) {
            name = pm.sourceName;
            if (showMegaPixel)
                name += boost::str(boost::format(" (%.2f)") % (pm.width * pm.height));
        }

        PerformanceResult res = pm.computeResult();
        if (showTotal) {
            if (col == 1)
                StringTable(row, 0) = "total:" + name;
            StringTable(row++, col) = boost::str(boost::format("%.3f") % (res.total));
        }
        if (showMean) {
            if (col == 1)
                StringTable(row, 0) = "mean:" + name;
            StringTable(row++, col) = boost::str(boost::format("%.3f") % (res.mean));
        }
        if (showStdDev) {
            if (col == 1)
                StringTable(row, 0) = "sdev:" + name;
            StringTable(row, col) = boost::str(boost::format("%.3f") % (res.stddev));
        }
    }

    StringTable PerformanceTest::resultTable(bool fullReport) {

        size_t rows_per_measure = (showTotal ? 1 : 0) + (showMean ? 1 : 0) + (showStdDev ? 1 : 0);
        // write tasks as cols, since it is much less than sources for full reports
        size_t cols = tasks.size() + 1;
        size_t rows = data.size() * rows_per_measure + 1;
        if (fullReport)
            rows += providerRecords_ * rows_per_measure;

        StringTable StringTable(rows, cols);
        StringTable(0, 0) = "Data";

        size_t col = 1;
        for_each(tasks.begin(), tasks.end(), [&](const PerformanceTaskPtr &e) {
            StringTable(0, col++) = e->name;
        });

        col = 1;
        size_t row = 1;
        // in results we have row by row since each task is a col and the tasks are always written in sequence
        for_each(results_.begin(), results_.end(), [&](const DataProviderTaskMeasure& data) {
            PerformanceMeasure pm = data.accumulatedResult();
            writeMeasure(pm, StringTable, col, row);
            if (fullReport) {
                for_each(data.results.begin(), data.results.end(), [&](const PerformanceMeasure& pm) {
                    writeMeasure(pm, StringTable, col, row);
                });
            }
            ++col;
            if (col >= cols) {
                row += rows_per_measure;
                col = 1;
            }
        });
        return StringTable;
    }

}
