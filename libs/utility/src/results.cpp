#include <utility/results.hpp>

namespace lsfm {

    cv::Mat applyBorder(cv::Mat& inout, int border, int borderType, const cv::Scalar& color) {
        if (border < 0) {
            border = -border;
            inout.colRange(0, border).setTo(color);
            inout.colRange(inout.cols - border, inout.cols).setTo(color);
            inout.rowRange(0, border).setTo(color);
            inout.rowRange(inout.rows - border, inout.rows).setTo(color);
        }
        else if (border > 0)
            cv::copyMakeBorder(inout, inout, border, border, border, border, borderType, color);
        return inout;
    }

    cv::Mat applyBorderCopy(const cv::Mat& in, int border, int borderType, const cv::Scalar& color) {
        cv::Mat out;
        if (border < 0) {
            border = -border;
            out = in.clone();
            out.colRange(0, border).setTo(color);
            out.colRange(out.cols - border, out.cols).setTo(color);
            out.rowRange(0, border).setTo(color);
            out.rowRange(out.rows - border, out.rows).setTo(color);
        }
        else if (border > 0)
            cv::copyMakeBorder(in, out, border, border, border, border, borderType, color);
        else 
            out = in.clone();
        return out;
    }

    cv::Mat createNMS(const cv::Mat &emap) {

        cv::Mat emapImg;
        emapImg.create(emap.rows, emap.cols, CV_8UC3);

        emapImg.setTo(cv::Vec3b(0, 0, 0));
        emapImg.setTo(cv::Vec3b(220, 150, 255), emap == 7); // magenta2
        emapImg.setTo(cv::Vec3b(255, 0, 150), emap == 6); // lila
        emapImg.setTo(cv::Vec3b(255, 0, 0), emap == 5); // blue
        emapImg.setTo(cv::Vec3b(255, 255, 0), emap == 4); // cyan
        emapImg.setTo(cv::Vec3b(0, 255, 0), emap == 3); // green
        emapImg.setTo(cv::Vec3b(0, 255, 255), emap == 2); // yellow
        emapImg.setTo(cv::Vec3b(0, 150, 255), emap == 1); // orange
        emapImg.setTo(cv::Vec3b(0, 0, 255), emap == 0); // red
        return emapImg;
    }

    void saveEdge(const cv::Mat &data, const std::string &name, int border) {
        cv::Mat nms = createNMS(data);
        cv::imwrite(name + ".png", applyBorder(nms,border));
    }

    void saveNormalized(const cv::Mat &in, const std::string &name, int border) {
        cv::Mat data = applyBorderCopy(in, border);
        
        double vmin, vmax;
        cv::minMaxIdx(data, &vmin, &vmax);
        if (vmin < 0) {
            vmax = std::max(vmax, -vmin);
            cv::Mat tmp = cv::abs(data);
            tmp *= 255.0 / vmax;
            tmp.convertTo(tmp, CV_8U);
            cv::imwrite(name + "_abs.png", tmp);
            data += vmax;
            data *= 127.5 / vmax;
        }
        else
            data *= 255.0 / vmax;
        data.convertTo(data, CV_8U);
        cv::imwrite(name + ".png", data);
    }

    

    

}
