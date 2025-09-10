#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry/draw.hpp>
#include <imgproc/gradient_adapter.hpp>
#include <imgproc/derivative_gradient.hpp>
#include <imgproc/susan.hpp>
#include <imgproc/rcmg.hpp>
#include <imgproc/quadratureG2.hpp>
#include <imgproc/quadratureS.hpp>
#include <imgproc/quadratureSF.hpp>
#include <imgproc/quadratureLGF.hpp>
#include <imgproc/pc_sqf.hpp>
#include <imgproc/pc_lgf.hpp>
#include <imgproc/pc_matlab.hpp>
#include <imgproc/image_operator.hpp>
#include <utility/matlab_helpers.hpp>

#include <filesystem>
#include <algorithm>  
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#define WRITE_IMAGE_FILES
//#define SHOW_IMAGES

using namespace lsfm;
using namespace std;
namespace fs = std::filesystem;


std::string path = "./orientation/", fname;
int stepsi = -1, steps = 45;
double magTh = 0.1;

constexpr int ENTRY_HALF_RANGE = 1;

template<class GT, class MT = int, class DT = double>
struct Entry {
    Entry() :  flags(0) {}
    Entry(const cv::Ptr<GradientI<uchar, GT, MT, DT>>& a, const std::string& b, int f = 0)
        : grad(a), name(b), flags(f) {}
    
    cv::Ptr<GradientI<uchar, GT, MT, DT>> grad;
    std::string name;
    int flags;

    inline bool hr() const {
        return flags & ENTRY_HALF_RANGE;
    }

    inline cv::Mat process(const cv::Mat src) {
        grad->process(src);
        return grad->magnitude();
    }

    cv::Mat fixGrad(const cv::Mat &m) const {
        cv::Mat mag;
        if (m.type() != CV_32F)
            m.convertTo(mag, CV_32F);
        else
            m.copyTo(mag);

        double vmin, vmax;
        cv::minMaxIdx(mag, &vmin, &vmax);
        // laplace
        if (vmin < 0) {
            vmax = std::max(vmax, -vmin);
            mag = cv::abs(mag);
            mag *= 255.0 / vmax;
        }
        else {
            mag -= vmin;
            mag *= 255.0 / (vmax - vmin);
        }
        mag.convertTo(mag, CV_8U);
        return mag;
    }
};

double error(const cv::Mat &gt, const cv::Mat &dir, bool hr = false) {
    cv::Mat res = cv::abs(gt - dir);
    cv::subtract(2 * CV_PI, res, res, res > CV_PI);
    if (hr)
        cv::subtract(CV_PI, res, res, res > CV_PI / 2);
#ifdef WRITE_IMAGE_FILES
    cv::Mat write = res / CV_PI * 2550;
    write.convertTo(write, CV_8U);
    if (stepsi < 0)
        cv::imwrite(path + fname + "-error.png", write);
    else
        cv::imwrite(path + boost::str(boost::format("%04d-") % stepsi) + fname + "-error.png", write);
#endif
#ifdef SHOW_IMAGES
    imshow("error", res / CV_PI * 10);
    cv::waitKey();
#endif
    return cv::sum(res)[0];
}


template<class GT, class MT = int, class DT = float>
double testSingle(Entry<GT, MT, DT> &e, const cv::Mat &img, const cv::Mat &mask, double rot = 0, double noise = 0) {

    RotateOperator<double> op_rot(rot, cv::Point2d(127, 127), cv::INTER_CUBIC);
    RotateOperator<double> op_rot_mask(rot, cv::Point2d(127, 127), cv::INTER_NEAREST);
    GaussianNoiseOperator op_noise(noise);
    
    cv::Mat gt(mask.size(),CV_64F);
    gt.setTo(0);

    gt.setTo(-rot, mask == 1);
    gt.setTo(-rot + CV_PI/2, mask == 2);

    if (e.hr()) {
        gt.setTo(-rot, mask == 3);
        gt.setTo(-rot + CV_PI / 2, mask == 4);
    }
    else
    {
        gt.setTo(-rot + CV_PI, mask == 3);
        gt.setTo(-rot -  CV_PI / 2, mask == 4);
    }

    cv::Mat nimg;
    img.copyTo(nimg);

    //cv::imshow("img", img);

    cv::Mat nmask;
    mask.copyTo(nmask);
    if (rot > 0) {
        op_rot(nimg);
        op_rot_mask(nmask);
        op_rot_mask(gt);
    }

    int num = static_cast<int>(cv::sum(gt > 0)[0]) / 255;
    if (noise > 0)
        op_noise(nimg);

    e.grad->process(nimg);
    cv::Mat dir = e.grad->direction();
    dir.setTo(0, nmask == 0);

#ifdef WRITE_IMAGE_FILES
    cv::imwrite(path + boost::str(boost::format("%04d-") % stepsi) + fname + ".png", nimg);
    cv::imwrite(path + boost::str(boost::format("%04d-") % stepsi) + fname + "-mask.png", nmask > 0);
#endif
#ifdef SHOW_IMAGES
    cv::imshow("img", nimg);
    cv::imshow("mask", nmask > 0);
    //cv::imshow("gt", gt > 0);
    cv::imshow("cA", quiverDir<DT, uchar>(nimg, gt, Range<double>(-CV_PI, CV_PI), nmask > 0, 2, 2, 1, 8, 1));
    cv::imshow("cB", quiverDir<DT, uchar>(nimg, dir, Range<double>(-CV_PI, CV_PI), nmask > 0, 2, 2, 1, 8, 1));
    //cv::imshow(e.name + "-mask", nmask > 0);
    std::cout << rot << std::endl;
#endif
    

    return error(gt, dir, e.hr()) / num;
    
}

cv::Mat boxImage() {
    cv::Mat img(256, 256, CV_8U);
    img.setTo(85);
    img.rowRange(47, 208).colRange(47, 208).setTo(170);
    return img;
}

template<class GT, class MT = int, class DT = float>
double testBox(Entry<GT,MT,DT> &e, cv::Mat img, cv::Mat mask, double blur = 0, double noise = 0) {
    GaussianBlurOperator op_blur(blur);
    
    if (blur > 0)
        op_blur(img);
        
    // remove borders dependong on blur
    int ks = cvRound(blur * 6 + 1) | 1;
    // make sure that border has at least the size of greatest filter kernel (7x7)
    if (ks < 9)
        ks = 9;
    mask.rowRange(47 - ks, 47 + ks).colRange(47 - ks, 47 + ks).setTo(0);
    mask.rowRange(207 - ks, 207 + ks).colRange(47 - ks, 47 + ks).setTo(0);
    mask.rowRange(47 - ks, 47 + ks).colRange(207 - ks, 207 + ks).setTo(0);
    mask.rowRange(207 - ks, 207 + ks).colRange(207 - ks, 207 + ks).setTo(0);

    cv::Mat tmp = mask.rowRange(47 + ks, 207 - ks).colRange(47 - ks, 47 + ks);
    tmp.setTo(1, tmp > 0);

    tmp = mask.rowRange(47 - ks, 47 + ks).colRange(47 + ks, 207 - ks);
    tmp.setTo(2, tmp > 0);

    tmp = mask.rowRange(47 + ks, 207 - ks).colRange(207 - ks, 207 + ks);
    tmp.setTo(3, tmp > 0);

    tmp = mask.rowRange(207 - ks, 207 + ks).colRange(47 + ks, 207 - ks);
    tmp.setTo(4, tmp > 0);

    double sum = 0;
    for (stepsi = 0; stepsi <= steps; ++stepsi)
        sum += testSingle(e, img, mask, stepsi * CV_PI / (4 *steps), noise);
    
    return sum / steps;
}

cv::Mat diskImage() {
    cv::Mat img(512, 512, CV_8U);
    img.setTo(85);
    cv::circle(img, cv::Point(256, 256), 150, 170, -1, cv::LINE_AA);
    return img;
}

template<class GT, class MT = int, class DT = double>
double testDisk(Entry<GT, MT, DT> &e, cv::Mat img, cv::Mat mask, double blur = 0, double noise = 0) {
    stepsi = -1;
    GaussianBlurOperator op_blur(blur);
    GaussianNoiseOperator op_noise(noise);
    
    cv::Mat gt(512, 512, CV_64F);
    for (int y = 0; y != 512; ++y)
        for (int x = 0; x != 512; ++x)
            gt.at<double>(y, x) = std::atan2(static_cast<double>(256-y), static_cast<double>(256-x));

    if (e.hr()) {
        cv::add(gt, CV_PI, gt, gt < -CV_PI / 2);
        cv::subtract(gt, CV_PI, gt, gt > CV_PI / 2);
    }  

    if (blur > 0)
        op_blur(img);
    
    int num = static_cast<int>(cv::sum(mask)[0]) / 255;
    gt.setTo(0, mask == 0);

    if (noise > 0)
        op_noise(img);
    
    e.grad->process(img);
    cv::Mat dir = e.grad->direction();
    dir.setTo(0, mask == 0);

#ifdef WRITE_IMAGE_FILES
    cv::imwrite(path + fname + ".png", img);
    cv::imwrite(path + fname + "-mask.png", mask);
#endif
#ifdef SHOW_IMAGES
    cv::imshow("img", img);
    cv::imshow("mask", mask > 0);
    cv::imshow("A", quiverDir<DT, uchar>(img, gt, Range<double>(-CV_PI, CV_PI), mask, 4, 4, 1, 8, 1));
    cv::imshow("B", quiverDir<DT, uchar>(img, dir, Range<double>(-CV_PI, CV_PI), mask, 4, 4, 1, 8, 1));
    //cv::imshow(e.name + "-mask", mask > 0);
    
    //cv::Mat res = e.fixGrad(e.process(img));
    //imshow(e.name + " - grad", res);
    //imshow(e.name + " - grad", quiverDir<DT, MT>(res, e.grad->direction(), e.grad->directionRange(), e.grad->magnitude(), 1, 1, 4, 1, 0.5));
#endif
    return error(gt, dir) / num;
}


int main(int argc, char** argv)
{  
    std::vector<Entry<short>> gradI;
    gradI.push_back(Entry<short>(new SusanGradient<short, int, double>, "Susan (37)", ENTRY_HALF_RANGE));
    gradI.push_back(Entry<short>(new SusanGradient<short, int, double>(20,true), "Susan (3x3)", ENTRY_HALF_RANGE));
    gradI.push_back(Entry<short>(new RCMGradient<uchar, 1, short, int, double>(3,1), "RMG (3x3)",ENTRY_HALF_RANGE));
    gradI.push_back(Entry<short>(new RCMGradient<uchar, 1, short, int, double>(5, 3), "RMG (5x5)", ENTRY_HALF_RANGE));

    std::vector<Entry<double, double>> gradF;
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, RobertsDerivative>, "Roberts (2x2)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, PrewittDerivative>, "Prewitt (3x3)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, ScharrDerivative>, "Scharr (3x3)"));

    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, SobelDerivative>, "Sobel (3x3)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, SobelDerivative>({ NV("grad_kernel_size",5)}), "Sobel (5x5)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, SobelDerivative>({ NV("grad_kernel_size",7) }), "Sobel (7x7)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, SobelDerivative>({ NV("grad_kernel_size",9) }), "Sobel (9x9)"));
    
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, GaussianDerivative>({ NV("grad_kernel_size",3), NV("grad_range",1.5) }), "Gauss (3x3)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, GaussianDerivative>({ NV("grad_kernel_size",5), NV("grad_range",2.3) }), "Gauss (5x5)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, GaussianDerivative>({ NV("grad_kernel_size",7), NV("grad_range",3.0) }), "Gauss (7x7)"));
    gradF.push_back(Entry<double, double>(new DerivativeGradient<uchar, double, double, double, GaussianDerivative>({ NV("grad_kernel_size",9), NV("grad_range",3.5) }), "Gauss (9x9)"));

    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureG2<uchar, double>>({ NV("grad_kernel_size",3), NV("grad_kernel_spacing",1.24008) }), "QF StG (3x3)", ENTRY_HALF_RANGE));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureG2<uchar, double>>({ NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.008) }), "QF StG (5x5)", ENTRY_HALF_RANGE));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureG2<uchar, double>>({ NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 0.873226) }), "QF StG (7x7)", ENTRY_HALF_RANGE));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureG2<uchar, double>>({ NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 0.781854) }), "QF StG (9x9)", ENTRY_HALF_RANGE));

    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureS<uchar, double, double>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 3), NV("grad_kernel_spacing", 1.2) }), "SQF Po (3x3)"));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureS<uchar, double, double>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 5), NV("grad_kernel_spacing", 1.2) }), "SQF Po (5x5)"));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureS<uchar, double, double>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 7), NV("grad_kernel_spacing", 1.2) }), "SQF Po (7x7)"));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureS<uchar, double, double>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_size", 9), NV("grad_kernel_spacing", 1.2) }), "SQF Po (9x9)"));

    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureLGF<uchar, double>>({ NV("grad_waveLength", 5), NV("grad_sigmaOnf", 0.55) }), "SQFF LG"));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureSF<uchar, double>>({ NV("grad_scale", 1), NV("grad_muls", 2), NV("grad_kernel_spacing", 1.2) }), "SQFF Po12"));
    gradF.push_back(Entry<double, double>(new GradientOdd<QuadratureSF<uchar, double>>({ NV("grad_scale", 1), NV("grad_muls", 3), NV("grad_kernel_spacing", 2.5) }), "SQFF Po"));

    gradF.push_back(Entry<double, double>(new GradientEnergy<PCLgf<uchar, double>>({ NV("grad_waveLength", 3), NV("grad_sigmaOnf", 0.55) }), "PCF Lg"));
    gradF.push_back(Entry<double, double>(new GradientOdd<PCSqf<uchar, double>>({ NV("grad_scale", 1), NV("grad_muls", 3), NV("grad_kernel_spacing", 2.5) }), "PCF Po"));

    int cols = gradI.size() + gradF.size() + 1;
    int rows = 7;
    std::vector<std::vector<std::string>> table_box, table_disk;
    table_box.resize(cols); table_disk.resize(cols);
    for_each(table_box.begin(), table_box.end(), [&](std::vector<std::string> &row) {
        row.resize(rows);
    });

    for_each(table_disk.begin(), table_disk.end(), [&](std::vector<std::string> &row) {
        row.resize(rows);
    });

    std::cout << "none...";
    table_box[0][0] = "Method";
    table_box[0][1] = "none";

    table_disk[0][0] = "Method";
    table_disk[0][1] = "none";

    DerivativeGradient<uchar, double, double, double, SobelDerivative> sobel;
    cv::Mat box = boxImage();
    sobel.process(box);
    cv::Mat box_mask = sobel.magnitude() > sobel.magnitudeThreshold(magTh);
     
    cv::Mat disk = diskImage();
    sobel.process(disk);
    cv::Mat disk_mask = sobel.magnitude() > sobel.magnitudeThreshold(magTh);


    int row = 1;
    for_each(gradI.begin(), gradI.end(), [&](Entry<short> &e) {
        table_box[row][0] = e.name;
        table_disk[row][0] = e.name;
        fname = "box-"+ e.name;
        table_box[row][1] = boost::str(boost::format("%.3f") % testBox(e,box.clone(),box_mask.clone(),0,0));
        fname = "disk-"+ e.name;
        table_disk[row++][1] = boost::str(boost::format("%.3f") % testDisk(e,disk.clone(),disk_mask.clone(),0,0));
    });
    for_each(gradF.begin(), gradF.end(), [&](Entry<double, double> &e) {
        table_box[row][0] = e.name;
        table_disk[row][0] = e.name;
        fname = "box-" + e.name;
        table_box[row][1] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 0, 0));
        fname = "disk-" + e.name;
        table_disk[row++][1] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 0, 0));
    });

    std::cout << "done" << std::endl << "blur...";
    table_box[0][2] = "blur";
    table_disk[0][2] = "blur";

    row = 1;
    for_each(gradI.begin(), gradI.end(), [&](Entry<short> &e) {
        fname = "box-blur" + e.name;
        table_box[row][2] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 2, 0));
        fname = "disk-blur" + e.name;
        table_disk[row++][2] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 2, 0));
    });
    for_each(gradF.begin(), gradF.end(), [&](Entry<double, double> &e) {
        fname = "box-blur-"+ e.name;
        table_box[row][2] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 2, 0));
        fname = "disk-blur-"+ e.name;
        table_disk[row++][2] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 2, 0));
    });

    std::cout << "done" << std::endl << "noise 10...";
    table_box[0][3] = "noise10";
    table_disk[0][3] = "noise10";

    row = 1;
    for_each(gradI.begin(), gradI.end(), [&](Entry<short> &e) {
        fname = "box-noise10-"+ e.name;
        table_box[row][3] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 0, 10));
        fname = "disk-noise10-"+ e.name;
        table_disk[row++][3] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 0, 10));
    });
    for_each(gradF.begin(), gradF.end(), [&](Entry<double, double> &e) {
        fname = "box-noise10-"+ e.name;
        table_box[row][3] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 0, 10));
        fname = "disk-noise10-" + e.name;
        table_disk[row++][3] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 0, 10));
    });

    std::cout << "done" << std::endl << "noise 40...";
    table_box[0][4] = "noise40";
    table_disk[0][4] = "noise40";

    row = 1;
    for_each(gradI.begin(), gradI.end(), [&](Entry<short> &e) {
        fname = "box-noise40-" + e.name;
        table_box[row][4] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 0, 40));
        fname = "disk-noise40-" + e.name;
        table_disk[row++][4] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 0, 40));
    });
    for_each(gradF.begin(), gradF.end(), [&](Entry<double, double> &e) {
        fname = "box-noise40-" + e.name;
        table_box[row][4] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 0, 40));
        fname = "disk-noise40-" + e.name;
        table_disk[row++][4] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 0, 40));
    });

    std::cout << "done" << std::endl << "blur + noise 10...";
    table_box[0][5] = "bnoise10";
    table_disk[0][5] = "bnoise10";

    row = 1;
    for_each(gradI.begin(), gradI.end(), [&](Entry<short> &e) {
        fname = "box-bnoise10-" + e.name;
        table_box[row][5] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 2, 10));
        fname = "disk-bnoise10-" + e.name;
        table_disk[row++][5] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 2, 10));
    });
    for_each(gradF.begin(), gradF.end(), [&](Entry<double, double> &e) {
        fname = "box-bnoise10-" + e.name;
        table_box[row][5] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 2, 10));
        fname = "disk-bnoise10-" + e.name;
        table_disk[row++][5] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 2, 10));
    });

    std::cout << "done" << std::endl << "blur + noise 40...";
    table_box[0][6] = "bnoise40";
    table_disk[0][6] = "bnoise40";

    row = 1;
    for_each(gradI.begin(), gradI.end(), [&](Entry<short> &e) {
        fname = "box-bnoise40-" + e.name;
        table_box[row][6] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 2, 40));
        fname = "disk-bnoise40-" + e.name;
        table_disk[row++][6] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 2, 40));
    });
    for_each(gradF.begin(), gradF.end(), [&](Entry<double, double> &e) {
        fname = "box-bnoise40-" + e.name;
        table_box[row][6] = boost::str(boost::format("%.3f") % testBox(e, box.clone(), box_mask.clone(), 2, 40));
        fname = "disk-bnoise40-" + e.name;
        table_disk[row++][6] = boost::str(boost::format("%.3f") % testDisk(e, disk.clone(), disk_mask.clone(), 2, 40));
    });
    std::cout << "done" << std::endl;
    
    std::ofstream ofs;
    ofs.open("gradient_orientation_box.csv");

    for_each(table_box.begin(), table_box.end(), [&](const std::vector<std::string> &row) {
        for_each(row.begin(), row.end(), [&](const std::string &cell) {
            std::cout << cell << "\t";
            ofs << cell << ";";
        });
        std::cout << std::endl;
        ofs << std::endl;
    });

    ofs.close();

    ofs.open("gradient_orientation_disk.csv");

    for_each(table_disk.begin(), table_disk.end(), [&](const std::vector<std::string> &row) {
        for_each(row.begin(), row.end(), [&](const std::string &cell) {
            std::cout << cell << "\t";
            ofs << cell << ";";
        });
        std::cout << std::endl;
        ofs << std::endl;
    });

    ofs.close();

    cv::waitKey();
    
    return 0;
}

