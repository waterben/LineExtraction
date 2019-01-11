#include "helpers.h"
#include <geometry/draw.hpp>
#include <ctime>

using namespace lsfm;
using namespace std;

ImageModes createPresets() {
    ImageModes presets;
    presets.push_back(ImageMode("Gray", QCPColorGradient::gpGrayscale));
    presets.push_back(ImageMode("Hot", QCPColorGradient::gpHot));
    presets.push_back(ImageMode("Cold", QCPColorGradient::gpCold));
    presets.push_back(ImageMode("Night", QCPColorGradient::gpNight));
    presets.push_back(ImageMode("Candy", QCPColorGradient::gpCandy));
    presets.push_back(ImageMode("Geography", QCPColorGradient::gpGeography));
    presets.push_back(ImageMode("Ion", QCPColorGradient::gpIon));
    presets.push_back(ImageMode("Thermal", QCPColorGradient::gpThermal));
    presets.push_back(ImageMode("Polar", QCPColorGradient::gpPolar));
    presets.push_back(ImageMode("Spectrum", QCPColorGradient::gpSpectrum));
    presets.push_back(ImageMode("Jet", QCPColorGradient::gpJet));
    presets.push_back(ImageMode("Hues", QCPColorGradient::gpHues));
    return presets;
}

ImageModes Detector::imageModePresets = createPresets();

ImageModes& edgeMapModes() {
    static ImageModes emap_modes;
    if (emap_modes.empty()) {
        emap_modes.push_back(ImageMode("8 Dir Color Grad", QCPColorGradient()));
        QCPColorGradient *emap_grad = &emap_modes.back().grad;

        emap_grad->clearColorStops();
        emap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
        emap_grad->setLevelCount(9);

        emap_grad->setColorStopAt(0, QColor(0, 0, 0));
        emap_grad->setColorStopAt(1.0 / 8, QColor(255, 0, 0));     // red 0
        emap_grad->setColorStopAt(2.0 / 8, QColor(255, 150, 0));   // orange 1
        emap_grad->setColorStopAt(3.0 / 8, QColor(255, 255, 0));   // yellow 2
        emap_grad->setColorStopAt(4.0 / 8, QColor(0, 255, 0));     // green 3
        emap_grad->setColorStopAt(5.0 / 8, QColor(0, 255, 255));   // cyan 4
        emap_grad->setColorStopAt(6.0 / 8, QColor(0, 0, 255));     // blue 5
        emap_grad->setColorStopAt(7.0 / 8, QColor(150, 0, 255));   // lila 6
        emap_grad->setColorStopAt(1, QColor(255, 0, 255)); // magenta 7

        emap_modes.push_back(ImageMode("8 Dir Color Line", QCPColorGradient()));
        emap_grad = &emap_modes.back().grad;

        emap_grad->clearColorStops();
        emap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
        emap_grad->setLevelCount(9);

        
        emap_grad->setColorStopAt(0, QColor(0, 0, 0));
        emap_grad->setColorStopAt(1.0 / 8, QColor(150, 0, 255));   // lila 6
        emap_grad->setColorStopAt(2.0 / 8, QColor(255, 0, 255)); // magenta 7
        emap_grad->setColorStopAt(3.0 / 8, QColor(255, 0, 0));     // red 0
        emap_grad->setColorStopAt(4.0 / 8, QColor(255, 150, 0));   // orange 1
        emap_grad->setColorStopAt(5.0 / 8, QColor(255, 255, 0));   // yellow 2
        emap_grad->setColorStopAt(6.0 / 8, QColor(0, 255, 0));     // green 3
        emap_grad->setColorStopAt(7.0 / 8, QColor(0, 255, 255));   // cyan 4
        emap_grad->setColorStopAt(1, QColor(0, 0, 255));     // blue 5
        

        emap_modes.push_back(ImageMode("BW", QCPColorGradient()));
        emap_grad = &emap_modes.back().grad;

        emap_grad->clearColorStops();
        emap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
        emap_grad->setLevelCount(3);

        emap_grad->setColorStopAt(0, QColor(0, 0, 0));
        emap_grad->setColorStopAt(1.0 / 8, QColor(255, 255, 255));
        emap_grad->setColorStopAt(1.0, QColor(255, 255, 255));
    }
    return emap_modes;
}

ImageModes& cannyMapMode() {
    static ImageModes emap_modes;
    if (emap_modes.empty()) {
        emap_modes.push_back(ImageMode("BW", QCPColorGradient()));
        QCPColorGradient *emap_grad = &emap_modes.back().grad;

        emap_grad->clearColorStops();
        emap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
        emap_grad->setLevelCount(2);

        emap_grad->setColorStopAt(0, QColor(0, 0, 0));
        emap_grad->setColorStopAt(1.0, QColor(255, 255, 255));
    }
    return emap_modes;
}


ImageModes& dirMapModes() {
    static ImageModes dir_modes;
    if (dir_modes.empty()) {
        dir_modes = createPresets();
        dir_modes.insert(dir_modes.begin(),ImageMode("Dir Color", QCPColorGradient()));
        QCPColorGradient *dir_grad = &dir_modes.front().grad;

        dir_grad->clearColorStops();
        dir_grad->setColorInterpolation(QCPColorGradient::ciRGB);
        //dir_grad->setPeriodic(true);
        dir_grad->setLevelCount(1024);

        dir_grad->setColorStopAt(0,       QColor(150, 0, 255));   // lila 6
        dir_grad->setColorStopAt(1.0 / 8, QColor(255, 0, 255));   // magenta 7
        dir_grad->setColorStopAt(2.0 / 8, QColor(255, 0, 0));     // red 0
        dir_grad->setColorStopAt(3.0 / 8, QColor(255, 150, 0));   // orange 1
        dir_grad->setColorStopAt(4.0 / 8, QColor(255, 255, 0));   // yellow 2
        dir_grad->setColorStopAt(5.0 / 8, QColor(0, 255, 0));     // green 3
        dir_grad->setColorStopAt(6.0 / 8, QColor(0, 255, 255));   // cyan 4
        dir_grad->setColorStopAt(7.0 / 8, QColor(0, 0, 255));     // blue 5
        dir_grad->setColorStopAt(1,       QColor(150, 0, 255));   // lila 6

    }
    return dir_modes;
}


ImageModes segmentMapModes(const cv::Mat &data, QCPRange &r) {
    ImageModes smap_modes;

    cv::RNG &rng = cv::theRNG();
    double rmin, rmax;
    cv::minMaxIdx(data, &rmin, &rmax);
    r.lower = rmin;
    r.upper = rmax;
    double diff = (rmax - rmin) + 1;
    int count = static_cast<int>(diff);

    smap_modes.push_back(ImageMode("Random Color", QCPColorGradient()));
    QCPColorGradient *smap_grad = &smap_modes.back().grad;

    smap_grad->clearColorStops();
    smap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
    smap_grad->setLevelCount(count);
    smap_grad->setColorStopAt(0, QColor(0, 0, 0));
    for (int i = 1; i != count; ++i) {
        smap_grad->setColorStopAt(i / diff, QColor(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225)));
    }

    smap_modes.push_back(ImageMode("BW", QCPColorGradient()));
    smap_grad = &smap_modes.back().grad;

    smap_grad->clearColorStops();
    smap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
    smap_grad->setLevelCount(count);

    smap_grad->setColorStopAt(0, QColor(0, 0, 0));
    smap_grad->setColorStopAt(1 / diff, QColor(255, 255, 255));
    smap_grad->setColorStopAt(1, QColor(255, 255, 255));
    return smap_modes;
}

ImageModes internalSegmentMapModes(const cv::Mat &data, QCPRange &r) {
    ImageModes smap_modes;

    cv::RNG &rng = cv::theRNG();
    double rmin, rmax;
    cv::minMaxIdx(data, &rmin, &rmax);
    rmin = -2;
    r.lower = rmin;
    r.upper = rmax;
    double diff = (rmax - rmin);
    int count = static_cast<int>(diff);

    smap_modes.push_back(ImageMode("Random Color", QCPColorGradient()));
    QCPColorGradient *smap_grad = &smap_modes.back().grad;

    smap_grad->clearColorStops();
    smap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
    smap_grad->setLevelCount(count);
    smap_grad->setColorStopAt(0, QColor(255, 255, 255));
    smap_grad->setColorStopAt(1 / diff, QColor(0, 0, 0));
    for (int i = 2; i != count; ++i) {
        smap_grad->setColorStopAt(i / diff, QColor(20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225), 20 + rng.uniform(0, 225)));
    }

    smap_modes.push_back(ImageMode("BW", QCPColorGradient()));
    smap_grad = &smap_modes.back().grad;

    smap_grad->clearColorStops();
    smap_grad->setColorInterpolation(QCPColorGradient::ciRGB);
    smap_grad->setLevelCount(count);

    smap_grad->setColorStopAt(0, QColor(255, 0, 0));
    smap_grad->setColorStopAt(1 / diff, QColor(0, 0, 0));
    smap_grad->setColorStopAt(2 / diff, QColor(255, 255, 255));
    smap_grad->setColorStopAt(1, QColor(255, 255, 255));
    return smap_modes;
}



ImageSources Detector::sources(const cv::Mat&) {
    ImageSources ret;
    if (!this->lsd)
        return ret;

    auto imgDes = this->lsd->imageDataDescriptor();
    auto imgData = this->lsd->imageData();
    size_t idx = 0;
    for_each(imgData.begin(), imgData.end(), [&](const cv::Mat& data){
        if (imgDes[idx].name == "canny_map") {
            if (data.type() == CV_8U)
                ret.push_back(ImageSource(imgDes[idx].name.c_str(), data, cannyMapMode(), QCPRange(0, 1)));
            else
                ret.push_back(ImageSource(imgDes[idx].name.c_str(), data, edgeMapModes(), QCPRange(-1, 7)));
        }
        else if (imgDes[idx].name == "edge_map")
            ret.push_back(ImageSource(imgDes[idx].name.c_str(), data, edgeMapModes(),QCPRange(-1,7)));
        else if (imgDes[idx].name == "segment_map") {
            QCPRange r;
            ret.push_back(ImageSource(imgDes[idx].name.c_str(), data, internalSegmentMapModes(data,r)));
            ret.back().range = r;
        }
        else if (imgDes[idx].name == "mag") {
            if (magSqr()) {
                ret.push_back(ImageSource("qmag", data, imageModePresets));
                cv::Mat tmp;
                
                if (data.type() != cv::DataType<float_type>::type)
                    data.convertTo(tmp, cv::DataType<float_type>::type);
                else 
                    data.copyTo(tmp);
                cv::sqrt(tmp, tmp);
                ret.push_back(ImageSource("mag", tmp, imageModePresets));
            }
            else {
                ret.push_back(ImageSource("mag", data, imageModePresets));
                cv::Mat tmp;

                if (data.type() != cv::DataType<float_type>::type)
                    data.convertTo(tmp, cv::DataType<float_type>::type);
                else
                    data.copyTo(tmp);
                ret.push_back(ImageSource("qmag", tmp.mul(tmp), imageModePresets));
            }

        }
        else
            ret.push_back(ImageSource(imgDes[idx].name.c_str(), data, imageModePresets));
        
        ++idx;
    });
    return ret;
}

