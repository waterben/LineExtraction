#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>

//#define USE_CERES_JET

#include <lsd/lsd_cc.hpp>
#include <lsd/lsd_el.hpp>
#include <geometry/draw.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <utility/camera_utilities.hpp>
#include <line_tracker/stereo_line_analyzer.hpp>
#include <slam/robotCameras.hpp>
#include <geometry/stereo.hpp>

#include <lfd/StereoLineFilter.hpp>
#include <lfd/StereoLineMatcher.hpp>
#include <lfd/MotionDescriptor.hpp>
#include <lfd/MotionLineFilter.hpp>
#include <lfd/MotionLineMatcher.hpp>
#include <line_tracker/linematcher.hpp>
#include <line_tracker/featurepoints.hpp>
#include <line_tracker/linetracker.hpp>


using namespace std;
using namespace lsfm;

class Histogram {
public:
    Histogram(bool cumulate = false, int hSize = 256) : size(hSize), cumulative(cumulate) {}

    Histogram(const cv::Mat& img,  bool cumulate = false, int hSize = 256) : size(hSize), cumulative(cumulate)
    {
        calc(img, values, cumulative);
    }

    Histogram(const Histogram& other) : size(other.size), cumulative(other.cumulative), values(other.values.clone()) {}

    Histogram& operator=(const Histogram& other) {
        size = other.size;
        cumulative = other.cumulative;
        values = other.values.clone();
    }

    inline void calc(const cv::Mat& img)
    {
        calc(img, values);
    }

    inline void calc(const cv::Mat& img, cv::Mat& hist) const
    {
        calc(img, hist, cumulative);
    }

    inline void calc(const cv::Mat& img, cv::Mat& hist, bool cumulate) const
    {
        int channels = 0;
        float range[] = {0, static_cast<float>(size)};
        const float* ranges[] = {range};
        hist.zeros(size, 1, CV_32F);
        cv::calcHist(&img, 1, &channels, cv::Mat(), hist, 1, &size, ranges, true, false);
        //make cumulative
        if(cumulate) {
            cumsum(hist);
        }
    }

    inline void draw(cv::Mat& img, int width = 512, int height = 384) const
    {
        if(size <= 0) return;
        int hWidth = (width / size) * size;     //make multiple of size
        int footerHeight = 10;
        double maxVal = 0;
        int bins = values.rows;
        int binWidth = hWidth / bins;
        cv::minMaxLoc(values, 0, &maxVal, 0, 0);
        double scale = height / maxVal;
        img = cv::Mat::zeros(height, hWidth, CV_8U);
        for( int h = 0; h < bins; h++ ) {
            float binVal = *values.ptr<float>(h);
            cv::rectangle( img, cv::Point(h*binWidth, height-footerHeight - static_cast<int>(binVal*scale)),
                        cv::Point( (h+1)*binWidth - 2, height-footerHeight-1),
                        cv::Scalar::all(255),
                        CV_FILLED );
            cv::rectangle( img, cv::Point(h*binWidth, height-footerHeight),
                        cv::Point( (h+1)*binWidth - 1, height),
                        cv::Scalar::all(h),
                        CV_FILLED );
        }
    }

    inline void cumsum(cv::Mat& hist) const
    {
        for(int i=1; i<hist.rows; i++) {
            *hist.ptr<float>(i) = *hist.ptr<float>(i) + *hist.ptr<float>(i-1);
        }
    }

    inline const cv::Mat& getValues() const {return values;}

    inline void calcTransFunc(const cv::Mat& hist, vector<int>& tFunc) const
    {
        float cumVal = 0.0f;
        tFunc.resize(values.rows);
        for(int i=0; i<values.rows; i++) {
            float cumHist = 0.0f;
            for(int j=0; j<hist.rows; j++) {
                cumVal += values.ptr()[i];
                cumHist += hist.ptr()[j];
                if(cumHist >= cumVal) {
                    tFunc[i] = j;
                    break;
                }
            }
        }
    }

    static inline void applyTransFunc(cv::Mat& img, const vector<int>& tFunc)
    {
        for(int r=0; r<img.rows; r++) {
            for(int c=0; c<img.cols; r++) {
                int idx = r*img.cols + c;
                *img.ptr(idx) = tFunc[*img.ptr(idx)];
            }
        }
    }

    static inline void histTransfer(const cv::Mat& srcImg, cv::Mat& dstImg)
    {
        //calc histograms
        cv::Mat h1, h2;
        int hSize = 256;
        int channels = 0;
        float range[] = {0, static_cast<float>(hSize)};
        const float* ranges[] = {range};
        h1.zeros(hSize, 1, CV_32F);
        h2.zeros(hSize, 1, CV_32F);
        cv::calcHist(&dstImg, 1, &channels, cv::Mat(), h1, 1, &hSize, ranges, true, false);
        cv::calcHist(&srcImg, 1, &channels, cv::Mat(), h2, 1, &hSize, ranges, true, false);
        //calc transfer function
        vector<int>tf;
        float cumH1 = 0.0f;
        tf.resize(h1.rows);

        for(int i=0; i<h1.rows; i++) {
            cumH1 += *h1.ptr<float>(i);
            float cumH2 = 0.0f;

            for(int j=0; j<h2.rows; j++) {
                cumH2 += *h2.ptr<float>(j);
                if(cumH2 >= cumH1) {
                    tf[i] = j;
                    break;
                }
            }
        }
        //apply transfer
        for(int r=0; r<dstImg.rows; r++) {

            for(int c=0; c<dstImg.cols; c++) {
                dstImg.ptr(r)[c] = tf[dstImg.ptr(r)[c]];
            }
        }
    }

private:
    int size;
    bool cumulative;
    cv::Mat values;
};

inline void drawIndexPoints(    cv::Mat& img,                               ///< Image to draw to
                                const IndexVector& pVec,                    ///< Vector containing the indexes of the points to draw.
                                int size = DEFAULT_PSIZE,                   ///< Size of the points to draw in pixels. Default is 3.
                                int thickness = CV_FILLED,                  ///< Thickness of the surrounding rectangle of the points. Default is filled (CV_FILLED).
                                cv::Scalar color = cv::Scalar(0, 255, 255))   ///< color of the points
{
    for(IndexVector::const_iterator it = pVec.begin(); it != pVec.end(); it++) {
        cv::Point tp = cv::Point(*it % img.cols, *it / img.cols);
        int radius = size / 2;
        cv::rectangle(img, cv::Point(tp.x - radius, tp.y - radius), cv::Point(tp.x + radius, tp.y + radius), color, thickness);
    }
}

inline void drawDirectionMap( cv::Mat& img, const cv::Mat& dMap)
{
//    uchar colors[8][3] = {{0,0,255},{0,207,255},{0,255,0},{64,255,0},{255,255,0},{255,64,0},{255,0,128},{191,0,255}};
    uchar colors[8][3] = {{0,0,128},{0,104,128},{0,128,0},{32,128,0},{128,128,0},{128,32,0},{128,0,64},{96,0,128}};
    img = cv::Mat::zeros(dMap.rows, dMap.cols, CV_8UC3);
    int cn = img.channels();
    for(int r=0; r<dMap.rows; r++) {
        uchar* rowPtr = img.ptr(r);
        for(int c=0; c<dMap.cols; c++) {
            size_t d = *dMap.ptr<uchar>(r,c);
            if(d < 8 && d >=0) {
                rowPtr[c*cn + 0] = colors[d][0]; // B
                rowPtr[c*cn + 1] = colors[d][1]; // G
                rowPtr[c*cn + 2] = colors[d][2]; // R
            }
        }
    }
}
inline void getLinePixels(int imgWidth, const LineSegment<double>& l, IndexVector& vec)
{
    double diffX, diffY, ratio;
    int quadrant;
    double tmpX = getX(l.startPoint());
    double tmpY = getY(l.startPoint());
    int x_0 = static_cast<int>(tmpX);
    int y_0 = static_cast<int>(tmpY);
    int x_1 = static_cast<int>(getX(l.endPoint()));
    int y_1 = static_cast<int>(getY(l.endPoint()));
    ratio = fabs(l.directionY() / l.directionX());
    vec.clear();
    // calc quadrant
    if(y_1 < y_0) { //upper half
        if(x_1 > x_0) { //right side
            quadrant = 0;
            diffX = ceil(tmpX) - tmpX;
            diffY = tmpY - floor(tmpY);
        }
        else { //left side
            quadrant = 1;
            diffX = tmpX - floor(tmpX);
            diffY = tmpY - floor(tmpY);
        }
    }
    else { //lower half
        if(x_1 > x_0) { //right side
            quadrant = 3;
            diffX = ceil(tmpX) - tmpX;
            diffY = ceil(tmpY) - tmpY;
        }
        else { //left side
            quadrant = 2;
            diffX = tmpX - floor(tmpX);
            diffY = ceil(tmpY) - tmpY;
        }
    }
    // add start point to vec
    vec.push_back(x_0 + imgWidth * y_0);

    while(!(x_0 == x_1 && y_0 == y_1)) {
        switch(quadrant) {
        case 0:
            if(diffY / diffX < ratio) {
                // next pixel upwards
                tmpY = tmpY - diffY;
                tmpX = l.x(tmpY);
                diffY = 1.0;
                diffX = ceil(tmpX) - tmpX;
                y_0--;
            }
            else {
                //next pixel right
                tmpX = tmpX + diffX;
                tmpY = l.y(tmpX);
                diffX = 1.0;
                diffY = tmpY - floor(tmpY);
                x_0++;
            }
            break;
        case 1:
            if(diffY / diffX < ratio) {
                // next pixel upwards
                tmpY = tmpY - diffY;
                tmpX = l.x(tmpY);
                diffY = 1.0;
                diffX = tmpX - floor(tmpX);
                y_0--;
            }
            else {
                //next pixel left
                tmpX = tmpX - diffX;
                tmpY = l.y(tmpX);
                diffX = 1.0;
                diffY = tmpY - floor(tmpY);
                x_0--;
            }
            break;
        case 2:
            if(diffY / diffX < ratio) {
                // next pixel downwards
                tmpY = tmpY + diffY;
                tmpX = l.x(tmpY);
                diffY = 1.0;
                diffX = tmpX - floor(tmpX);
                y_0++;
            }
            else {
                //next pixel left
                tmpX = tmpX - diffX;
                tmpY = l.y(tmpX);
                diffX = 1.0;
                diffY = ceil(tmpY) - tmpY;
                x_0--;
            }
            break;
        case 3:
            if(diffY / diffX < ratio) {
                // next pixel downwards
                tmpY = tmpY + diffY;
                tmpX = l.x(tmpY);
                diffY = 1.0;
                diffX = ceil(tmpX) - tmpX;
                y_0++;
            }
            else {
                //next pixel right
                tmpX = tmpX + diffX;
                tmpY = l.y(tmpX);
                diffX = 1.0;
                diffY = ceil(tmpY) - tmpY;
                x_0++;
            }
            break;
        }
        vec.push_back(x_0 + imgWidth * y_0);
    }
}


#define NUM_CAMS 2

int main(int argc, char** argv)
{
    // handle CMDline params ---------------------------------------------------------
    std::string filename1 = "../../Datasets/euroc/t1_3_stereo.avi";
    std::string filename2 = "../../Datasets/euroc/intrinsics.yml";
    std::string filename3 = "../../Datasets/euroc/extrinsics.yml";

    //TODO better argument handling
    if (argc > 2) {
        filename1 = argv[1];
        filename2 = argv[2];
        filename3 = argv[3];
    }

    cv::Mat frameLR, imLeft, imRight, prevImLeft, tmpImg1, tmpImg2;
    VideoCapture capture;
    capture = VideoCapture(filename1);
    capture.grab();
    capture.retrieve(frameLR, IMREAD_GRAYSCALE);
    cvtColor(frameLR,frameLR, CV_RGB2GRAY);

    double scaleFactor = 1.0;
    Size img_size;
    if(NUM_CAMS == 2)
        img_size.width = frameLR.size().width * scaleFactor / 2;
    else
        img_size.width = frameLR.size().width * scaleFactor;
    img_size.height = frameLR.size().height * scaleFactor;

    tmpImg1 = frameLR(cv::Range(0, frameLR.rows), cv::Range(0, frameLR.cols/2)); //no data copying here - Range excludes the last value of width, so correct.

    // reading intrinsic parameters
    cv::FileStorage fs(filename2, READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file intrinsic_filename\n");
        return -1;
    }

    cv::Mat M1, D1, M2, D2;
    fs["M1"] >> M1;     //camera matrix 1
    fs["D1"] >> D1;     //distortion parameters 1
    fs["M2"] >> M2;     //camera matrix 2
    fs["D2"] >> D2;     //distortion parameters 2

    M1 *= scaleFactor;
    M2 *= scaleFactor;
    fs.release();

    // initialize Line Detector -------------------------------------------------------------------------

     /// \typedef
    typedef float FT;
    typedef cv::Point_<FT> LPT;
    typedef cv::Point_<FT> PT;
//    typedef LsdCC<FT>::LineSegment MyDetectorLine;
//    typedef lsfm::Line3d MyLine3D;
    //th_low, th_high, min_px, dist, min_len, log_eps, flags
    //LsdCC<FT> lsd1(0.008, 0.012, 30, 0, 2);
    //LsdEL<FT> lsd1(0.004, 0.012, 30, 2, 5, 0, 0);
    //LsdEL<FT> lsd1(0.004, 0.03, 60, 4, 30, 10, EL_USE_NFA);
    LsdEL_tracker<FT> lsd1(0.004, 0.03, 60, 4, 30, 10, EL_USE_NFA);

    cv::undistort(tmpImg1, prevImLeft, M1, D1);

//    cv::equalizeHist(tmpImg2, prevImLeft);

    // line detection ------------------------------------------------------------------
    lsd1.detect(prevImLeft);

    //double mean, lastMean = cv::mean( prevImLeft)[0];

    //LsdCC<FT>::LineSegmentVector lines = lsd1.lines();
    //LsdCC<FT>::LineSegmentVector lastLines;
    LineSegment2Vector<FT> lines = lsd1.lines();
    LineSegment2Vector<FT> lastLines;

    // calculate previous feature points ------------------------------------------------
    FeaturePoints<FT> features(FeaturePoints<FT>::MIN_NUM_FEATURES, 40.0);
    features.calcFeaturePoints(lines);

    double error = features.meanError(), dtime = 0, time = 0;
    LineMatcher<FT> matcher(7.0, 5.0);

    int last_key_press = 0;
    int cycleNr = 0;
    while (last_key_press != 'q')
    {
        // get new frame ---------------------------------------
        if(!capture.grab())
            break;
        capture.retrieve(frameLR, IMREAD_GRAYSCALE);
        cvtColor(frameLR,frameLR,CV_RGB2GRAY);

        last_key_press = cv::waitKey(1000000);
        //last_key_press = cvWaitKey(1)%256;
        ++cycleNr;

        // undistort  -------------------------------------------
        tmpImg1 = frameLR(cv::Range(0, frameLR.rows), cv::Range(0, frameLR.cols/2)); //no data copying here - Range excludes the last value of width, so correct.
//        imRight = frameLR(cv::Range(0, frameLR.rows), cv::Range(frameLR.cols/2 + 1, frameLR.cols));

        cv::undistort(tmpImg1, imLeft, M1, D1);

        // histogram transfer
        cv::Mat imLeftTransf = imLeft.clone();
        Histogram::histTransfer(prevImLeft, imLeftTransf);

        //calc optical flow --------------------------------------
        int64 start = cv::getTickCount();
        features.predictPoints(prevImLeft, imLeftTransf);
        features.outlierDetection(FeaturePoints<FT>::OUTLIER_ALL);

        // line detection ---------------------------------------
        int64 dstart = cv::getTickCount();

        lsd1.processEdges(imLeftTransf);

//        lsd1.detectLineSegments(lsd1.grad(), lsd1.nms(), features);
//        LineSegment2Vector<FT> Testlines = lsd1.lines();

        vector<cv::Point2f> pts;
        features.getValidPredPoints(pts, FeaturePoints<FT>::OUTLIER_ALL);
        //IndexVector seeds = IndexVector(lsd1.nms().seeds());
        IndexVector seeds;

        for(vector<cv::Point2f>::const_iterator it = pts.begin(); it != pts.end(); it++) {
            // search for nearest edge point
            int idx;
            if(searchNeighbor<FT>(*it, lsd1.nms().directionMap(), 3, idx)) {
                seeds.push_back(idx);
            }
        }
//        //std::cout<<"size of seeds: "<<seeds.size()<<"\tpredicted points: "<<pts.size()<<"\tseedcounter: "<<seedcounter<<"\n";

        lsd1.detectLinesNoSplit(lsd1.grad(), lsd1.nms(), seeds);

//        lsd1.detect(imLeftTransf);
        lastLines = lines;
        lines = lsd1.lines();

        //TODO check if the lines lie on a seed point
//        IndexVector linePixels;
//        std::cout<<"seeds: "<<seeds.size()<<"\tlines: "<<lines.size()<<std::endl;
//        int droppedLines = 0;
//        for(int i=0; i<lines.size(); i++) {
//            linePixels.clear();
//            getLinePixels(imLeft.cols, lines[i], linePixels);
//            int j;
//            for(j=0; j<linePixels.size(); j++) {
//                bool quit = false;
//                for(int k=0; k<seeds.size(); k++) {
//                    if(seeds[k] == linePixels[j]) {
//                        quit = true;
//                        break;
//                    }
//                }
//                if(quit) break;
//            }
//            if(j == linePixels.size()) droppedLines++;
//        }
//        std::cout<<"dropped lines: "<<droppedLines<<"\n";

        //normalize image
        //mean = cv::mean( imLeft)[0];
        //cv::Mat linesNormLeft = imLeft * (lastMean / mean);
        //cv::Mat eqLinesLeft;


        //normHist(hist);
//        cv::Mat eHist;
//        histogram(imLeft, eHist);
        //normHist(eHist);

        dtime = (cv::getTickCount() - dstart) * 1000.0 / cv::getTickFrequency();

        // match lines ----------------------------------------------------------------

        matcher.match(features, lastLines, lines);
        time = (cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency();

        // drawing functions ---------------------------------------------
//        cv::Mat linesLeft = imLeft.clone();
        cv::Mat linesPrevLeft = prevImLeft.clone();
        cv::Mat flowLeft, dirMap;
//        //TODO remove test
        cv::Mat testimg = cv::Mat::zeros(imLeft.rows, imLeft.cols, CV_8UC3);
        for(int i=0; i<seeds.size(); i++) {
            testimg.ptr<uchar>(seeds[i] / testimg.cols)[(seeds[i] % testimg.cols) * 3] = 255;
            testimg.ptr<uchar>(seeds[i] / testimg.cols)[(seeds[i] % testimg.cols) * 3 + 1] = 255;
            testimg.ptr<uchar>(seeds[i] / testimg.cols)[(seeds[i] % testimg.cols) * 3 + 2] = 255;
        }
//        cv::Point2f p1 = cv::Point2f(56.04, 37.70);
//        cv::Point2f p2 = cv::Point2f(123.05, 234.20);
//        LineSegment<double> testLs = LineSegment<double>(p1, p2);
//        IndexVector testVec;

//        cv::Point tp = cv::Point(static_cast<int>(round(p1.x)), static_cast<int>(round(p1.y)));
//        cv::rectangle(testimg, cv::Point(tp.x - 1, tp.y - 1), cv::Point(tp.x + 1, tp.y + 1), cv::Scalar(0,255,0), CV_FILLED);
//        tp = cv::Point(static_cast<int>(round(p2.x)), static_cast<int>(round(p2.y)));
//        cv::rectangle(testimg, cv::Point(tp.x - 1, tp.y - 1), cv::Point(tp.x + 1, tp.y + 1), cv::Scalar(0,0,255), CV_FILLED);
//        getLinePixels(testimg, testLs, testVec);
//        for(int i=0; i<testVec.size(); i++) {
//            testimg.ptr<uchar>(testVec[i] / testimg.cols)[(testVec[i] % testimg.cols) * 3] = 255;
//            testimg.ptr<uchar>(testVec[i] / testimg.cols)[(testVec[i] % testimg.cols) * 3 + 1] = 255;
//            testimg.ptr<uchar>(testVec[i] / testimg.cols)[(testVec[i] % testimg.cols) * 3 + 2] = 255;
//        }
//        //test end

        cv::cvtColor(imLeftTransf, flowLeft, CV_GRAY2BGR);

        //draw direction map
        drawDirectionMap(dirMap, lsd1.nms().directionMap());
        //draw lines
        matcher.drawNext(dirMap, lines, Scalar(255,255,255));
        matcher.drawPrev(linesPrevLeft, lastLines, Scalar(255,0,0));
//        linesLeft = drawLines<FT>(linesLeft, lines);
//        for(LineSegment2Vector<FT>::const_iterator it = Testlines.begin(); it!=Testlines.end(); it++) {
//            linesLeft = it->draw(linesLeft, Scalar(0,192,192));
//        }
        //draw points
        features.drawFeaturePoints(linesPrevLeft);
        //features.drawValidPredPoints(linesLeft);
//        drawIndexPoints(linesLeft, seeds);
        //draw optical flow
        features.drawOpticalFlow(flowLeft);

        //draw histograms
//        drawHist(histImg, hist);
//        drawHist(eHistImg, eHist);
        

        // calculate feature points --------------------------------------------------------
        features.calcFeaturePoints(lines);
        error = features.meanError();
        prevImLeft = imLeft.clone();
        //lastMean = mean;
//        lastHist = hist;

        //print summary
        cout<<"cycleNr: "<<cycleNr<<"\tmatches: "<<matcher.numMatches()<<"\tmerror: " << error << "\tdtime: " << dtime << "\ttime: " << time << "\n";

//        linesRight = drawLines<FT>(linesRight, lsd2.lines());
//        linesPrevLeft = drawLines<FT>(linesPrevLeft, previousLeftLines);
//        imshow("leftLines", linesLeft);
        imshow("prevLeftLines", linesPrevLeft);
        imshow("optical Flow", flowLeft);
        imshow("direction map", dirMap);

        imshow("seeds", testimg);

        //imshow("equalized histogram", eHistImg);

//        linesPrevLeft = linesLeft.clone();
//        previousLeftLines = lsd1.lines();
//        previousRightLines = lsd2.lines();

    }

    return 0;
}
