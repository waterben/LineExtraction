#include <geometry/draw.hpp>
#include <lsd/lsd_el.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using namespace lsfm;


int main(int argc, char** argv) {
  cv::VideoCapture capture;

  if (argc > 1)
    capture = cv::VideoCapture(argv[1]);
  else
    capture = cv::VideoCapture(0);

  cv::Mat frame;

  capture.grab();
  capture.retrieve(frame, IMREAD_GRAYSCALE);

  cv::Size size = frame.size();
  cout << "Image size: " << size.width << " x " << size.height << '\n' << endl;

  typedef float FT;
  typedef LsdEL<FT> LSD;

  // similar settings
  LSD lsd(0.01, 0.02, 20, 2, 10, 40, EL_USE_NFA);

  double cumulative_old = 0;
  double start;
  double duration_ms;
  double cumulative_duration = 0;
  int count = 0;

  cout << "this frame \t average \t fps \t frame\t\t" << endl;

  int exit_key_press = 0;
  while (exit_key_press != 'q') {
    count++;

    cv::Mat src = frame.clone();
    if (src.channels() != 1) cv::cvtColor(src, src, cv::COLOR_RGB2GRAY);

    // Reduce noise with a kernel 3x3
    cv::blur(src, src, cv::Size(3, 3));

    start = double(cv::getTickCount());

    lsd.detect(src);

    // end time
    duration_ms = (double(cv::getTickCount()) - start) * 1000 / cv::getTickFrequency();

    cumulative_duration += duration_ms;
    cout << duration_ms << "ms \t" << double(cumulative_duration / count) << "ms \t~" << int(1 / duration_ms * 1000)
         << "fps \t" << count << "frame \r" << flush;

    cv::Mat out(cv::Mat::zeros(frame.size(), CV_8UC3));
    frame.copyTo(out);

    lines(out, lsd.lineSegments(), cv::Scalar(255, 0, 0));
    imshow("video", out);

    if (!capture.grab() || !capture.retrieve(frame, IMREAD_GRAYSCALE)) break;

    exit_key_press = cvWaitKey(1);
  }

  cout << '\n' << endl;
  capture.release();
  return 0;
}
