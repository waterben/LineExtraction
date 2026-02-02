#include <imgproc/image_operator.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <utility/test_images.hpp>

#include <ctime>
#include <fstream>
#include <iostream>
#include <string>


using namespace std;
using namespace lsfm;
using namespace cv;

int main(int argc, char** argv) {
  lsfm::TestImages::init(argv[0]);
  string filename = argc >= 2 ? argv[1] : lsfm::TestImages::noise("circle.png");

  cv::Mat src = cv::imread(filename);
  if (src.empty()) {
    cout << "Can not open " << filename << endl;
    return -1;
  }

  size_t p = filename.find_last_of('/');
  if (p == string::npos)
    p = 0;
  else
    ++p;
  string path = filename.substr(0, p);
  string name = filename.substr(p);
  name = name.substr(0, name.find_last_of('.'));

  cv::Mat noise = src.clone();
  GaussianNoiseOperator n10(10);
  n10.apply(noise);
  imwrite(path + name + "_noise10.png", noise);

  noise = src.clone();
  GaussianNoiseOperator n20(20);
  n20.apply(noise);
  imwrite(path + name + "_noise20.png", noise);

  noise = src.clone();
  GaussianNoiseOperator n30(30);
  n30.apply(noise);
  imwrite(path + name + "_noise30.png", noise);

  noise = src.clone();
  GaussianNoiseOperator n40(40);
  n40.apply(noise);
  imwrite(path + name + "_noise40.png", noise);

  noise = src.clone();
  GaussianNoiseOperator n50(50);
  n50.apply(noise);
  imwrite(path + name + "_noise50.png", noise);

  return 0;
}
