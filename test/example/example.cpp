
#include <gtest/gtest.h>
#include <Eigen/Eigen>

using namespace Eigen;

template <DenseIndex rows, DenseIndex cols>
void resizeLikeTest() {
  MatrixXf A(rows, cols);
  MatrixXf B;
  Matrix<double, rows, cols> C;
  B.resizeLike(A);
  C.resizeLike(B);  // Shouldn't crash.
  EXPECT_EQ(B.rows(), rows);
  EXPECT_EQ(B.cols(), cols);

  VectorXf x(rows);
  RowVectorXf y;
  y.resizeLike(x);
  EXPECT_EQ(y.rows(), 1);
  EXPECT_EQ(y.cols(), rows);

  y.resize(cols);
  x.resizeLike(y);
  EXPECT_EQ(x.rows(), cols);
  EXPECT_EQ(x.cols(), 1);
}

TEST(resizeLikeTest12, EXAMPLE) { resizeLikeTest<1, 2>(); }

TEST(resizeLikeTest1020, EXAMPLE) { resizeLikeTest<10, 20>(); }

TEST(resizeLikeTest31, EXAMPLE) { resizeLikeTest<3, 1>(); }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
