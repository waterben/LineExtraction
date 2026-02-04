/// @file obj_test.cpp
/// @brief OBJ file loading and 3D object handling.
///
/// Demonstrates loading 3D objects from Wavefront OBJ files:
/// - Vertex and edge extraction
/// - Object3D data structure usage
/// - Eigen matrix interoperability
///
/// @usage ./obj_test

#include <geometry/object3d.hpp>

#include <Eigen/Dense>

#include <iostream>

using namespace std;
using namespace lsfm;

typedef double FT;

int main(int argc, char** argv) {
  string f = "/media/lange/Data/Blender/example.obj";
  Object3DList<FT> objs;
  loadObjects(f, objs);
  Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> test(3, 2);
  test << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  Eigen::Vector3d vec;
  cout << sizeof(vec) << " " << sizeof(vec) / 8 << endl;
  Eigen::Matrix<double, 3, 3> m;
  cout << sizeof(m) << " " << sizeof(m) / 8 << endl;


  for_each(objs.begin(), objs.end(), [](const Object3D<FT> obj) { std::cout << obj; });


  return 0;
}
