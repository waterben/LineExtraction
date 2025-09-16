#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "libs/geometry/include/geometry/base.hpp"
#include "libs/geometry/include/geometry/camera.hpp"

using FT = float;
using lsfm::Vec2;
using lsfm::Vec3;
using lsfm::CameraHom;

int main() {
    std::cout << "=== Projection Comparison Test ===" << std::endl;
    
    // Create a simple camera
    Vec3<FT> origin(0, 0, 0);
    Vec3<FT> orientation(0, 0, 0);
    FT fov = static_cast<FT>(50.0/180.0 * 3.14159265);
    lsfm::Vec2<FT> size(800, 400);
    
    CameraHom<FT> cam(fov, size, origin, orientation);
    
    // Test points
    std::vector<Vec3<FT>> test_points = {
        {1.0f, 2.0f, 5.0f},
        {-1.0f, 1.0f, 3.0f},
        {0.5f, -0.5f, 4.0f}
    };
    
    std::cout << "Input points:" << std::endl;
    for (size_t i = 0; i < test_points.size(); ++i) {
        std::cout << "  [" << i << "]: (" << test_points[i].x() << ", " << test_points[i].y() << ", " << test_points[i].z() << ")" << std::endl;
    }
    
    // Method 1: Element-wise projection
    std::vector<Vec2<FT>> result1(test_points.size());
    for (size_t i = 0; i < test_points.size(); ++i) {
        result1[i] = cam.project(test_points[i]);
    }
    
    // Method 2: Matrix-based projection with Eigen::Map
    std::vector<Vec2<FT>> result2(test_points.size());
    Eigen::Map<const Eigen::Matrix<FT,Eigen::Dynamic,3>> pts(&test_points[0][0], test_points.size(), 3);
    Eigen::Map<Eigen::Matrix<FT,Eigen::Dynamic,2>> res(&result2[0][0], result2.size(), 2);
    cam.projectRowMap(pts, res);
    
    // Method 3: Direct matrix operations (what projectRowMap does internally)
    std::vector<Vec2<FT>> result3(test_points.size());
    Eigen::Map<const Eigen::Matrix<FT,Eigen::Dynamic,3>> pts3(&test_points[0][0], test_points.size(), 3);
    Eigen::Map<Eigen::Matrix<FT,Eigen::Dynamic,2>> res3(&result3[0][0], result3.size(), 2);
    
    // Get the projection matrix from the camera
    // We need to access the internal projection matrix
    std::cout << "\n=== Manual Matrix Projection ===" << std::endl;
    
    // Convert points to homogeneous coordinates and project manually
    Eigen::Matrix<FT,Eigen::Dynamic,4> homogeneous_pts(test_points.size(), 4);
    for (size_t i = 0; i < test_points.size(); ++i) {
        homogeneous_pts.row(i) << test_points[i].x(), test_points[i].y(), test_points[i].z(), 1.0f;
    }
    
    std::cout << "Homogeneous points:" << std::endl << homogeneous_pts << std::endl;
    
    // Compare results
    std::cout << "\n=== Results Comparison ===" << std::endl;
    std::cout << "Method 1 (element-wise):" << std::endl;
    for (size_t i = 0; i < result1.size(); ++i) {
        std::cout << "  [" << i << "]: (" << result1[i].x() << ", " << result1[i].y() << ")" << std::endl;
    }
    
    std::cout << "\nMethod 2 (projectRowMap):" << std::endl;
    for (size_t i = 0; i < result2.size(); ++i) {
        std::cout << "  [" << i << "]: (" << result2[i].x() << ", " << result2[i].y() << ")" << std::endl;
    }
    
    std::cout << "\n=== Differences ===" << std::endl;
    for (size_t i = 0; i < result1.size(); ++i) {
        FT diff_x = std::abs(result1[i].x() - result2[i].x());
        FT diff_y = std::abs(result1[i].y() - result2[i].y());
        std::cout << "  [" << i << "]: diff_x=" << diff_x << ", diff_y=" << diff_y << std::endl;
        
        if (diff_x > 1e-6f || diff_y > 1e-6f) {
            std::cout << "    WARNING: Significant difference detected!" << std::endl;
        }
    }
    
    return 0;
}