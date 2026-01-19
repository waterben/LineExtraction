cmake_minimum_required(VERSION 3.5...3.27)

# Add Libraries

add_executable(test_other_houghlines houghlines.cpp)
target_link_libraries(test_other_houghlines lib_geometry le::opencv)

add_executable(test_other_dlib test_dlib.cpp)
target_link_libraries(test_other_dlib lib_geometry dlib::dlib)

add_executable(test_opencv_features opencv_features.cpp)
target_link_libraries(test_opencv_features lib_geometry le::opencv)

# arpack++ test using SuperLU for sparse eigenvalue problems
if(TARGET arpack++ AND TARGET superlu::superlu)
    add_executable(test_arpack arpack_test.cpp)
    target_link_libraries(test_arpack arpack++ superlu::superlu)
    # Suppress specific warnings from arpack++ headers (legacy C++98 code)
    # These warnings cannot be fixed without modifying upstream arpack++
    target_compile_options(test_arpack PRIVATE
        -Wno-shadow
        -Wno-overloaded-virtual
        -Wno-implicit-fallthrough
    )
endif()
