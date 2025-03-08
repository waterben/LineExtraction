cmake_minimum_required(VERSION 3.0)

if (POLICY CMP0020)
cmake_policy(SET CMP0020 OLD)
cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0020)

# Add Libraries
add_executable(app_line_analyzer main.cpp helpers.cpp controlwindow.cpp preprocessing.cpp profileanalyzer.cpp precisionoptimizer.cpp pofuncplot.cpp quiver.cpp
#  continuityoptimizer.cpp analyseroptions.cpp lineanalyser2d.cpp connectionoptimizer.cpp 
helpers.h controlwindow.h  preprocessing.h  profileanalyzer.h precisionoptimizer.h pofuncplot.h quiver.h
#  continuityoptimizer.h analyseroptions.h lineanalyser2d.h connectionoptimizer.h 
controlwindow.ui preprocessing.ui profileanalyzer.ui precisionoptimizer.ui pofuncplot.ui quiver.ui
# continuityoptimizer.ui  analyseroptions.ui  lineanalyser2d.ui connectionoptimizer.ui 
)

qt5_use_modules(app_line_analyzer Core Gui Widgets PrintSupport OpenGL)
target_link_libraries(app_line_analyzer lib_lsd lib_phase_cong lib_qplot lib_qplot3d ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${DLIB_LIBRARY})


