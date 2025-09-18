cmake_minimum_required(VERSION 3.5...3.27)

if(POLICY CMP0020)
    cmake_policy(SET CMP0020 OLD)
    cmake_policy(SET CMP0043 OLD)
endif(POLICY CMP0020)

# Line Analyzer Qt Application
le_add_executable(app_line_analyzer
    QT_APP
    SOURCES 
        main.cpp 
        helpers.cpp 
        controlwindow.cpp 
        preprocessing.cpp 
        profileanalyzer.cpp 
        precisionoptimizer.cpp 
        pofuncplot.cpp 
        quiver.cpp
        # Commented out files - enable as needed:
        # continuityoptimizer.cpp 
        # analyseroptions.cpp 
        # lineanalyser2d.cpp 
        # connectionoptimizer.cpp
    HEADERS
        helpers.h 
        controlwindow.h  
        preprocessing.h  
        profileanalyzer.h 
        precisionoptimizer.h 
        pofuncplot.h 
        quiver.h
        # Commented out files - enable as needed:
        # continuityoptimizer.h 
        # analyseroptions.h 
        # lineanalyser2d.h 
        # connectionoptimizer.h
        # UI files
        controlwindow.ui 
        preprocessing.ui 
        profileanalyzer.ui 
        precisionoptimizer.ui 
        pofuncplot.ui 
        quiver.ui
        # Commented out files - enable as needed:
        # continuityoptimizer.ui  
        # analyseroptions.ui  
        # lineanalyser2d.ui 
        # connectionoptimizer.ui
    QT_MODULES Core Gui Widgets PrintSupport OpenGL
    DEPS 
        lib_lsd 
        lib_imgproc 
        lib_qplot 
        lib_qplot3d 
        ${OPENGL_LIBRARIES} 
        ${OpenCV_LIBS} 
        dlib::dlib
)


