cmake_minimum_required(VERSION 3.5...3.27)

# Set CMake policies for Qt
if(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)  # Automatically link Qt executables to qtmain target on Windows
endif()
if(POLICY CMP0043)
    cmake_policy(SET CMP0043 NEW)  # Ignore COMPILE_DEFINITIONS_<Config> properties
endif()

# Include paths for headers spread across src/ and extensions/
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/accuracy
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/continuity_optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/detector_profile
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/image_analyzer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/ground_truth_inspector
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/3d_profile_plot
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/precision_optimizer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/profile_analyzer
    ${CMAKE_CURRENT_SOURCE_DIR}/extensions/quivers
)

# Line Analyzer Qt Application
le_add_executable(app_line_analyzer
    QT_APP
    SOURCES
        # Core
        src/main.cpp
        src/helpers.cpp
        src/analyzer.cpp
        src/preprocessing.cpp
        src/help_viewer.cpp
        # Extensions
        extensions/accuracy/accuracypanel.cpp
        extensions/continuity_optimizer/continuity_optimizer.cpp
        extensions/detector_profile/detector_profile_panel.cpp
        extensions/image_analyzer/image_analyzer_panel.cpp
        extensions/ground_truth_inspector/analyser_options.cpp
        extensions/ground_truth_inspector/ground_truth_inspector.cpp
        extensions/3d_profile_plot/3d_profile_plot.cpp
        extensions/precision_optimizer/precision_optimizer.cpp
        extensions/profile_analyzer/profile_analyzer.cpp
        extensions/quivers/quiverpanel.cpp
    HEADERS
        # Core
        src/helpers.h
        src/analyzer.h
        src/help_viewer.h
        src/help_button.hpp
        src/markdown_converter.hpp
        src/preprocessing.h
        src/latool.h
        # Extensions
        extensions/accuracy/accuracypanel.h
        extensions/continuity_optimizer/continuity_optimizer.h
        extensions/detector_profile/detector_profile_panel.h
        extensions/image_analyzer/image_analyzer_panel.h
        extensions/ground_truth_inspector/analyser_options.h
        extensions/ground_truth_inspector/ground_truth_inspector.h
        extensions/3d_profile_plot/3d_profile_plot.h
        extensions/precision_optimizer/precision_optimizer.h
        extensions/profile_analyzer/profile_analyzer.h
        extensions/quivers/quiverpanel.h
        # UI files
        ui/analyzer.ui
        ui/preprocessing.ui
        extensions/accuracy/accuracypanel.ui
        extensions/continuity_optimizer/continuity_optimizer.ui
        extensions/detector_profile/detector_profile_panel.ui
        extensions/image_analyzer/image_analyzer_panel.ui
        extensions/ground_truth_inspector/analyser_options.ui
        extensions/ground_truth_inspector/ground_truth_inspector.ui
        extensions/3d_profile_plot/3d_profile_plot.ui
        extensions/precision_optimizer/precision_optimizer.ui
        extensions/profile_analyzer/profile_analyzer.ui
        extensions/quivers/quiverpanel.ui
    QT_MODULES Core Gui Widgets PrintSupport OpenGL
    DEPS
        lib_algorithm
        lib_lsd
        lib_imgproc
        lib_qplot
        lib_qplot3d
        ${OPENGL_LIBRARIES}
        le::opencv
        dlib::dlib
)

# Copy README files to the binary output directory so the help viewer
# can find them at runtime when running the CMake-built executable.
file(GLOB _ext_readmes "${CMAKE_CURRENT_SOURCE_DIR}/extensions/*/README.md")
set(_all_readmes "${CMAKE_CURRENT_SOURCE_DIR}/README.md" ${_ext_readmes})

add_custom_command(TARGET app_line_analyzer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "$<TARGET_FILE_DIR:app_line_analyzer>/apps/line_analyzer"
    COMMENT "Copying help viewer README files"
)

# Copy the top-level README.
add_custom_command(TARGET app_line_analyzer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
        "$<TARGET_FILE_DIR:app_line_analyzer>/apps/line_analyzer/README.md"
)

# Copy each extension README, preserving the directory structure.
foreach(_readme ${_ext_readmes})
    file(RELATIVE_PATH _rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_readme}")
    get_filename_component(_rel_dir "${_rel}" DIRECTORY)
    add_custom_command(TARGET app_line_analyzer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:app_line_analyzer>/apps/line_analyzer/${_rel_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_readme}"
            "$<TARGET_FILE_DIR:app_line_analyzer>/apps/line_analyzer/${_rel}"
    )
endforeach()
