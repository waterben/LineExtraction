# LineExtraction CMake Utilities
# Modern CMake utility functions to reduce redundancy and improve modularity

include_guard(GLOBAL)

# Configure project-wide common settings
function(le_configure_common_settings)
    # Set common properties for all targets in this directory and subdirectories
    set(CMAKE_CXX_STANDARD 17 PARENT_SCOPE)
    set(CMAKE_CXX_STANDARD_REQUIRED ON PARENT_SCOPE)

    # Output directories
    set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/" PARENT_SCOPE)
    set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/" PARENT_SCOPE)

    # Build type specific flags
    if(BUILD_DEBUG)
        set(CMAKE_BUILD_TYPE DEBUG PARENT_SCOPE)
        if(NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -pg" PARENT_SCOPE)
        endif()
    else()
        set(CMAKE_BUILD_TYPE RELEASE PARENT_SCOPE)
        if(NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3 -ffast-math" PARENT_SCOPE)
        endif()
    endif()

    # Set warning flags separately so they can be tuned independently
    if(NOT MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Weffc++\
        -Wmissing-field-initializers -Wcast-align -Wcast-qual -Wold-style-cast -Woverloaded-virtual -Wdangling-else\
        -Wformat-security -Wshadow -Wsign-promo -Wundef -Wzero-as-null-pointer-constant -Wno-write-strings -Wreorder\
        -Wdelete-non-virtual-dtor -Wno-comment -Wnoexcept-type -Wnon-virtual-dtor -Werror" PARENT_SCOPE)
    endif()

    # Library type
    if(BUILD_STATIC)
        set(LIBRARY_TYPE STATIC PARENT_SCOPE)
        add_definitions(-D_STATIC)
    else()
        set(LIBRARY_TYPE SHARED PARENT_SCOPE)
        add_definitions(-D_SHARED)
    endif()
endfunction()

# Setup core dependencies that are used by many components
function(le_setup_core_dependencies)
    # Eigen
    include(extern_eigen)
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
        )
    endif()

    # dlib
    include(extern_dlib)
    if(NOT TARGET dlib::dlib)
        add_library(dlib::dlib INTERFACE IMPORTED)
        set_target_properties(dlib::dlib PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${DLIB_INCLUDE_DIR}"
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${DLIB_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${DLIB_LIBRARY}"
            INTERFACE_COMPILE_OPTIONS "$<$<CXX_COMPILER_ID:GNU>:-Wno-maybe-uninitialized>"
        )
    endif()

    # OpenCV
    include(extern_opencv)
    if(NOT TARGET le_opencv)
        add_library(le_opencv INTERFACE)
        target_include_directories(le_opencv SYSTEM INTERFACE ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(le_opencv INTERFACE ${OpenCV_LIBS})
    endif()
    if(NOT TARGET le::opencv)
        add_library(le::opencv ALIAS le_opencv)
    endif()

    # OpenGL/GLUT (optional)
    find_package(GLUT QUIET)
    find_package(OpenGL QUIET)

    # LAPACK (optional)
    find_package(LAPACKExtern QUIET)
    if(NOT LAPACK_FOUND)
        find_package(LAPACK QUIET)
    endif()

    # Other optional dependencies
    find_package(SUPERLU QUIET)
    find_package(ARPACK QUIET)
endfunction()

# Setup Qt dependencies if enabled
function(le_setup_qt_dependencies)
    if(NOT ENABLE_QT)
        return()
    endif()

    # Find Qt5 components with REQUIRED when ENABLE_QT is ON
    find_package(Qt5Core QUIET)
    if(Qt5Core_FOUND)
        message(STATUS "Qt5 found")
        # Don't set global AUTOMOC - this will be set per target as needed

        find_package(Qt5Gui QUIET)
        find_package(Qt5Widgets QUIET)
        find_package(Qt5PrintSupport QUIET)
        find_package(Qt5OpenGL QUIET)

        # Propagate the FOUND variables to parent scope
        set(Qt5Core_FOUND ${Qt5Core_FOUND} PARENT_SCOPE)
        set(Qt5Gui_FOUND ${Qt5Gui_FOUND} PARENT_SCOPE)
        set(Qt5Widgets_FOUND ${Qt5Widgets_FOUND} PARENT_SCOPE)
        set(Qt5PrintSupport_FOUND ${Qt5PrintSupport_FOUND} PARENT_SCOPE)
        set(Qt5OpenGL_FOUND ${Qt5OpenGL_FOUND} PARENT_SCOPE)

        # Check if Qt5OpenGL was found and warn if not
        if(NOT Qt5OpenGL_FOUND)
            message(WARNING "Qt5OpenGL not found. OpenGL-dependent Qt features may not work.")
        endif()

        if(NOT (Qt5Gui_FOUND AND Qt5Widgets_FOUND AND Qt5PrintSupport_FOUND))
            message(WARNING "Some Qt5 components not found. Qt-dependent features will be disabled.")
            set(ENABLE_QT OFF PARENT_SCOPE)
        endif()
    else()
        message(WARNING "Qt5Core not found. Qt-dependent features will be disabled.")
        set(ENABLE_QT OFF PARENT_SCOPE)
    endif()
endfunction()

# Setup GTest for unit testing
function(le_setup_testing)
    if(NOT ENABLE_UNIT_TEST)
        return()
    endif()

    find_package(Threads REQUIRED)
    include(extern_gtest)
    if(GTEST_FOUND)
        if(NOT TARGET le_gtest)
            add_library(le_gtest INTERFACE)
            target_include_directories(le_gtest SYSTEM INTERFACE ${GTEST_INCLUDE_DIR})
            target_link_libraries(le_gtest INTERFACE ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY})
        endif()
        if(NOT TARGET le::gtest)
            add_library(le::gtest ALIAS le_gtest)
        endif()
    endif()
    # Note: enable_testing() is now called at top level in main CMakeLists.txt
endfunction()

# Add a LineExtraction library with common configuration
function(le_add_library target_name)
    cmake_parse_arguments(LE
        "HEADER_ONLY;AUTO_TESTS"  # boolean options
        "TYPE"         # single value args
        "SOURCES;HEADERS;PUBLIC_DEPS;PRIVATE_DEPS;PUBLIC_INCLUDES;PRIVATE_INCLUDES;PUBLIC_SYSTEM_INCLUDES;PRIVATE_SYSTEM_INCLUDES;COMPILE_DEFS;TEST_DEPS;EXCLUDED_SOURCES"  # multi-value args
        ${ARGN}
    )

    # Determine library type
    if(LE_HEADER_ONLY)
        set(lib_type INTERFACE)
    elseif(LE_TYPE)
        set(lib_type ${LE_TYPE})
    else()
        set(lib_type ${LIBRARY_TYPE})
    endif()

    # Auto-detect sources and headers if not provided
    if(NOT LE_SOURCES AND NOT LE_HEADER_ONLY)
        file(GLOB_RECURSE LE_SOURCES "src/*.cpp" "src/*.c")

        # Remove excluded sources
        if(LE_EXCLUDED_SOURCES)
            foreach(excluded_file ${LE_EXCLUDED_SOURCES})
                get_filename_component(abs_excluded "${CMAKE_CURRENT_SOURCE_DIR}/src/${excluded_file}" ABSOLUTE)
                list(REMOVE_ITEM LE_SOURCES ${abs_excluded})
            endforeach()
        endif()
    endif()

    if(NOT LE_HEADERS)
        file(GLOB_RECURSE LE_HEADERS "include/*/*.hpp" "include/*/*.h")
    endif()

    # Create the library
    if(LE_HEADER_ONLY)
        add_library(${target_name} INTERFACE)
    else()
        add_library(${target_name} ${lib_type} ${LE_SOURCES} ${LE_HEADERS})
        set_target_properties(${target_name} PROPERTIES LINKER_LANGUAGE CXX)
    endif()

    # Set include directories
    if(LE_HEADER_ONLY)
        if(LE_PUBLIC_INCLUDES)
            target_include_directories(${target_name} INTERFACE ${LE_PUBLIC_INCLUDES})
        else()
            target_include_directories(${target_name} INTERFACE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
            )
        endif()

        if(LE_PUBLIC_SYSTEM_INCLUDES)
            target_include_directories(${target_name} SYSTEM INTERFACE ${LE_PUBLIC_SYSTEM_INCLUDES})
        endif()

        if(LE_PRIVATE_SYSTEM_INCLUDES)
            target_include_directories(${target_name} SYSTEM INTERFACE ${LE_PRIVATE_SYSTEM_INCLUDES})
        endif()
    else()
        if(LE_PUBLIC_INCLUDES)
            target_include_directories(${target_name} PUBLIC ${LE_PUBLIC_INCLUDES})
        else()
            target_include_directories(${target_name} PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>
            )
        endif()

        if(LE_PRIVATE_INCLUDES)
            target_include_directories(${target_name} PRIVATE ${LE_PRIVATE_INCLUDES})
        endif()

        if(LE_PUBLIC_SYSTEM_INCLUDES)
            target_include_directories(${target_name} SYSTEM PUBLIC ${LE_PUBLIC_SYSTEM_INCLUDES})
        endif()

        if(LE_PRIVATE_SYSTEM_INCLUDES)
            target_include_directories(${target_name} SYSTEM PRIVATE ${LE_PRIVATE_SYSTEM_INCLUDES})
        endif()
    endif()

    # Link dependencies
    if(LE_PUBLIC_DEPS)
        target_link_libraries(${target_name} PUBLIC ${LE_PUBLIC_DEPS})
    endif()

    if(LE_PRIVATE_DEPS AND NOT LE_HEADER_ONLY)
        target_link_libraries(${target_name} PRIVATE ${LE_PRIVATE_DEPS})
    endif()

    # Compile definitions
    if(LE_COMPILE_DEFS)
        if(LE_HEADER_ONLY)
            target_compile_definitions(${target_name} INTERFACE ${LE_COMPILE_DEFS})
        else()
            target_compile_definitions(${target_name} PUBLIC ${LE_COMPILE_DEFS})
        endif()
    endif()

    # Auto-generate tests if enabled and tests directory exists
    if((LE_AUTO_TESTS OR ENABLE_UNIT_TEST) AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
        le_add_auto_tests(${target_name}
            TEST_DEPS ${LE_TEST_DEPS}
            EXCLUDED_SOURCES ${LE_EXCLUDED_SOURCES}
        )
    endif()
endfunction()

# Function to automatically generate test targets from test files
function(le_add_auto_tests library_target)
    cmake_parse_arguments(LE_TEST
        ""             # boolean options
        ""             # single value args
        "TEST_DEPS;EXCLUDED_SOURCES"    # multi-value args
        ${ARGN}
    )

    # Find all test files
    file(GLOB test_files "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp")

    # Remove excluded test files if specified
    if(LE_TEST_EXCLUDED_SOURCES)
        foreach(excluded_source ${LE_TEST_EXCLUDED_SOURCES})
            list(FILTER test_files EXCLUDE REGEX ".*${excluded_source}.*")
        endforeach()
    endif()

    # Only proceed if we have test files and gtest is available
    if(test_files AND TARGET le::gtest AND ENABLE_UNIT_TEST)
        foreach(test_file ${test_files})
            # Extract test name from filename
            get_filename_component(test_name_full ${test_file} NAME_WE)

            # Create test executable
            add_executable(${test_name_full} ${test_file})

            # Link to the library being tested
            target_link_libraries(${test_name_full} PRIVATE ${library_target})

            # Link to gtest
            target_link_libraries(${test_name_full} PRIVATE le::gtest)

            # Link to additional test dependencies
            if(LE_TEST_TEST_DEPS)
                target_link_libraries(${test_name_full} PRIVATE ${LE_TEST_TEST_DEPS})
            endif()

            # Add to test suite
            add_test(NAME ${test_name_full} COMMAND ${test_name_full})

            # Set test working directory to binary directory
            set_tests_properties(${test_name_full} PROPERTIES
                WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
            )
        endforeach()
    elseif(test_files AND NOT TARGET le::gtest)
        message(STATUS "Tests found for ${library_target} but gtest not available. Skipping test generation.")
    endif()
endfunction()

# Add a LineExtraction executable with common configuration
function(le_add_executable target_name)
    cmake_parse_arguments(LE
        "QT_APP"       # boolean options
        ""             # single value args
        "SOURCES;HEADERS;DEPS;INCLUDES;COMPILE_DEFS;QT_MODULES"  # multi-value args
        ${ARGN}
    )

    # Auto-detect sources if not provided
    if(NOT LE_SOURCES)
        file(GLOB LE_SOURCES "*.cpp" "*.c")
        file(GLOB LE_HEADERS "*.hpp" "*.h")
        if(LE_QT_APP)
            file(GLOB LE_UI_FILES "*.ui")
            list(APPEND LE_SOURCES ${LE_UI_FILES})
        endif()
    endif()

    # Create executable
    add_executable(${target_name} ${LE_SOURCES} ${LE_HEADERS})

    # Include directories
    if(LE_INCLUDES)
        target_include_directories(${target_name} PRIVATE ${LE_INCLUDES})
    endif()

    # Link dependencies
    if(LE_DEPS)
        target_link_libraries(${target_name} ${LE_DEPS})
    endif()

    # Qt setup
    if(LE_QT_APP AND ENABLE_QT AND Qt5Widgets_FOUND)
        # Collect unique directories that contain .ui files so AUTOUIC can
        # locate them even when the including header lives elsewhere.
        set(_ui_search_paths "")
        foreach(_src ${LE_SOURCES} ${LE_HEADERS})
            if(_src MATCHES "\\.ui$")
                get_filename_component(_ui_dir "${_src}" DIRECTORY)
                if(_ui_dir)
                    # Resolve relative paths against the current source dir
                    if(NOT IS_ABSOLUTE "${_ui_dir}")
                        set(_ui_dir "${CMAKE_CURRENT_SOURCE_DIR}/${_ui_dir}")
                    endif()
                    list(APPEND _ui_search_paths "${_ui_dir}")
                endif()
            endif()
        endforeach()
        list(REMOVE_DUPLICATES _ui_search_paths)

        # Enable AUTOMOC/AUTOUIC only for Qt applications
        set_target_properties(${target_name} PROPERTIES
            AUTOMOC ON
            AUTOUIC ON
        )
        if(_ui_search_paths)
            set_target_properties(${target_name} PROPERTIES
                AUTOUIC_SEARCH_PATHS "${_ui_search_paths}"
            )
        endif()

        if(LE_QT_MODULES)
            foreach(module ${LE_QT_MODULES})
                # Check if the specific Qt5 module target exists before linking
                if(TARGET Qt5::${module})
                    target_link_libraries(${target_name} Qt5::${module})
                else()
                    message(WARNING "Qt5::${module} target not found for ${target_name}. Skipping...")
                endif()
            endforeach()
        else()
            # Default Qt modules for typical apps
            target_link_libraries(${target_name} Qt5::Core Qt5::Gui Qt5::Widgets)
        endif()
    endif()

    # Compile definitions
    if(LE_COMPILE_DEFS)
        target_compile_definitions(${target_name} PRIVATE ${LE_COMPILE_DEFS})
    endif()
endfunction()

# Add an example executable
function(le_add_example target_name)
    cmake_parse_arguments(LE
        ""             # boolean options
        ""             # single value args
        "SOURCES;DEPS" # multi-value args
        ${ARGN}
    )

    # Auto-detect sources if not provided
    if(NOT LE_SOURCES)
        get_filename_component(example_name ${target_name} NAME_WE)
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${example_name}.cpp")
            set(LE_SOURCES "${example_name}.cpp")
        else()
            file(GLOB LE_SOURCES "*.cpp")
        endif()
    endif()

    add_executable(${target_name} ${LE_SOURCES})

    if(LE_DEPS)
        target_link_libraries(${target_name} ${LE_DEPS})
    endif()
endfunction()
