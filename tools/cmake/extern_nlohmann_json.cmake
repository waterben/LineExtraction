# extern_nlohmann_json.cmake - Fetch nlohmann/json (header-only)
#
# Provides the nlohmann_json::nlohmann_json target for CMake builds.
# Uses find_package first; falls back to FetchContent if not found.

include(FetchContent)

# Try system installation first.
find_package(nlohmann_json 3.11.0 QUIET)

if(nlohmann_json_FOUND)
    message(STATUS "Found system nlohmann_json: ${nlohmann_json_VERSION}")
else()
    message(STATUS "nlohmann_json not found on system — fetching v3.12.0 via FetchContent")
    FetchContent_Declare(
        nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.12.0
        GIT_SHALLOW    TRUE
    )
    # Disable tests and other extras.
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    set(JSON_Install OFF CACHE INTERNAL "")
    FetchContent_MakeAvailable(nlohmann_json)
endif()
