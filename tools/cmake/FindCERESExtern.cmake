# ======================================================
# Include directories to add to the user project:
# ======================================================

# Provide CERES dir
SET(CERES_DIR "${MAINFOLDER}/extern_libs/ceres")

# When its possible to use the Config script use it.
if(EXISTS "${CERES_DIR}/CeresConfig.cmake")
# Include the standard CMake script
include("${CERES_DIR}/CeresConfig.cmake") 
endif()
