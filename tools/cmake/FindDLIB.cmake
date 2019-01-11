# Find dlib headers and libraries.
#
#  DLIB_INCLUDE_DIR   - where to find dlib headers
#  DLIB_LIBRARIES     - List of dlib libraries
#  DLIB_FOUND         - True if dlib was found.


# Include dir
find_path(DLIB_INCLUDE_DIRS dlib
	PATHS "${DLIB_ROOT}" "$ENV{DLIB_ROOT}"
)

# Finally the library itself
find_library(DLIB_LIBRARY NAMES dlib PATHS ${DLIB_ROOT}/lib $ENV{DLIB_ROOT}/lib)
find_library(DLIB_LIBRARY_DEBUG NAMES dlib_d dlibd PATHS ${DLIB_ROOT}/lib $ENV{DLIB_ROOT}/lib)
set(DLIB_LIBRARIES optimized ${DLIB_LIBRARY} debug ${DLIB_LIBRARY_DEBUG})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set DLIB_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(DLIB DEFAULT_MSG DLIB_INCLUDE_DIRS DLIB_LIBRARY)


mark_as_advanced(DLIB_INCLUDE_DIRS DLIB_LIBRARIES)
