# -Try to find Arpack
#
#
# The following are set after configuration is done:
#  ARPACK_FOUND
#  ARPACK_LIBRARIES

IF(NOT ARPACK_DIR)
IF (WIN32)
SET(ARPACK_DIR
    ${MAINFOLDER}/extern_libs/arpack/lib
)
ELSE(WIN32)
SET(ARPACK_DIR
  ${MAINFOLDER}/extern_libs/arpack/lib
  /usr/lib
  /usr/local/lib
  /usr/lib64
  /usr/local/lib64
  /afs/cg.cs.tu-bs.de/lib/linux/c++/arpack
)
ENDIF(WIN32)
ENDIF(NOT ARPACK_DIR)

IF (WIN32)
SET (ARPACK_NAMES libarpack parpack)
ELSE(WIN32)
SET (ARPACK_NAMES arpack arpack)
ENDIF(WIN32)

FIND_LIBRARY(ARPACK_LIBRARIES
  NAMES ${ARPACK_NAMES}
  PATHS ${ARPACK_DIR}
)

IF(ARPACK_LIBRARIES)
	SET(ARPACK_FOUND TRUE)
    SET(ARPACK_LIBS ${ARPACK_LIBRARIES})
    MESSAGE(STATUS "Arpack library found.")
ELSE(ARPACK_LIBRARIES)
	SET(ARPACK_FOUND FALSE)
    MESSAGE(WARNING "Arpack library not found.")
ENDIF(ARPACK_LIBRARIES)

MARK_AS_ADVANCED(
  ARPACK_LIBRARIES
  ARPACK_FOUND
)
