# -Try to find Lapack
#
#
# The following are set after configuration is done:
#  BLAS_FOUND
#  BLAS_LIBRARIES

IF(NOT BLAS_DIR)
IF (WIN32)
SET(BLAS_DIR
    ${MAINFOLDER}/extern_libs/blas/lib
)
ELSE(WIN32)
SET(BLAS_DIR
  /usr/lib
  /usr/local/lib
  /usr/lib64
  /usr/local/lib64
  ${MAINFOLDER}/extern_libs/blas/lib
)
ENDIF(WIN32)
ENDIF(NOT BLAS_DIR)

IF (WIN32)
SET (BLAS_NAME libblas)
ELSE(WIN32)
SET (BLAS_NAME blas)
ENDIF(WIN32)

FIND_LIBRARY(BLAS_LIBRARIES
  NAMES ${BLAS_NAME}
  PATHS ${BLAS_DIR}
)

IF(BLAS_LIBRARIES)
	SET(BLAS_FOUND TRUE)
    SET(BLAS_LIBS ${BLAS_LIBRARIES})
    MESSAGE(STATUS "Blas library found.")
ELSE(BLAS_LIBRARIES)
	SET(BLAS_FOUND FALSE)
    #MESSAGE(WARNING "Blas library not found.")
ENDIF(BLAS_LIBRARIES)

MARK_AS_ADVANCED(
  BLAS_LIBRARIES
  BLAS_FOUND
)
