cmake_minimum_required(VERSION 3.5...3.27)

# Install complete directory
#
#   Copy the given folder to installation directory when "make install" is called.
#
#   DIRECTORY_TARGET    Relative path within cmake project directory.
#   INSTALL_PATH        Relative path within installation root.
#   INSTALL_COMPONENT   Component name used by installer.
macro ( install_dir DIRECTORY_TARGET INSTALL_PATH INSTALL_COMPONENT )
    install(DIRECTORY ${DIRECTORY_TARGET}
        DESTINATION "${INSTALL_DIR_SHARE}/${INSTALL_PATH}"
        COMPONENT ${INSTALL_COMPONENT} )
endmacro()

# Combined call of get_headers and get sources
#
#   TARGET_DIR      Search for files underneath this directory.
#   OUT_FILES       List of found header and source files.
#   FLAG: RECURSIVE  Search directory recursively
function( get_headers_and_sources TARGET_DIR OUT_FILES)
    get_sources("${TARGET_DIR}" OUT_SOURCES "${ARGN}")
    get_headers("${TARGET_DIR}" OUT_HEADERS "${ARGN}")
    set( ${OUT_FILES} ${OUT_SOURCES} ${OUT_HEADERS} PARENT_SCOPE )
endfunction()



# Wrapper function for combined call of get_sources and get_headers.
#
#   For convenience this function can be used to collect headers and sources at the same time.
#
#   TARGET_DIR      Search for files underneath this directory.
#   OUT_FILES       List of found source and header files.
#   FLAG: RECURSIVE  Search directory recursively
function( get_source_and_headers TARGET_DIR OUT_FILES )
    get_headers( ${TARGET_DIR} OUT_HEADERS "${ARGN}")
    get_sources( ${TARGET_DIR} OUT_SOURCES "${ARGN}")
    set( ${OUT_FILES} ${OUT_HEADERS} ${OUT_SOURCES} PARENT_SCOPE )
endfunction()



# Wrapper function for get_files_in_directory.
#
#   This function calls get_files_in_directory with an already defined set of file extensions in order to filter out
#   everything that is not a source file. The result of the function call is just passed down to the output variable.
#
#   TARGET_DIR      Search for files underneath this directory.
#   OUT_SOURCES     List of found source files.
#   FLAG: RECURSIVE  Search directory recursively
function( get_sources TARGET_DIR OUT_SOURCES)
    set( FILE_EXT "cpp" "cc" "c" )
    get_files_in_directory( ${TARGET_DIR} "${FILE_EXT}" OUT_FILES "${ARGN}")
    set( ${OUT_SOURCES} ${OUT_FILES} PARENT_SCOPE )
endfunction()



# Wrapper function for get_files_in_directory.
#
#   This function calls get_files_in_directory with an already defined set of file extensions in order to filter out
#   everything that is not a header file. The result of the function call is just passed down to the output variable.
#
#   TARGET_DIR      Search for files underneath this directory.
#   OUT_HEADERS     List of found header files.
#   FLAG: RECURSIVE  Search directory recursively
function( get_headers TARGET_DIR OUT_HEADERS)
    set( FILE_EXT "h" "hpp" )
    get_files_in_directory( ${TARGET_DIR} "${FILE_EXT}" OUT_FILES "${ARGN}")
    set( ${OUT_HEADERS} ${OUT_FILES} PARENT_SCOPE )
endfunction()



# Search the target directory for files that match any of the stated extensions
#
#   Search the given target directory for files (if flag is set, search recursively). For every file found check if
#   any of the extensions matches the files extension. If so, store the absolute path of that file in a designated list
#   that is "returned" by this function.
#
#   TARGET_DIR      Search for files underneath this directory.
#   FILE_EXT        List of file extensions that are accepted.
#   OUT_FILES       List of found files that match any of the extensions.
#   FLAG: RECURSIVE  Search directory recursively
function( get_files_in_directory TARGET_DIR FILE_EXT OUT_FILES)
    set( TMP_FILE_LIST "" )
    foreach( EXT ${FILE_EXT} )
        if( "RECURSIVE" IN_LIST "${ARGN}")
            file( GLOB_RECURSE FILES "${TARGET_DIR}/*.${EXT}" )
        else()
            file( GLOB FILES "${TARGET_DIR}/*.${EXT}" )
        endif()

        foreach( FILE ${FILES} )
            list( APPEND TMP_FILE_LIST ${FILE} )
        endforeach()
    endforeach()
    set( ${OUT_FILES} ${TMP_FILE_LIST} PARENT_SCOPE )
endfunction()



# Get all items stored underneath the given target directory
#
#   Search the given target directory for items that lie directly underneath it and store their absolute path in a
#   designated list.
#
#   TARGET_DIR      This is the target directory to be searched.
#   OUT_DIR_ITEMS   List that contains the absolute path of target directories items
function( get_dir_items TARGET_DIR OUT_DIR_ITEMS )
    file( GLOB ITEMS "${TARGET_DIR}/*" )
    set( ${OUT_DIR_ITEMS} ${ITEMS} PARENT_SCOPE )
endfunction()



# Get all subdirectories underneath the given target directory
#
#   Search the given target directory for items that lie directly underneath it. If an item is a directory, store the
#   absolute path of the subdirectory in a designated list.
#
#   TARGET_DIR  This is the target directory to be searched for subdirectories.
#   OUT_DIRS    Absolute paths of found directories are stored in this variable as return value.
function( get_dirs TARGET_DIR OUT_DIRS )
    set( TMP_DIR_LIST "" )
    get_dir_items( ${TARGET_DIR} ITEMS )

    foreach( ITEM ${ITEMS} )
        if ( IS_DIRECTORY ${ITEM} )
            list( APPEND TMP_DIR_LIST ${ITEM} )
        endif()
    endforeach()

    set( ${OUT_DIRS} ${TMP_DIR_LIST} PARENT_SCOPE )
endfunction()


# Copy files from list to target dir
#
#   FILE_LIST   List of files to copy
#   TARGET_DIR  This is the copy target directory
function( copy_files FILE_LIST TARGET_DIR)
    file(MAKE_DIRECTORY ${TARGET_DIR})
    foreach(file ${FILE_LIST})
        file(COPY ${file} DESTINATION ${TARGET_DIR} )
    endforeach()
endfunction()

# Get all files by expression and copy to target directory
#
#   EXPRESSION  Expression to search for
#   TARGET_DIR  This is the copy target directory
#   FLAG: RECURSIVE  Search for files recursively
function( copy_files_glob EXPRESSION TARGET_DIR )
    if( "RECURSIVE" IN_LIST "${ARGN}")
        file( GLOB_RECURSE FILES ${EXPRESSION} )
    else()
        file( GLOB FILES ${EXPRESSION} )
    endif()
    copy_files("${FILES}" ${TARGET_DIR})
endfunction()
