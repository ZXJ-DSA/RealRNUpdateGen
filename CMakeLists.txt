cmake_minimum_required(VERSION 3.16)
project(ogr)

set(CMAKE_CXX_STANDARD 14)

if(APPLE)
    set(GDAL_INCLUDE_DIR "/usr/local/opt/gdal/include")
    set(GDAL_LIBRARY "/usr/local/opt/gdal/lib/libgdal.dylib")
endif()

find_package(GDAL REQUIRED)
if (GDAL_FOUND)
    message(STATUS "GDAL_INCLUDE_DIRS: ${GDAL_INCLUDE_DIRS}")
    message(STATUS "GDAL_LIBRARIES: ${GDAL_LIBRARY_DIRS}")
    message(STATUS "GDAL_VERSION: ${GDAL_VERSION}")
    include_directories(${GDAL_INCLUDE_DIRS})
    link_directories(${GDAL_LIBRARY_DIRS})
endif ()

# boost
find_package(Boost 1.71.0 COMPONENTS system filesystem REQUIRED)
if (Boost_FOUND)
    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARY_DIRS}")
    message(STATUS "Boost_VERSION: ${Boost_VERSION}")
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif ()


add_executable(ogrNew ogrNew.cpp)
add_executable(trajectory trajectory.cpp)
add_executable(process process.cpp)

target_link_libraries(ogrNew ${GDAL_LIBRARY})
target_link_libraries(trajectory ${GDAL_LIBRARY})

if (Boost_FOUND)
    if(UNIX AND NOT APPLE)
        target_link_libraries(trajectory pthread boost_system boost_thread)# For linux
        target_link_libraries(ogrNew pthread boost_system boost_thread)# For linux
        target_link_libraries(process pthread boost_system boost_thread)# For linux
    endif()
    if(APPLE)
        target_link_libraries(trajectory boost_system boost_thread-mt)# For mac
        target_link_libraries(ogrNew boost_system boost_thread-mt)# For mac
        target_link_libraries(process boost_system boost_thread-mt)# For mac
    endif()
endif ()
