cmake_minimum_required(VERSION 3.16)
# Builds the library from this checkout; in your own project use components/ or idf_component.yml.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(getVescValues)
