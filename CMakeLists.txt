cmake_minimum_required(VERSION 3.13)
project(short-path)

set(CMAKE_CXX_STANDARD 14)

include_directories(explore/include)
include_directories(explore/include/explore)
include_directories(map_merge/include)
include_directories(map_merge/include/combine_grids)
include_directories(map_merge/include/map_merge)
include_directories(map_merge/src/combine_grids)
include_directories(map_merge/test)

add_executable(short-path
        explore/doc/architecture.dia
        explore/doc/screenshot.png
        explore/doc/wiki_doc.txt
        explore/include/explore/costmap_client.h
        explore/include/explore/costmap_tools.h
        explore/include/explore/explore.h
        explore/include/explore/frontier_search.h
        explore/launch/explore.launch
        explore/launch/explore_costmap.launch
        explore/src/costmap_client.cpp
        explore/src/explore.cpp
        explore/src/frontier_search.cpp
        explore/CHANGELOG.rst
        explore/CMakeLists.txt
        explore/package.xml
        LICENSE
        README.md)
