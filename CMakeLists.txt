# CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project("MFRT" LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add the libraries
add_subdirectory(Controllers)
add_subdirectory(Systems)
# add the executable
add_executable(MFRT main.cpp)

# link the libraries to the executable
target_link_libraries(MFRT PRIVATE Controllers)
target_link_libraries(MFRT PRIVATE Systems)

target_include_directories(MFRT PUBLIC
                           "${PROJECT_BINARY_DIR}"
                            "${PROJECT_SOURCE_DIR}/Controllers"
                            "${PROJECT_SOURCE_DIR}/Systems"
                           )