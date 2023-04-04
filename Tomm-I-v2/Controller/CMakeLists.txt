cmake_minimum_required (VERSION 3.18)

project(Controller)

set(include_dirs 
    external/rpidmx512/include
    external/rpidmx512/lib-debug/include
    external/lib-hal/include
    external/lib-pca9685/include
)
include_directories( ${include_dirs} )

file(GLOB uuid_src external/lib-uuid/src/*.c)
file(GLOB uuid_headers external/lib-uuid/src/*.h)
add_library(lib-uuid ${uuid_src} ${uuid_headers})
# target_include_directories(lib-uuid PUBLIC ${include_dirs})

file(GLOB hal_src external/lib-hal/src/*.cpp)
file(GLOB hal_headers external/lib-hal/include/*.h)
add_library(lib-hal ${hal_src} ${hal_headers})
# target_include_directories(lib-hal PUBLIC ${include_dirs})

file(GLOB pca9685_src external/lib-pca9685/src/*.cpp)
file(GLOB pca9685_headers external/lib-pca9685/include/*.h)
add_library(lib-pca9685 ${pca9685_src} ${pca9685_headers})
# target_include_directories(lib-pca9685 PUBLIC ${include_dirs})

#add_subdirectory(external)

