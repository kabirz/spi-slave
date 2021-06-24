cmake_minimum_required(VERSION 3.13)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.stm32-cmake/cmake/stm32_gcc.cmake)

project(spi_slave C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

stm32_fetch_cmsis(F7)
stm32_fetch_hal(F7)
find_package(CMSIS COMPONENTS STM32F7 REQUIRED)
find_package(HAL COMPONENTS STM32F7 LL_RCC LL_GPIO LL_SPI REQUIRED)

# STM32F4-Discovery
add_executable(${CMAKE_PROJECT_NAME} main.c)
target_link_libraries(${CMAKE_PROJECT_NAME}
    HAL::STM32::F7::LL_RCC
    HAL::STM32::F7::LL_GPIO
		HAL::STM32::F7::LL_SPI
    CMSIS::STM32::F767IG
    STM32::NoSys 
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USE_FULL_LL_DRIVER)
stm32_print_size_of_target(${CMAKE_PROJECT_NAME})

add_custom_target(flash-stlink
	VERBATIM COMMAND openocd
	-f interface/stlink-v2.cfg
	-f target/stm32f7x.cfg
	-c "program ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX_C} verify reset exit"
	DEPENDS ${CMAKE_PROJECT_NAME}
)

