cmake_minimum_required(VERSION 3.10)
project(GPIO_Zedboard C)

set(CMAKE_C_STANDARD 99)

add_executable(GPIO_Zedboard GPIO_Control.c)