cmake_minimum_required(VERSION 3.5)

project(i2c_app LANGUAGES C)

add_executable(i2c_app main.c h3_i2c.h h3.h h3_gpio.h h3_opi_one.h)

install (TARGETS i2c_app DESTINATION /tmp/i2c)

