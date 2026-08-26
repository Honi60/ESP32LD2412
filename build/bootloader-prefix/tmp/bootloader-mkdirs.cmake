# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/honi/esp/esp-idf/components/bootloader/subproject"
  "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader"
  "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader-prefix"
  "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader-prefix/tmp"
  "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader-prefix/src/bootloader-stamp"
  "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader-prefix/src"
  "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/honi/Documents/pyython-proj/ESP32LD2412/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
