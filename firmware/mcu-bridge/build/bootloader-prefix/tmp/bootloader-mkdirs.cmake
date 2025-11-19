# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/nick/esp/v6.0/esp-idf/components/bootloader/subproject")
  file(MAKE_DIRECTORY "C:/Users/nick/esp/v6.0/esp-idf/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader"
  "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader-prefix"
  "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader-prefix/tmp"
  "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader-prefix/src"
  "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/nick/Desktop/AutoCombat/firmware/mcu-bridge/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
