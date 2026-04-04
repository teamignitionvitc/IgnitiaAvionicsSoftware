# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "E:/IGNITION/Ignitia_Avionics/pico-sdk/tools/elf2uf2")
  file(MAKE_DIRECTORY "E:/IGNITION/Ignitia_Avionics/pico-sdk/tools/elf2uf2")
endif()
file(MAKE_DIRECTORY
  "E:/IGNITION/Ignitia_Avionics/build/elf2uf2"
  "E:/IGNITION/Ignitia_Avionics/build/elf2uf2"
  "E:/IGNITION/Ignitia_Avionics/build/elf2uf2/tmp"
  "E:/IGNITION/Ignitia_Avionics/build/elf2uf2/src/ELF2UF2Build-stamp"
  "E:/IGNITION/Ignitia_Avionics/build/elf2uf2/src"
  "E:/IGNITION/Ignitia_Avionics/build/elf2uf2/src/ELF2UF2Build-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/IGNITION/Ignitia_Avionics/build/elf2uf2/src/ELF2UF2Build-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/IGNITION/Ignitia_Avionics/build/elf2uf2/src/ELF2UF2Build-stamp${cfgdir}") # cfgdir has leading slash
endif()
