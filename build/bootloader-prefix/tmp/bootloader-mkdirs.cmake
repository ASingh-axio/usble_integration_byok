# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Avijit/esp/v5.2.1/esp-idf/components/bootloader/subproject"
  "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader"
  "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader-prefix"
  "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader-prefix/tmp"
  "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader-prefix/src"
  "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Axio/espIDF/usb_hid_tri_proj/hid/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
