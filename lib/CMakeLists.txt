# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/mspsa430)
link_directories(${Boost_LIBRARY_DIRS})

list(APPEND mspsa430_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/mspsa430/mspsa430.cpp
    source_impl.cc
)

list(APPEND mspsa430_lld_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/mspsa430/mspsa430_lld.c
)

add_library(gnuradio-mspsa430-lld SHARED ${mspsa430_lld_sources})
add_library(gnuradio-mspsa430 SHARED ${mspsa430_sources})
target_link_libraries(gnuradio-mspsa430 ${Boost_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} ${gnuradio-mspsa430-lld})
set_target_properties(gnuradio-mspsa430 PROPERTIES DEFINE_SYMBOL "gnuradio_mspsa430_EXPORTS")

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-mspsa430
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

