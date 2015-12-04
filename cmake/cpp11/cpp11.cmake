# -*- cmake -*-
#  Copyright 2010-2012 Matus Chochlik. Distributed under the Boost
#  Software License, Version 1.0. (See accompanying file
#  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
#

# we need C++11
if(${CMAKE_COMPILER_IS_GNUCXX})
  set(CPP11_flags -std=c++0x)
  set(CPP11_link_flags "-std=c++0x")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CPP11_flags -std=c++0x -stdlib=libc++ -DCPP11_NO_NOEXCEPT=1)
  set(CPP11_link_flags "-std=c++0x -stdlib=libc++")
endif()

set(CPP11_COMPILER_SWITCH ${CPP11_flags} CACHE LIST "C++ compiler flags")
set(CPP11_LINK_FLAGS ${CPP11_link_flags} CACHE LIST "C++ link flags")
mark_as_advanced(CPP11_COMPILER_SWITCH)
mark_as_advanced(CPP11_LINK_FLAGS)



# TODO add support for other compilers
 
#add_definitions(${CPP11_COMPILER_SWITCH})

function(cpp11_feature_detection FEATURE_NAME)
  configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cpp11/has_${FEATURE_NAME}.cpp
    ${CMAKE_BINARY_DIR}/cpp11/has_${FEATURE_NAME}.cpp
    )
  
  try_run(
    CPP11_RAN_${FEATURE_NAME}
    CPP11_COMPILED_${FEATURE_NAME}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/cpp11/has_${FEATURE_NAME}.cpp
    CMAKE_FLAGS "-DCMAKE_EXE_LINKER_FLAGS=${CPP11_link_flags}"
    COMPILE_DEFINITIONS ${CPP11_COMPILER_SWITCH}
    COMPILE_OUTPUT_VARIABLE COMP_OUT
    RUN_OUTPUT_VARIABLE RUN_OUT
    )
  set(CPP11_HAS_${FEATURE_NAME} FALSE)

  if(CPP11_COMPILED_${FEATURE_NAME})

    if(CPP11_RAN_${FEATURE_NAME} EQUAL 0)
      set(CPP11_HAS_${FEATURE_NAME} TRUE)
    else(CPP11_RAN_${FEATURE_NAME} EQUAL 0)
      file(WRITE has_${FEATURE_NAME}.exec ${RUN_OUT})
      message(WARNING "Failed to run test for c++11 ${FEATURE_NAME}: ${CPP11_RAN_${FEATURE_NAME}}"
	"Failed execution output stored in has_${FEATURE_NAME}.exec")
    endif(CPP11_RAN_${FEATURE_NAME} EQUAL 0)

  else(CPP11_COMPILED_${FEATURE_NAME})
    file(WRITE has_${FEATURE_NAME}.comp ${COMP_OUT})
    message(WARNING "Failed to compile test for c++11 ${FEATURE_NAME}"
      "compilation output is in ${has_${FEATURE_NAME}}.comp")
  endif(CPP11_COMPILED_${FEATURE_NAME})

  message(STATUS "Detecting support for c++11 feature '${FEATURE_NAME}': ${CPP11_HAS_${FEATURE_NAME}}")
  if(CPP11_HAS_${FEATURE_NAME})
    set(CPP11_HAS_${FEATURE_NAME} TRUE PARENT_SCOPE)
    set(CPP11_ENABLED TRUE PARENT_SCOPE)
  else()
    set(CPP11_HAS_${FEATURE_NAME} FALSE PARENT_SCOPE)
    file(WRITE ${PROJECT_BINARY_DIR}/cpp11/has_${FEATURE_NAME}.out ${OUT})
  endif()
  unset(CPP11_HAS_${FEATURE_NAME})
endfunction()

function(cpp11_lib_support NAME CODE DEFINITIONS LIBS)
  set(FLAGS ${CPP11_COMPILER_SWITCH} ${DEFINITIONS})
  set(test_file ${CMAKE_BINARY_DIR}/cpp11/supported_by_${NAME})
  file(WRITE  ${test_file}.cc ${CODE})
  try_compile(success
    ${CMAKE_BINARY_DIR}
    ${test_file}.cc
    COMPILE_DEFINITIONS ${FLAGS}
    CMAKE_FLAGS 
      -DCMAKE_EXE_LINKER_FLAGS:STRING=${CPP11_link_flags} 
      -DLINK_LIBRARIES:STRING=${LIBS}
    OUTPUT_VARIABLE OUT)
  message(STATUS "Checking c++11 compatibility for '${NAME}': ${success}")
  set(CPP11_SUPPORT_${NAME} ${success} PARENT_SCOPE)
  if(NOT success)
    file(WRITE ${test_file}.out ${OUT})
    message(STATUS "Failed compilation output stored in ${test_file}.out")
  endif(NOT success)
endfunction()

#cpp11_feature_detection(SCOPED_ENUMS)
#cpp11_feature_detection(VARIADIC_MACROS)
#cpp11_feature_detection(VARIADIC_TEMPLATES)
#cpp11_feature_detection(UNIFIED_INITIALIZATION_SYNTAX)
#cpp11_feature_detection(INITIALIZER_LISTS)
#cpp11_feature_detection(DEFAULTED_FUNCTIONS)
#cpp11_feature_detection(DELETED_FUNCTIONS)
#cpp11_feature_detection(EXPLICIT_CONVERSION_OPERATORS)
#cpp11_feature_detection(FUNCTION_TEMPLATE_DEFAULT_ARGS)
#cpp11_feature_detection(CONSTEXPR)
#cpp11_feature_detection(NOEXCEPT)
#cpp11_feature_detection(LAMBDAS)
#cpp11_feature_detection(NULLPTR)

#cpp11_feature_detection(CHRONO)
#cpp11_feature_detection(UNIQUE_PTR)

unset(CPP11_COMPILER_SWITCH)
