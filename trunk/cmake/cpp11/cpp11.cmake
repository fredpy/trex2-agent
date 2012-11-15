# -*- cmake -*-
#  Copyright 2010-2012 Matus Chochlik. Distributed under the Boost
#  Software License, Version 1.0. (See accompanying file
#  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
#

# we need C++11
if(${CMAKE_COMPILER_IS_GNUCXX})
  set(CPP11_COMPILER_SWITCH -std=c++0x)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CPP11_COMPILER_SWITCH -std=c++0x -stdlib=libc++ -DCPP11_NO_NOEXCEPT=1)
endif()
# TODO add support for other compilers
 
add_definitions(${CPP11_COMPILER_SWITCH})

function(cpp11_feature_detection FEATURE_NAME)
  configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cpp11/has_${FEATURE_NAME}.cpp
    ${CMAKE_BINARY_DIR}/cpp11/has_${FEATURE_NAME}.cpp
    )
  
  try_compile(
    CPP11_HAS_${FEATURE_NAME}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/cpp11/has_${FEATURE_NAME}.cpp
    COMPILE_DEFINITIONS ${CPP11_COMPILER_SWITCH}
    OUTPUT_VARIABLE OUT
    )
  message(STATUS "Detecting support for c++11 feature '${FEATURE_NAME}': ${CPP11_HAS_${FEATURE_NAME}}")
  if(CPP11_HAS_${FEATURE_NAME})
    set(CPP11_HAS_${FEATURE_NAME} TRUE PARENT_SCOPE)
  else()
    set(CPP11_HAS_${FEATURE_NAME} FALSE PARENT_SCOPE)
    file(WRITE ${PROJECT_BINARY_DIR}/cpp11/has_${FEATURE_NAME}.out ${OUT})
  endif()
  unset(CPP11_HAS_${FEATURE_NAME})
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

cpp11_feature_detection(CHRONO)
cpp11_feature_detection(UNIQUE_PTR)

# explicit configuration
if(
    ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    )
  set(CPP11_NO_CHRONO TRUE)
endif()

unset(CPP11_COMPILER_SWITCH)
