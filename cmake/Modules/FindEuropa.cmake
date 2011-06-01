# - Try to find europa-pso
# Once done this will define
#  EUROPA_FOUND - System has LibXml2
#  EUROPA_INCLUDE_DIRS - The LibXml2 include directories
#  EUROPA_LIBRARIES - The libraries needed to use LibXml2
#  EUROPA_DEFINITIONS - Compiler switches required for using LibXml2

set(_europa_HINTS
  $ENV{EUROPA_HOME})


set(_europa_VARIANT "_o") #only look for optimized 
set(_europa_LIBRARIES ConstraintEngine Solvers NDDL System PlanDatabase 
  TemporalNetwork Resources TinyXml RulesEngine Utils)

set(EUROPA_FLAGS TIXML_USE_STL;EUROPA_FAST)

if(NOT Europa_FIND_COMPONENTS)
  # we need everything then
  set(Europa_FIND_COMPONENTS ${_europa_LIBRARIES})
endif(NOT Europa_FIND_COMPONENTS)

set(_europa_MISSING "")
set(EUROPA_LIB_NAMES "")
set(EUROPA_LIBRARIES "")
set(EUROPA_LIBRARY_DIRS "")



foreach(COMPONENT ${Europa_FIND_COMPONENTS})
  string(TOUPPER ${COMPONENT} UPPERCOMPONENT)
  find_library(EUROPA_${UPPERCOMPONENT}_LIBRARY
    NAMES ${COMPONENT}${_europa_VARIANT}
    HINTS ${_europa_HINTS}/lib
    DOCS "Looking for ${COMPONENT}")
  if(EUROPA_${UPPERCOMPONENT}_LIBRARY)
    set(EUROPA_${UPPERCOMPONENT}_NAME ${COMPONENT}${_europa_VARIANT})
    list(APPEND EUROPA_LIBRARIES ${EUROPA_${UPPERCOMPONENT}_LIBRARY})
    get_filename_component(_europa_my_lib_path "${EUROPA_${UPPERCOMPONENT}_LIBRARY}" 
      PATH)
    list(APPEND EUROPA_LIBRARY_DIRS ${_europa_my_lib_path})
  else(EUROPA_${UPPERCOMPONENT}_LIBRARY)
    list(APPEND _europa_MISSING ${COMPONENT})
  endif(EUROPA_${UPPERCOMPONENT}_LIBRARY)
endforeach(COMPONENT)

list(REMOVE_DUPLICATES EUROPA_LIBRARY_DIRS)

list(REMOVE_DUPLICATES _europa_MISSING)
list(REMOVE_DUPLICATES EUROPA_LIBRARIES)

find_path(EUROPA_INCLUDE_DIR 
  "PSSolvers.hh" HINT ${_europa_HINTS}/include)

set(EUROPA_INCLUDE_DIRS
  ${EUROPA_INCLUDE_DIR}
  )

if(_europa_MISSING) 
  set(EUROPA_FOUND FALSE)
  if(Europa_FIND_REQUIRED)
    message(SEND_ERROR "Unable to find the requested europa libraries")
  endif(Europa_FIND_REQUIRED)
  message(STATUS "Europa not found\nSet your EUROPA_HOME if you want ot compile this plugin")
else(_europa_MISSING)
  set(EUROPA_FOUND TRUE)
  message(STATUS "Europa found : include directory is ${EUROPA_INCLUDE_DIR}")
endif(_europa_MISSING)


mark_as_advanced(EUROPA_INCLUDE_DIR
  EUROPA_INCLUDE_DIRS
  EUROPA_LIBRARIES
  EUROPA_FLAGS
  EUROPA_LIB_NAMES
  EUROPA_LIBRARY_DIRS
  )
