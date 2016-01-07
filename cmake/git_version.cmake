set(GIT_PROJECT NO)

if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/.git)
  set(GIT_PROJECT YES)
  find_package(Git QUIET)
  message(STATUS "Looking for git: ${GIT_EXECUTABLE}")
  if(NOT GIT_EXECUTABLE)
    set(GIT_PROJECT NO)
  endif()
endif()

message(STATUS "Track git revision: ${GIT_PROJECT}")


  


if(GIT_PROJECT)
  add_custom_target(get_git)
  add_custom_command(TARGET get_git PRE_BUILD
    COMMAND ${CMAKE_COMMAND}
    -DOUTPUT_FILE=${CMAKE_BINARY_DIR}/git/git_version.hh
    -DGIT_EXECUTABLE=${GIT_EXECUTABLE}
    -P ${CMAKE_SOURCE_DIR}/cmake/git_check.cmake
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Checking git version info")


  macro(git_header file)
    add_custom_command(OUTPUT ${file} 
      DEPENDS get_git ${CMAKE_BINARY_DIR}/git/git_version.hh
      COMMAND ${CMAKE_COMMAND}
      ARGS -E copy_if_different ${CMAKE_BINARY_DIR}/git/git_version.hh ${file})
    set_source_files_properties(${file}
      PROPERTIES GENERATED TRUE
      LOCATION ${CMAKE_CURRENT_BINARY_DIR})
  endmacro(git_header)
else(GIT_PROJECT)
  macro(git_header file)
    set(GIT_WC_REV "unknown")
    configure_file(${CMAKE_SOURCE_DIR}/pkg/git_version.hh.in
      ${file})
    set_source_files_properties(${file}
      PROPERTIES
      LOCATION ${CMAKE_CURRENT_BINARY_DIR})
  endmacro(git_header)
endif(GIT_PROJECT)
  