# -*- cmake -*- 

set(GIT_PROJECT YES)
set(CMAKE_GENERATED_MESSAGE "DO NOT EDIT: file automatically generated")

execute_process(COMMAND ${GIT_EXECUTABLE} describe --all
  RESULT_VARIABLE GIT_RET
  OUTPUT_VARIABLE GIT_WC_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(GIT_RET EQUAL 0)
  message(STATUS "git branch: ${GIT_WC_BRANCH}")

  execute_process(COMMAND ${GIT_EXECUTABLE} describe --always --dirty
    RESULT_VARIABLE GIT_RET
    OUTPUT_VARIABLE GIT_WC_REV
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if(GIT_RET EQUAL 0)
    message(STATUS "Git version: [${GIT_WC_BRANCH}:${GIT_WC_REV}]")
  else()
    message(WARNING "Failed to get git revision info")
    set(GIT_WC_REV "unknown")
  endif()
else()
  message(WARNING "Failed to get git branch")
  unset(GIT_WC_BRANCH)
  set(GIT_WC_REV "unknown")
endif()

configure_file(pkg/git_version.hh.in ${OUTPUT_FILE})