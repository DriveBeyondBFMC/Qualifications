
if(NOT "/home/eric/Embedded_Platform-master/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitinfo.txt" IS_NEWER_THAN "/home/eric/Embedded_Platform-master/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: '/home/eric/Embedded_Platform-master/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/eric/Embedded_Platform-master/build/_deps/greentea-client-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/eric/Embedded_Platform-master/build/_deps/greentea-client-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/ARMmbed/greentea-client.git" "greentea-client-src"
    WORKING_DIRECTORY "/home/eric/Embedded_Platform-master/build/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/ARMmbed/greentea-client.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"  checkout 472aad2327fbfde827852fc44775904706847a3a --
  WORKING_DIRECTORY "/home/eric/Embedded_Platform-master/build/_deps/greentea-client-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '472aad2327fbfde827852fc44775904706847a3a'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git"  submodule update --recursive --init 
    WORKING_DIRECTORY "/home/eric/Embedded_Platform-master/build/_deps/greentea-client-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/eric/Embedded_Platform-master/build/_deps/greentea-client-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "/home/eric/Embedded_Platform-master/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitinfo.txt"
    "/home/eric/Embedded_Platform-master/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/eric/Embedded_Platform-master/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitclone-lastrun.txt'")
endif()

