include(GNUInstallDirs)

function(sclerp_target_warnings tgt)
  if (MSVC)
    target_compile_options(${tgt} PRIVATE /W4)
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()
