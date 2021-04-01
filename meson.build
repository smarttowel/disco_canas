project('discoled', 'c', 'cpp',
    version : '0.1',
    default_options : ['warning_level=3', 'cpp_std=c++17'])

subdir('cube_dep')

canas_proj = subproject('canas')
canas_raw_dep = canas_proj.get_variable('canas_raw_dep')

elf = executable('discoled.elf',
                 'ledtask.cpp',
                 'canasdevice.cpp',
                 'FreeRTOS-openocd.c',
                 'cube/Core/Src/main.cpp',
                  dependencies: [canas_raw_dep,
                                 cube_dep],
                  install : false)

hex = custom_target('discoled.hex',
                     depends : elf,
                     input : elf,
                     output : 'discoled.hex',
                     command : find_program('objcopy'),
                     build_by_default : true)

run_target('flash', depends: hex,
                    command: [find_program('flash.sh', dirs: meson.source_root() / 'scripts'),
                              hex.full_path()])