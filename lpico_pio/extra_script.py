Import("projenv")
Import("env")

import os
import sys
import shutil

# Project root
main_path = os.path.realpath('.')

# Ensure Python can import microros helper package from extra_packages
extra_packages_path = os.path.join(main_path, 'extra_packages')
if os.path.isdir(extra_packages_path):
    # add extra_packages itself
    if extra_packages_path not in sys.path:
        sys.path.insert(0, extra_packages_path)
    # also add any subfolders (for example extra_packages/micro_ros_platformio)
    for entry in sorted(os.listdir(extra_packages_path)):
        p = os.path.join(extra_packages_path, entry)
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)

##########################
#### Global variables ####
##########################

boards_metas = {
    "portenta_h7_m7": "colcon.meta",
    "nanorp2040connect": "colcon_verylowmem.meta",
    "teensy41": "colcon.meta",
    "teensy40": "colcon.meta",
    "teensy36": "colcon_lowmem.meta",
    "teensy35": "colcon_lowmem.meta",
    "teensy32": "colcon_lowmem.meta",
    "teensy31": "colcon_lowmem.meta",
    "esp32dev": "colcon.meta",
    "olimex_e407": "colcon.meta",
    "due": "colcon_verylowmem.meta",
    "zero": "colcon_verylowmem.meta",
    "pico": "colcon.meta",
}

project_options = env.GetProjectConfig().items(env=env["PIOENV"], as_dict=True)
global_env = DefaultEnvironment()
board = env['BOARD']
framework = env['PIOFRAMEWORK'][0]

selected_board_meta = boards_metas.get(board, "colcon.meta")

# Default micro-ROS configuration
microros_distro = global_env.BoardConfig().get("microros_distro", "kilted")
microros_transport = global_env.BoardConfig().get("microros_transport", "serial")
microros_user_meta = os.path.join(env['PROJECT_DIR'], global_env.BoardConfig().get("microros_user_meta", ""))

# Ensure SRC_FILTER is present and in list form
def ensure_src_filter():
    cur = env.get('SRC_FILTER', None)
    if cur is None:
        env['SRC_FILTER'] = []
    elif isinstance(cur, str):
        env['SRC_FILTER'] = [cur]

ensure_src_filter()
env['SRC_FILTER'].append('-<build/include/*>')

################################
#### Library custom targets ####
################################

def clean_microros_callback(*args, **kwargs):
    library_path = os.path.join(main_path, 'libmicroros')
    build_path = os.path.join(main_path, 'build')
    shutil.rmtree(library_path, ignore_errors=True)
    shutil.rmtree(build_path, ignore_errors=True)
    print("micro-ROS library cleaned!")
    os._exit(0)

if "clean_microros" not in global_env.get("__PIO_TARGETS", {}):
    global_env.AddCustomTarget("clean_microros", None, clean_microros_callback, title="Clean Micro-ROS", description="Clean Micro-ROS build environment")

def clean_libmicroros_callback(*args, **kwargs):
    library_path = os.path.join(main_path, 'libmicroros')
    shutil.rmtree(library_path, ignore_errors=True)
    print("libmicroros cleaned")
    os._exit(0)

if "clean_libmicroros" not in global_env.get("__PIO_TARGETS", {}):
    global_env.AddCustomTarget("clean_libmicroros", None, clean_libmicroros_callback, title="Clean libmicroros", description="Clean libmicroros")


def build_microros(*args, **kwargs):
    # Ensure required Python packages in the PlatformIO venv
    pip_packages = [x.split("==")[0] for x in os.popen('{} -m pip freeze'.format(env['PYTHONEXE'])).read().split('\n')]
    required_packages = ["catkin-pkg", "lark-parser", "colcon-common-extensions", "importlib-resources", "pyyaml", "pytz", "markupsafe==2.0.1", "empy==3.3.4"]
    for p in [x for x in required_packages if x.split('==')[0] not in pip_packages]:
        print('Installing {} with pip at PlatformIO environment'.format(p))
        env.Execute('$PYTHONEXE -m pip install {}'.format(p))

    # Import the micro-ROS builder from the extra_packages path
    import microros_utils.library_builder as library_builder

    print("Configuring {} with transport {}".format(board, microros_transport))

    cmake_toolchain = library_builder.CMakeToolchain(
        os.path.join(main_path, "platformio_toolchain.cmake"),
        env['CC'],
        env['CXX'],
        env['AR'],
        "{} {} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(' '.join(env['CFLAGS']), ' '.join(env['CCFLAGS'])) ,
        "{} {} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='".format(' '.join(env['CXXFLAGS']), ' '.join(env['CCFLAGS']))
    )

    python_env_path = env['PROJECT_CORE_DIR'] + "/penv/bin/activate"
    builder = library_builder.Build(library_folder=main_path, packages_folder=extra_packages_path, distro=microros_distro, python_env=python_env_path)
    builder.run(os.path.join(main_path, 'metas', selected_board_meta), cmake_toolchain.path, microros_user_meta)

    # Add library/includes to environment
    if board in ("portenta_h7_m7", "nanorp2040connect", "pico"):
        global_env['_LIBFLAGS'] = "-Wl,--start-group " + global_env.get('_LIBFLAGS', '') + " -l{} -Wl,--end-group".format(builder.library_name)
    else:
        global_env.Append(LIBS=[builder.library_name])

    global_env.Append(LIBPATH=[builder.library_path])


def update_env():
    global_env.Append(CPPDEFINES=[("CLOCK_MONOTONIC", 1)])
    global_env.Append(CPPPATH=[os.path.join(main_path, "libmicroros", "include")])
    env.Append(CPPPATH=[os.path.join(main_path, "libmicroros", "include")])
    global_env.Append(CPPPATH=[os.path.join(main_path, "platform_code"), os.path.join(main_path, "platform_code", framework)])
    env.Append(CPPPATH=[os.path.join(main_path, "platform_code"), os.path.join(main_path, "platform_code", framework)])

    # Add micro-ROS defines
    try:
        projenv.Append(CPPDEFINES=[('MICRO_ROS_TRANSPORT_{}_{}'.format(framework.upper(), microros_transport.upper()), 1)])
        projenv.Append(CPPDEFINES=[('MICRO_ROS_DISTRO_{} '.format(microros_distro.upper()), 1)])
    except Exception:
        pass

    # Add clock and transport sources into SRC_FILTER safely
    ensure_src_filter()
    env['SRC_FILTER'].append(' +<platform_code/{}/clock_gettime.cpp>'.format(framework))
    env['SRC_FILTER'].append(' +<platform_code/{}/{}/micro_ros_transport.cpp>'.format(framework, microros_transport))


from SCons.Script import COMMAND_LINE_TARGETS

# Do not build library on clean targets or when IDE fetches metadata
if set(["clean_microros", "clean_libmicroros", "_idedata", "idedata"]).isdisjoint(set(COMMAND_LINE_TARGETS)):
    build_microros()

update_env()
