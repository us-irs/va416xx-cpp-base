C/C++ template project for Vorago VA416xx based projects
========

It is recommended to use the RevB of the VA416xx chips, the  RevA boards have prove problematic
for development with non-Keil tools.

# Prerequisites

1. ARM Cross-compiler installed and added to system path
2. CMake installed
3. SEGGER J-Link tools installed
4. Eclipse or VS Code for development recommended, project files provided

# Note on RevA

For the RevA board, the JLink Server needs to be started with a JLinkScript file argument
to ensure that the ROM protection is disabled. Otherwise, flashing the chip won't work.

Furthermore, resetting the chip on boot is problematic for this revision as well.
It wasn't tested yet whether these problems occur on RevB as well.

# <a id="build"></a> Build

Instructions for command line. Make sure you can call the cross-compiler from the command line,
e.g. with `arm-none-eabi-gcc -v`.

1. Create build directory

   ```sh
   mmkdir build && cd build
   ```

2. Set up build system. Depending on the OS and desired build system, you might have
   to specify the build system explicitely (e.g. with `-G "MinGW Makefiles"` or
   `-G "Ninja"` on Windows) instead of using the defaults. There are also various build options
   available. See the [cmake](#cmake) section for more information.

   ```sh
   cmake ..
   ```

3. Build the project

   ```sh
   cmake --build . -j
   ```

# <a id="flashdebug"></a> Flashing and Debugging

It is recommended to use Eclipse or VS code for convenient flashing. The instructions here
show how to do it in the command line

1. Start the JLink GDB server, for example with `JLinkGDBServer` on Linux. You might need to pass
   the `-jlinkscriptfile ./jlink/JLinkSettings.JLinkScript` argument because
   this is required to disable the ROM protection, allowing flashing. See more information in
   the Note on RevA. A `jlink-gdb.sh` file was provided to perform this step conveniently

2. Run the GDB application, for example `gdb-multiarch` on Linux

   ```sh
   gdb-multiarch -q -x jlink.gdb build/va416xx-template
   ```

3. The debugger should now be stopped at the start of the application. Press `C` to continue

# Using VS Code

The only requirement on the [VS Code](https://code.visualstudio.com/) installation is the
following plugin:

- [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)

While this plugin is able to start the GDB application as well, it always resets the CPU in the
process. For now, it was decided that the user needs to start the GDB server themselves like
shown in step 1 of the [debug](#flashdebug) section.

Another requirement is that you still need to set up the Eclipse build configuration at least once
in the `build` folder like shown in the [build](#build) section. After that, you can use the
Run & Debug Tab of VS Code to debug your application.

# Using Eclipse

The only requirement on the [Eclipse](https://www.eclipse.org/downloads/packages/installer)
installation is the following plugin:

- [Eclipse Embedded CDT](https://projects.eclipse.org/projects/iot.embed-cdt). It is recommended to
   install the [xPacks](https://xpack.github.io/arm-none-eabi-gcc/install/) cross-compiler as it
   offers the best Eclipse support when used with this plugin

It is also recommended to install the following plugins

- [CMakeEditor](https://marketplace.eclipse.org/content/cmake-editor)

After installing all plugins, copy the `.cproject` and `.project` files from the `eclipse` folder
to the repository root. Now open the folder in Eclipse as a project and make sure to deselect the
`eclipse` folder in the selection Window.

Another requirement is that you still need to set up the Eclipse build configuration at least once
in the `build` folder like shown in the [build](#build) section. After that you should be able to
build and debug with the hammer and debug button conveniently now.

# <a id="cmake"></a> Additional CMake options

You can configure the build process by passing these options with `-D<Option>=<Value>`
to the `cmake` build generation call. This sections give an overview of the most
important build options. All configuration options will be copied to the `VORConfig.h`
file inside the build folder.

## Boolean options

Set value to `ON` or `OFF`:

- `GCC_USE_NANO_LIB`: Can be used to use `newlib-nano` for reduced code size.
- `GCC_NANOLIB_SCAN_FLOAT`: Only relevant when using `newlib-nano`. Allow scanning floats.
- `GCC_NANOLIB_PRINT_FLOAT`: Only relevant when using `newlib-nano`. Allow printing floats
- `VOR_ENABLE_RTT`: Enable logging via Segger-RTT

## Integer options

Set value to integer value

- `VOR_XTAL`, `VOR_HBO` and `VOR_EXTCLK`: Set fixed clock values for software which can be modified
  by hardware configuration
