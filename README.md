C/C++ template project for VA416xx based projects
========

# WORK IN PROGRESS

Currently, there are still issue with the Watchdog IRQ being triggered and the application
being stuck in it.

Steps to reproduce:

1. Build the application like specified below
2. Start the JLinkGDBServer with the provided JLinkScript file. You can
   use `jlink-gdb.sh` to do this
3. Use the following commands to load the application

   ```sh
   arm-none-eabi-gdb -q build/va416xx-template
   target remote localhost:2331
   load
   ```

4. Keep stepping. In the main function, after 1-2 steps, the `WDT_IRQHandler` will be triggered

# Prerequisites

1. ARM Cross-compiler installed and added to system path
2. CMake installed
3. SEGGER J-Link tools installed
4. Eclipse recommended, project files provided

# Build

Instructions for command line

1. Create build directory

   ```sh
   make build && cd build
   ```

2. Set up build system. Depending on the OS and desired build system, you might have
   to specify the build system explicitely (e.g. with `-G "MinGW Makefiles"` or
   `-G "Ninja"` on Windows) instead of using the defaults

   ```sh
   cmake ..
   ```

3. Build the project

   ```sh
   cmake --build . -j
   ```

# Flashing and Debugging

It is recommended to use Eclipse or VS code for convenient flashing, but the instructions here
show how to do it in the commnand line

1. Start the JLink GDB server.
2. Run the GDB application, for example `gdb-multiarch`

   ```sh
   gdb-multiarch -q -x jlink.gdb build/va416xx-template
   ```

3. The debugger should now be stopped at the start of the application. Press `C` to continue

# Using Eclipse

Copy the `.cproject` and `.project` files from the `eclipse` folder to the repository root.
Now open the folder in Eclipse as a project and make sure to deselect the `eclipse` folder in
the selection Window.

As long as the first two steps in the Build instructions above were performed, you should be able
to build and debug with the hammer and debug button conveniently now.
