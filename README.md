# SHS-Robotics

Robotics code for SHS Robotics Club

# Commit Instructions

Before committing VEX Coding Studio project files (`<project_name>.vex`) please unpack their contents into `<project_name>.contents`
directory using `unpack_vex_project.py <project_name>.vex`.  

Git cannot track changes in `.vex` files since they are binary files.
Unpacking them into component source files in that directory allows tracking of the code changes.

For example:
```
SHS-Robotics>python unpack_vex_project.py "Arcade Drive C++.vex" --log_level INFO
INFO:__main__:Deleting Arcade Drive C++.contents\config.json
INFO:__main__:Deleting Arcade Drive C++.contents\main.cpp
INFO:__main__:Deleting Arcade Drive C++.contents\robot-config.h
INFO:__main__:Wrote Arcade Drive C++.contents\config.json
INFO:__main__:Wrote Arcade Drive C++.contents\robot-config.h
INFO:__main__:Wrote Arcade Drive C++.contents\main.cpp
```

More help on usage:
```
SHS-Robotics>python unpack_vex_project.py --help
usage: unpack_vex_project.py [-h] [--out_dir OUT_DIR] [--log_level LOG_LEVEL]
                             <project_name>.vex

Unpack contents of the VEX Coding Studio's binary project file and save its
component source code files in a directory. This is useful to simplify
tracking of the source code changes.

positional arguments:
  <project_name>.vex    VEX project file.

optional arguments:
  -h, --help            show this help message and exit
  --out_dir OUT_DIR     Output directory, defaults to
                        "<project_name>.contents". If exists, all files in it
                        will be deleted first.
  --log_level LOG_LEVEL
                        Set log level, one of WARN (default), INFO, DEBUG.
```
