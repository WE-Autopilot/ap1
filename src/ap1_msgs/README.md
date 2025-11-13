# AP1 Messages

These contain common message types for all packages in AP1.

To use, modify your package.xml and CMakeLists.txt to include the following:

```xml
<!-- package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  ...
  <depend>ap1_msgs</depend>
  ...
</package>
```

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
...
find_package(ap1_msgs REQUIRED)
ament_target_dependencies(
  ...
  "ap1_msgs"
)
...
```

You can find examples of using ap1_msgs in other packages in https://github.com/WE-Autopilot/planning and https://github.com/WE-Autopilot/control.
