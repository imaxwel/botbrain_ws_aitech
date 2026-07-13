#!/bin/bash
# Install rmw_zenoh_cpp from volume into container at startup
ZENOH_LIBS=/botbrain_ws/zenoh_libs
[ -d "$ZENOH_LIBS" ] || exit 0

cp -f "$ZENOH_LIBS/librmw_zenoh_cpp.so" /opt/ros/humble/lib/
cp -f "$ZENOH_LIBS/libzenohc.so" /usr/local/lib/
mkdir -p /opt/ros/humble/lib/rmw_zenoh_cpp
cp -f "$ZENOH_LIBS/rmw_zenohd" /opt/ros/humble/lib/rmw_zenoh_cpp/
chmod 755 /opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd

# ament index
mkdir -p /opt/ros/humble/share/ament_index/resource_index/rmw_typesupport_c
mkdir -p /opt/ros/humble/share/ament_index/resource_index/rmw_typesupport_cpp
mkdir -p /opt/ros/humble/share/ament_index/resource_index/packages
mkdir -p /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path
mkdir -p /opt/ros/humble/share/ament_index/resource_index/rmw_typesupport
mkdir -p /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies
echo "rosidl_typesupport_c;rosidl_typesupport_fastrtps_c;rosidl_typesupport_introspection_c" > /opt/ros/humble/share/ament_index/resource_index/rmw_typesupport_c/rmw_zenoh_cpp
echo "rosidl_typesupport_cpp;rosidl_typesupport_fastrtps_cpp;rosidl_typesupport_introspection_cpp" > /opt/ros/humble/share/ament_index/resource_index/rmw_typesupport_cpp/rmw_zenoh_cpp
touch /opt/ros/humble/share/ament_index/resource_index/packages/rmw_zenoh_cpp
echo "{prefix}" > /opt/ros/humble/share/ament_index/resource_index/parent_prefix_path/rmw_zenoh_cpp
echo "rosidl_typesupport_c;rosidl_typesupport_fastrtps_c;rosidl_typesupport_introspection_c;rosidl_typesupport_cpp;rosidl_typesupport_fastrtps_cpp;rosidl_typesupport_introspection_cpp" > /opt/ros/humble/share/ament_index/resource_index/rmw_typesupport/rmw_zenoh_cpp
echo "zenoh_cpp_vendor;ament_index_cpp;fastcdr;rcpputils;rcutils;rosidl_typesupport_fastrtps_c;rosidl_typesupport_fastrtps_cpp;rmw;tracetools" > /opt/ros/humble/share/ament_index/resource_index/package_run_dependencies/rmw_zenoh_cpp

cp -rf "$ZENOH_LIBS/share_rmw_zenoh_cpp" /opt/ros/humble/share/rmw_zenoh_cpp 2>/dev/null || true
ldconfig
