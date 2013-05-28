FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/kf_yaw_kf/msg"
  "../src/kf_yaw_kf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/kf_yaw_kf/Compass.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
