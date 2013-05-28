FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/kf_yaw_kf/msg"
  "../src/kf_yaw_kf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/rosbuild_premsgsrvgen"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rosbuild_premsgsrvgen.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
