FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/kf_yaw_kf/msg"
  "../src/kf_yaw_kf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/kf_yaw_kf/srv/__init__.py"
  "../src/kf_yaw_kf/srv/_SetMagOffset.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
