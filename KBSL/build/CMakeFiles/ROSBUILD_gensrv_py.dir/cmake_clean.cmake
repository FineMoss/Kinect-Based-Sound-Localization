file(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/KBSL/msg"
  "../src/KBSL/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/KBSL/srv/__init__.py"
  "../src/KBSL/srv/_FitBestPlaneSrv.py"
  "../src/KBSL/srv/_FitMinimalPlaneSrv.py"
  "../src/KBSL/srv/_FindInliersSrv.py"
  "../src/KBSL/srv/_TransformPointSrv.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
