
(cl:in-package :asdf)

(defsystem "KBSL-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "FitBestPlaneSrv" :depends-on ("_package_FitBestPlaneSrv"))
    (:file "_package_FitBestPlaneSrv" :depends-on ("_package"))
    (:file "FitMinimalPlaneSrv" :depends-on ("_package_FitMinimalPlaneSrv"))
    (:file "_package_FitMinimalPlaneSrv" :depends-on ("_package"))
    (:file "FindInliersSrv" :depends-on ("_package_FindInliersSrv"))
    (:file "_package_FindInliersSrv" :depends-on ("_package"))
    (:file "TransformPointSrv" :depends-on ("_package_TransformPointSrv"))
    (:file "_package_TransformPointSrv" :depends-on ("_package"))
  ))