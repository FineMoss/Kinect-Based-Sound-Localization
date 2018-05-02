
(cl:in-package :asdf)

(defsystem "KBSL-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PlaneParametersMsg" :depends-on ("_package_PlaneParametersMsg"))
    (:file "_package_PlaneParametersMsg" :depends-on ("_package"))
  ))