
(cl:in-package :asdf)

(defsystem "kf_yaw_kf-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetMagOffset" :depends-on ("_package_SetMagOffset"))
    (:file "_package_SetMagOffset" :depends-on ("_package"))
  ))