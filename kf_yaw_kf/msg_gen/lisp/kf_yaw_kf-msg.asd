
(cl:in-package :asdf)

(defsystem "kf_yaw_kf-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Compass" :depends-on ("_package_Compass"))
    (:file "_package_Compass" :depends-on ("_package"))
  ))