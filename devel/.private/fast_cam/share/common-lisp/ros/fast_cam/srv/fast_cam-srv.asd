
(cl:in-package :asdf)

(defsystem "fast_cam-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetCameraProperties" :depends-on ("_package_GetCameraProperties"))
    (:file "_package_GetCameraProperties" :depends-on ("_package"))
    (:file "SetGain" :depends-on ("_package_SetGain"))
    (:file "_package_SetGain" :depends-on ("_package"))
  ))