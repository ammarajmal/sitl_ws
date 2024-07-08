
(cl:in-package :asdf)

(defsystem "fast_cam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CameraSpecs" :depends-on ("_package_CameraSpecs"))
    (:file "_package_CameraSpecs" :depends-on ("_package"))
  ))