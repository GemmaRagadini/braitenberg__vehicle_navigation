
(cl:in-package :asdf)

(defsystem "husky_follow_light-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PerformAction" :depends-on ("_package_PerformAction"))
    (:file "_package_PerformAction" :depends-on ("_package"))
  ))