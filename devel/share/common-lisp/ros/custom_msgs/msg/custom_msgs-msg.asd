
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Speed_tick" :depends-on ("_package_Speed_tick"))
    (:file "_package_Speed_tick" :depends-on ("_package"))
  ))