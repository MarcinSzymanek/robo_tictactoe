
(cl:in-package :asdf)

(defsystem "tic_tac_toebot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ManualMoveTo" :depends-on ("_package_ManualMoveTo"))
    (:file "_package_ManualMoveTo" :depends-on ("_package"))
    (:file "StartGame" :depends-on ("_package_StartGame"))
    (:file "_package_StartGame" :depends-on ("_package"))
  ))