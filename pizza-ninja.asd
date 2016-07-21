; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem pizza-ninja
  :name "pizza-ninja"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Uses giskard to perform cutting actions."
  :depends-on (:cl-transforms-stamped
               :alexandria
               :cl-tf
               :meshproc_msgs-srv
               :visualization_msgs-msg
               :cutplan-srv
               :cutplan
               :cl-giskard
               :ros-load-manifest
               :roslisp-utilities
               :roslisp)
  :components
  ((:module "src"
            :components
            ((:file "ninja-package")
             (:file "pizza-ninja" :depends-on ("ninja-package"))))))
