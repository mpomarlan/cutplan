; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cutplan
  :name "cutplan"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Code and simple test scenarios for cutplan."
  :depends-on (:cl-transforms-stamped
               :cl-tf
               :meshproc_msgs-srv
               :visualization_msgs-msg
               :cutplan-srv
               :ros-load-manifest
               :roslisp-utilities
               :roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "tests" :depends-on ("package"))
             (:file "cutplan-src" :depends-on ("package"))))))
