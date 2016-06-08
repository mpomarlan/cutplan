; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cutplan-demo
  :name "cutplan-demo"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Simple test scenarios for cutplan."
  :depends-on (:cl-transforms-stamped
               :cl-tf
               :ros-load-manifest
               :roslisp-utilities
               :roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "tests" :depends-on ("package"))))))
