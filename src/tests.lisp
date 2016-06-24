;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :cutplan)

(defparameter *pub-mrk* nil)

(defparameter *base-link* "/odom_combined")

(defparameter *identity-pose* nil)
(defparameter *identity-pose-msg* nil)

(defparameter *pizza-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.7 :b 0.3))

(defparameter *slice-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.3 :g 0.8 :b 0.3))

(defparameter *cut-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.3 :b 0.3))

(defun tr->ps (transform)
  (let* ((v (cl-transforms:translation transform))
         (r (cl-transforms:rotation transform))
         (x (cl-transforms:x v))
         (y (cl-transforms:y v))
         (z (cl-transforms:z v))
         (qx (cl-transforms:x r))
         (qy (cl-transforms:y r))
         (qz (cl-transforms:z r))
         (qw (cl-transforms:w r)))
    (roslisp:make-message "geometry_msgs/Pose"
      :position (roslisp:make-message "geometry_msgs/Point" :x x :y y :z z)
      :orientation (roslisp:make-message "geometry_msgs/Quaternion" :x qx :y qy :z qz :w qw))))

(defun make-mrk-msg (base-frame-name frame-locked id action pose color mesh-resource)
  (roslisp:make-message "visualization_msgs/Marker"
                        :header (roslisp:make-message "std_msgs/Header" 
                                                      :frame_id base-frame-name :stamp 0)
                                               :ns "cutplan"
                                               :id id
                                               :frame_locked frame-locked
                                               :type 10
                                               :action action
                                               :pose pose
                                               :scale (roslisp:make-message "geometry_msgs/Vector3"
                                                                            :x 1 :y 1 :z 1)
                                               :color color
                                               :mesh_resource mesh-resource))

(defun cut-test (&optional (visualization-topic "/visualization_marker"))
  (cutplan:start-cutplan-node)
  (setf *pub-mrk* (roslisp:advertise visualization-topic "visualization_msgs/Marker" :latch nil))
  (roslisp:call-service "/meshproc_csg/UnloadMesh" "meshproc_msgs/UnloadMesh"
                        :mesh_name "pizza")
  (roslisp:call-service "/meshproc_csg/UnloadMesh" "meshproc_msgs/UnloadMesh"
                        :mesh_name "slice")
  (roslisp:call-service "/meshproc_csg/UnloadMesh" "meshproc_msgs/UnloadMesh"
                        :mesh_name "dilated-slice")
  (roslisp:call-service "/meshproc_csg/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "pizza"
                        :mesh_filenames (list (format nil "~ameshes/pizza.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
                        :duplicate_dist 0.0001)
  (roslisp:call-service "/meshproc_csg/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "slice"
                        :mesh_filenames (list (format nil "~ameshes/slice.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
                        :duplicate_dist 0.0001)
  (roslisp:call-service "/meshproc_csg/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "dilated-slice"
                        :mesh_filenames (list (format nil "~ameshes/dilated-slice.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
                        :duplicate_dist 0.0001)
  (let* ((result (roslisp:call-service "/cutplan/GetManeuver" 'cutplan-srv:GetManeuver
                                       :maneuver "cut"
                                       :object "pizza"
                                       :target "slice"
                                       :dilated_target "dilated-slice"
                                       :forbidden ""
                                       :maneuver_mesh "cut-mesh"
                                       :new_target "pizza-slice"
                                       :new_object "pizza-remainder")))
    (roslisp:wait-duration 1.0)
    (setf *identity-pose* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.0 0.0 0.0)
                                                      (cl-transforms:make-quaternion 0 0 0 1)))
    (roslisp:call-service "/meshproc_csg/GetMesh" "meshproc_msgs/GetMesh"
                          :mesh_name "pizza-remainder"
                          :return_result 0
                          :result_to_file 1
                          :result_filename (format nil "~a/outputs/new-pizza.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
    (roslisp:call-service "/meshproc_csg/GetMesh" "meshproc_msgs/GetMesh"
                          :mesh_name "pizza-slice"
                          :return_result 0
                          :result_to_file 1
                          :result_filename (format nil "~a/outputs/new-slice.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
    (roslisp:call-service "/meshproc_csg/GetMesh" "meshproc_msgs/GetMesh"
                          :mesh_name "cut-mesh"
                          :return_result 0
                          :result_to_file 1
                          :result_filename (format nil "~a/outputs/new-cut.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
    (setf *identity-pose-msg* (tr->ps *identity-pose*))
    (roslisp:publish *pub-mrk* (make-mrk-msg *base-link* 0 1 0 *identity-pose-msg* *pizza-color* "package://cutplan/outputs/new-pizza.stl"))
    (roslisp:publish *pub-mrk* (make-mrk-msg *base-link* 0 2 0 *identity-pose-msg* *slice-color* "package://cutplan/outputs/new-slice.stl"))
    (roslisp:publish *pub-mrk* (make-mrk-msg *base-link* 0 3 0 *identity-pose-msg* *cut-color* "package://cutplan/outputs/new-cut.stl"))
    (roslisp:with-fields (parameters) result
      (coerce parameters 'list))))


