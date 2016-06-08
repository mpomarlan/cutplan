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

(in-package :cutplan-demo)

(defparameter *pub-mrk* nil)

(defparameter *base-link* "/odom_combined")

(defparameter *identity-pose* nil)
(defparameter *identity-pose-msg* nil)

(defparameter *crstep* 0)

(defparameter *goal-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.7 :b 0.3))


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

(defun init-brush-test (goal-mesh-path &optional (visualization-topic "/visualization_marker"))
  (roslisp:call-service "/cutplan/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "goal"
                        :mesh_filenames (list goal-mesh-path)
                        :duplicate_dist 0.0001)
  (setf *pub-mrk* (roslisp:advertise visualization-topic "visualization_msgs/Marker" :latch nil))
  (setf *identity-pose* (cl-transforms:make-transform (cl-transforms:make-3d-vector 0.0 0.0 0.0)
                                                      (cl-transforms:make-quaternion 0 0 0 1)))
  (setf *crstep* 0)
  (setf *identity-pose-msg* (tr->ps *identity-pose*)))


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

(defun step-brush-test ()
  (let* ((maneuver (roslisp:call-service "/cutplan/GetManeuver" "cutplan/GetManeuver"
                                         :maneuver "brush"
                                         :goal "goal"
                                         :forbidden ""
                                         :goal_minus "goal_minus"
                                         :new_goal "goal"))
         (dummy (roslisp:call-service "/cutplan/GetMesh" "meshproc_msgs/GetMesh"
                                      :mesh_name "goal"
                                      :result_to_file 1
                                      :result_filename (format nil "~aoutputs/goal_~a_out.stl" (namestring (ros-load-manifest:ros-package-path "cutplan")) *crstep*)))
         (goal-msg (make-mrk-msg *base-link* 0 1 0
                                 *identity-pose-msg*
                                 *goal-color*
                                 (format nil "package://cutplan/outputs/goal_~a_out.stl" *crstep*))))
    (declare (ignore dummy))
    (setf *crstep* (+ *crstep* 1))
    (roslisp:publish *pub-mrk* goal-msg)
    (roslisp:with-fields (parameters) maneuver
      parameters)))

