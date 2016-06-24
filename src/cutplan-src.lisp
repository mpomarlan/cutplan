;;;
;;; Copyright (c) 2016, Mihai Pomarlan <blandc@cs.uni-bremen.com>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(defun recognized (maneuver)
  (equal maneuver "cut"))

(defun is-mesh-loaded (mesh-name)
  (roslisp:with-fields ((mesh-loaded mesh_loaded)) 
                       (roslisp:call-service "/meshproc_csg/GetMesh" "meshproc_msgs/GetMesh" :mesh_name mesh-name)
                       (if (equal mesh-loaded 0) nil t)))

(defun get-skeleton-parameters (vertices edge-ends-l edge-ends-r)
  (let* ((edge-ends-l (coerce edge-ends-l 'list))
         (edge-ends-r (coerce edge-ends-r 'list))
         (vertices (coerce vertices 'vector))
         (result (apply #'append
                        (mapcar (lambda (e-l e-r)
                                  (let* ((e-l (aref vertices e-l))
                                         (e-r (aref vertices e-r)))
                                    (roslisp:with-fields ((x-l x) (y-l y) (z-l z)) e-l
                                      (roslisp:with-fields ((x-r x) (y-r y) (z-r z)) e-r
                                        (list x-l y-l z-l x-r y-r z-r)))))
                                edge-ends-l edge-ends-r))))
    (coerce result 'vector)))

(roslisp:def-service-callback GetManeuver (maneuver object target dilated_target forbidden maneuver_mesh new_target new_object)
  (let* ((dilated-target dilated_target)
         (maneuver-mesh maneuver_mesh)
         (new-target new_target)
         (new-object new_object))
    (let* ((new-object (if (equal new-object "") object new-object))
           (new-target (if (equal new-target "") (format nil "~a-~a-from-~a" maneuver target object) new-target))
           (maneuver-mesh (if (equal maneuver-mesh "") (format nil "~a-maneuver-on-~a-for-~a" maneuver object target) maneuver-mesh))
           (dilated-target-intersected (format nil "~a-intersected" dilated-target))
           (mesh-object-loaded (is-mesh-loaded object))
           (mesh-target-loaded (is-mesh-loaded target))
           (mesh-forbidden-loaded (if (not (equal forbidden ""))
                                    (is-mesh-loaded forbidden)
                                    T)))
      (if (not (recognized maneuver))
        (roslisp:make-response :have_maneuver 0
                               :mesh_object_loaded mesh-object-loaded
                               :mesh_target_loaded mesh-target-loaded
                               :mesh_forbidden_loaded mesh-forbidden-loaded
                               :operation_done 0
                               :parameters (vector))
        (cond
          ((not (and mesh-object-loaded mesh-target-loaded mesh-forbidden-loaded))
            (roslisp:make-response :have_maneuver 0
                                   :mesh_object_loaded mesh-object-loaded
                                   :mesh_target_loaded mesh-target-loaded
                                   :mesh_forbidden_loaded mesh-forbidden-loaded
                                   :operation_done 0
                                   :parameters (vector)))
          ((equal maneuver "cut")
            (if (not (is-mesh-loaded dilated-target))
              (roslisp:call-service "/meshproc_csg/CSGRequest" "meshproc_msgs/CSGRequest"
                                    :operation 4
                                    :mesh_A target
                                    :mesh_B "mini-cube"
                                    :mesh_R dilated-target))
            (roslisp:call-service "/meshproc_csg/CSGRequest" "meshproc_msgs/CSGRequest"
                                  :operation 1
                                  :mesh_A object
                                  :mesh_B dilated-target
                                  :mesh_R dilated-target-intersected)
            (roslisp:call-service "/meshproc_csg/CSGRequest" "meshproc_msgs/CSGRequest"
                                  :operation 1
                                  :mesh_A object
                                  :mesh_B target
                                  :mesh_R new-target)
            (roslisp:call-service "/meshproc_csg/CSGRequest" "meshproc_msgs/CSGRequest"
                                  :operation 2
                                  :mesh_A object
                                  :mesh_B dilated-target-intersected
                                  :mesh_R new-object)
            (roslisp:call-service "/meshproc_csg/CSGRequest" "meshproc_msgs/CSGRequest"
                                  :operation 2
                                  :mesh_A dilated-target-intersected
                                  :mesh_B target
                                  :mesh_R maneuver-mesh)
            (roslisp:with-fields (vertices (edge-ends-l edge_ends_L) (edge-ends-r edge_ends_R))
                                 (roslisp:call-service "/meshproc_csg/GetMeshSkeleton" "meshproc_msgs/GetMeshSkeleton"
                                                       :mesh_name maneuver-mesh
                                                       :approximate 1
                                                       :duplicate_distance 0.005)
                                 (format t "yay!")
                                 (roslisp:make-response :have_maneuver 1
                                                        :mesh_object_loaded (if mesh-object-loaded 1 0)
                                                        :mesh_target_loaded (if mesh-target-loaded 1 0)
                                                        :mesh_forbidden_loaded (if mesh-forbidden-loaded 1 0)
                                                        :operation_done 1
                                                        :parameters (get-skeleton-parameters vertices edge-ends-l edge-ends-r))))
          (T
            (roslisp:ros-info (basics-system) "CUTPLAN: Shouldn't have gotten to the default branch.")
            (roslisp:make-response :have_maneuver 0
                                   :mesh_object_loaded mesh-object-loaded
                                   :mesh_target_loaded mesh-target-loaded
                                   :mesh_forbidden_loaded mesh-forbidden-loaded
                                   :operation_done 0
                                   :parameters (vector))))))))

(defun start-cutplan-node (&optional (name "cutplan"))
  (roslisp:start-ros-node name)
  (roslisp:call-service "/meshproc_csg/LoadMesh" "meshproc_msgs/LoadMesh"
                        :mesh_name "mini-cube"
                        :mesh_filenames (vector (format nil "~ameshes/mini-cube.stl" (namestring (ros-load-manifest:ros-package-path "cutplan"))))
                        :duplicate_dist 0.00001)
;;  (roslisp:register-service (format nil "~a/LoadMesh" name) 'LoadMesh)
;;  (roslisp:register-service (format nil "~a/UnloadMesh" name) 'UnloadMesh)
;;  (roslisp:register-service (format nil "~a/GetLoadedMeshNames" name) 'GetLoadedMeshNames)
;;  (roslisp:register-service (format nil "~a/GetMesh" name) 'GetMesh)
  (roslisp:register-service (format nil "~a/GetManeuver" name) 'GetManeuver)
;;  (roslisp:register-service-fn (format nil "/~a/GetManeuver" name) #'GetManeuver 'cutplan-srv:GetManeuver)
)

