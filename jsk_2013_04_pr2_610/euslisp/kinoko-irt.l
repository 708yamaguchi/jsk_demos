#!/usr/bin/env roseus

;; Main
(load "package://jsk_2013_04_pr2_610/euslisp/setup.l")
;; Other objects
;; $ ls /opt/ros/noetic/share/euslisp/jskeus/eus/models
(load "package://euslisp/jskeus/eus/models/square-600-closet-object.l")
(load "package://euslisp/jskeus/eus/models/room73b2-dish-object.l")
(load "package://euslisp/jskeus/eus/models/toshiba-microwave-oven-object.l")
(load "package://euslisp/jskeus/eus/models/coffee-cup-cup-object.l")
;; Robot arm module
(load "package://jsk_2013_04_pr2_610/euslisp/gen3_lite_gen3_lite_2f.l")

(defun setup-kinoko ()
  ;; Setup normal IRT environment
  (setup)

  ;; Additional objects for kinoko IRT
  ;; Closet
  (setq *closet* (square-600-closet))
  (send *closet* :translate #f(1700 2500 0) :world)
  (send *closet* :rotate -pi/2 :z :local)
  ;; TODO: cloth and hanger
  ;; Coffee cup
  (setq *coffee-cup* (coffee-cup-cup))
  (send *coffee-cup* :translate #f(4700 100 720) :world)
  ;; microwave
  (setq *microwave* (toshiba-microwave-oven))
  (send *microwave* :rotate -pi/2 :z :local)
  (send *microwave* :translate #f(5400 2300 800) :world)
  ;; Dishes
  (setq *dishes* nil)
  (dotimes (i 10)
    (let ((dish (room73b2-dish)))
      (send dish :translate (float-vector 0 0 (* i 30)) :local)
      (send dish :translate #f(5050 0 720) :world)
      (send dish :name (format nil "dish~A" i))
      (setq *dishes* (append *dishes* (list dish)))))
  ;; Robot Arm object
  (setq *arms* nil)
  (dotimes (i 5)
    (let ((arm (instance gen3_lite_gen3_lite_2f-robot :init)))
      (send arm :translate (float-vector (* i 150) 0 0) :local)
      (send arm :translate #f(4700 -900 720) :world)
      (send arm :name (format nil "arm~A" i))
      (setq *arms* (append *arms* (list arm)))))
  (setq *avs-goal* (make-list 5 :initial-element #f(0 0 0 0 0 0)))

  ;; Show objects on IRT Viewer and Kinematics Simulator
  (dotimes (i 5)
    ;; Remove 610 walls for visibility
    (send *room610* :remove-object
	  (format nil "room610-external-wall-~A" i)))
  (setq *all-objects* (append (send *room610* :objects)
                              (list *closet* *coffee-cup* *microwave*)
                              *dishes* *arms*))
  (objects (append *all-objects* (list *pr2*)))
  (setq *ri* (instance pr2-interface :init :objects *all-objects*))

  ;; Set objects in *ri*
  (setq *ri-tray* (find *tray* (send *ri* :objects)
                        :test #'compare-name))
  (setq *ri-coffee-cup*
        (find *coffee-cup* (send *ri* :objects)
                             :test #'compare-name))
  (setq *ri-dishes* nil)
  (dolist (dish *dishes*)
    (let ((ri-dish (find dish (send *ri* :objects)
                         :test #'compare-name)))
      (setq *ri-dishes* (append *ri-dishes* (list ri-dish)))))
  (setq *ri-arms* nil)
  (dolist (arm *arms*)
    (let ((ri-arm (find arm (send *ri* :objects)
                        :test #'compare-name)))
      (setq *ri-arms* (append *ri-arms* (list ri-arm)))))
  (setq *ri-irtviewer* (*ri* . viewer))

  ;; Define spots
  (setq *arms-spot* (make-cascoords :pos #f(4500 -1500 0) :rpy (list pi/2 0 0)))
  (setq *microwave-spot* (make-coords :pos #f(5700 1500 0) :rpy (list pi/2 0 0)))
  )

(defun compare-name (a b)
  (string= (send a :name) (send b :name)))

(defun connect-dishes-to-tray (command) ;; command is :assoc or :dissoc
  ;; Assoc/Dissoc objects in IRT viewer
  (dolist (dish *dishes*)
    (send *tray* command dish))
  ;; Assoc/Dissoc objects in Kinematics Simulator
  (dolist (dish *ri-dishes*)
    (send *ri-tray* command dish))
  )

(defun move-to-arms ()
  (warn "move-to-arms~%")
  (use-tilt-laser-obstacle-cloud nil)
  (clear-costmap)
  (send *pr2* :move-to *arms-spot* :world)
  (if (not (send *ri* :move-to *arms-spot* :frame-id *room610-origin*))
      (return-from move-to-arms nil))
  (use-tilt-laser-obstacle-cloud t)
  t)

(defun pick-arm (arm-num) ;; arm-num is 0 ~ (len *arms*)
  (let* ((target-arm (elt *arms* arm-num))
         (arm-pose (send target-arm :copy-worldcoords))
         (ri-target-arm (elt *ri-arms* arm-num)))
    ;; Reach
    (send *pr2* :rarm :inverse-kinematics arm-pose :rotation-axis nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ;; Grasp
    (send *ri* :start-grasp :rarm :wait t :objects (list ri-target-arm))
    (send (send *pr2* :rarm :end-coords) :assoc target-arm)))

(defun place-arm (arm-num) ;; arm-num is 0 ~ (len *arms*)
  ;; Place
  (send *pr2* :rarm :move-end-pos #f(-50 200 50) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  ;; Release
  (let ((target-arm (elt *arms* arm-num)))
    (send *ri* :stop-grasp :rarm :wait t)
    (send (send *pr2* :rarm :end-coords) :dissoc target-arm))
  ;; Reset pose
  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun move-to-spot (spot &key (no-wait nil))
  ;; move (No wait because arm modules move during PR2 move)
  (clear-costmap)
  (send *pr2* :move-to spot :world)
  (send *ri* :move-to spot :frame-id *room610-origin* :no-wait no-wait)
  t)

(defun move-arm-to-sink ()
  (move-to-spot *arms-spot*) ;; (move-to-table-side)
  (pick-arm 0)
  (move-to-spot (send *room610* :spot-sink-spot)) ;; (move-to-sink)
  (send *ri* :go-pos 0 0.4 0)
  (place-arm 0)
  )

(defun set-arm-av (arm-num av-target)
  (let ((avs-arm (send-all *arms* :angle-vector)))
    (setq *avs-goal* avs-arm)
    (setf (elt *avs-goal* arm-num) av-target)))

(defmethod robot-move-base-interface
  (:robot-interface-simulation-callback
   ()
   (when current-goal-coords
     (let* ((orig-coords (send robot :copy-worldcoords))
            (diff-pos (send orig-coords :difference-position current-goal-coords))
            (diff-rot (send orig-coords :difference-rotation current-goal-coords :rotation-axis :xy)))
       (cond
         ((and (eps= (norm diff-pos) 0) (eps= (norm diff-rot) 0))
          (setq current-goal-coords nil))
         (t
          (send robot :newcoords (midcoords (min (/ 10 (max (norm diff-pos) 10))
                                                 (/ 0.02 (max (norm diff-rot) 0.02)))
                                            orig-coords current-goal-coords))))
       )) ;; when

   ;; Move arm
   (let* ((avs-orig (send-all *arms* :angle-vector))
          (avs-goal *avs-goal*)
          (avs-diff (mapcar #'v- *avs-goal* avs-orig))
          ;; (avs-diff-round (mapcar #'(lambda (x) (concatenate float-vector ;; Set each elements more than 5
          ;;                                                    (mapcar #'(lambda (y) (max y 5)) (concatenate cons x))))
          ;;                         avs-diff))
          (avs-plus (mapcar #'(lambda (x) (scale 0.05 x)) avs-diff))
          (avs-new (mapcar #'v+ avs-orig avs-plus)))
     (mapcar #'(lambda (x y) (send x :angle-vector y)) *arms* avs-new)
     (mapcar #'(lambda (x y) (send x :angle-vector y)) *ri-arms* avs-new)
     )
   (send-super :robot-interface-simulation-callback)
   ))

;; See (demo) function in setup.l
(defun move-tray-to-sink ()
  (connect-dishes-to-tray :assoc)
  ;; Move to tray
  (set-arm-av 0 #f(50 50 50 50 50 50))
  (move-to-spot (send *room610* :spot-tray-spot)) ;; (move-to-table-side)
  ;; Pick tray
  (while (not (pick-tray-table-side)))
  ;; Move to sink
  (set-arm-av 0 #f(0 0 0 0 0 0))
  (move-to-spot (send *room610* :spot-sink-spot)) ;; (move-to-sink)
  ;; Place tray
  (place-tray-sink)
  ;; Move
  (set-arm-av 0 #f(0 0 0 0 0 0))
  (send *ri* :go-pos-unsafe 0 0 -90)
  (connect-dishes-to-tray :dissoc)
  )

(defun move-cup-to-shelf ()
  ;; Move
  (move-to-spot (send *room610* :spot-tray-spot)) ;; (move-to-table-side)
  ;; Reach
  (send *pr2* :rarm :inverse-kinematics *coffee-cup* :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;; Grasp
  (send *ri* :start-grasp :rarm :wait t :objects (list *ri-coffee-cup*))
  (send (send *pr2* :rarm :end-coords) :assoc *coffee-cup*)
  ;; Move
  (move-to-spot *microwave-spot*)
  ;; Place
  (send *pr2* :rarm :move-end-pos #f(200 200 80) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;; Release
  (send *ri* :stop-grasp :rarm :wait t)
  (send (send *pr2* :rarm :end-coords) :dissoc *coffee-cup*)
  ;; Reset pose
  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun move-arm-to-shelf ()
  (move-to-spot *arms-spot*) ;; (move-to-table-side)
  (pick-arm 1)
  (move-to-spot *microwave-spot*) ;; (move-to-sink)
  (send *ri* :go-pos -0.1 0.25 0)
  (place-arm 1)
  )

(defun demo-dish-wash ()
  (move-tray-to-sink)
  (move-arm-to-sink)
  )

(setup-kinoko)
