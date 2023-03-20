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
;; (load "package://jsk_2013_04_pr2_610/euslisp/gen3_robotiq_2f_85.l")

(defun setup-kinoko ()
  ;; Additional objects for kinoko IRT
  ;; Closet
  (setq *closet* (square-600-closet))
  (send *closet* :translate #f(1700 2500 0) :world)
  (send *closet* :rotate -pi/2 :z :local)
  ;; Cloth
  (setq *cloth* (cloth))
  (send *cloth* :move-to (send *laundry* :copy-worldcoords) :world)
  (send *cloth* :translate #f(0 -400 800) :world)
  (send *cloth* :rotate pi/2 :y :local)
  (send *cloth* :rotate pi/2 :z :local)
  ;; Hanger
  (setq *hanger* (hanger))
  (send *hanger* :translate #f(4770 -200 720) :world)
  (send *hanger* :rotate -pi/2 :z)
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
      (send dish :translate (float-vector 0 0 (* i -30)) :local)
      (send dish :translate #f(5050 0 990) :world)
      (send dish :name (format nil "dish~A" i))
      (setq *dishes* (append *dishes* (list dish)))))
  (setq *dishes-on-tray* *dishes*)
  (setq *dishes-on-sink* nil)
  ;; Robot Arm object
  (setq *arms* nil)
  (let ((arm-num 4))
    (dotimes (i arm-num)
      (let ((arm (instance gen3_lite_gen3_lite_2f-robot :init)))
        (send arm :translate (float-vector (* i 200) 0 0) :local)
        (send arm :translate #f(4700 -900 720) :world)
        (send arm :name (format nil "arm~A" i))
        (setq *arms* (append *arms* (list arm)))))
    (setq *avs-goal* (make-list arm-num :initial-element #f(0.0 -16.0 75.0 0.0 -60.0 0.0)))
    ;; Robot Arms (only for appearance)
    ;; (setq *arms-85* nil)
    ;; (dotimes (i arm-num)
    ;;   (let ((arm (instance gen3_robotiq_2f_85-robot :init)))
    ;;     (send arm :translate (float-vector (* i 200) 0 0) :local)
    ;;     (send arm :translate #f(5100 -900 720) :world)
    ;;     (send arm :name (format nil "arm-85~A" i))
    ;;     (setq *arms-85* (append *arms-85* (list arm)))))
    )

  ;; Setup normal IRT environment
  (setup)

  ;; Show objects on IRT Viewer and Kinematics Simulator
  (dotimes (i 5)
    ;; Remove 610 walls for visibility
    (send *room610* :remove-object
          (format nil "room610-external-wall-~A" i)))
  (setq *all-objects* (append (send *room610* :objects)
                              (list *closet* *cloth* *hanger* *coffee-cup* *microwave*)
                              *dishes* *arms*))
  (objects (append *all-objects* (list *pr2*)))
  (setq *ri* (instance pr2-interface :init :objects *all-objects*))

  ;; Set objects in *ri*
  (setq *ri-tray* (find *tray* (send *ri* :objects)
                        :test #'compare-name))
  (setq *ri-coffee-cup*
        (find *coffee-cup* (send *ri* :objects)
              :test #'compare-name))
  (setq *ri-microwave*
        (find *microwave* (send *ri* :objects)
              :test #'compare-name))
  (setq *ri-laundry*
        (find *laundry* (send *ri* :objects)
              :test #'compare-name))
  (setq *ri-closet*
        (find *closet* (send *ri* :objects)
              :test #'compare-name))
  (setq *ri-cloth*
        (find *cloth* (send *ri* :objects)
              :test #'compare-name))
  (setq *ri-hanger*
        (find *hanger* (send *ri* :objects)
              :test #'compare-name))
  (setq *ri-dishes* nil)
  (dolist (dish *dishes*)
    (let ((ri-dish (find dish (send *ri* :objects)
                         :test #'compare-name)))
      (setq *ri-dishes* (append *ri-dishes* (list ri-dish)))))
  (setq *ri-dishes-on-tray* *ri-dishes*)
  (setq *ri-dishes-on-sink* nil)
  (setq *ri-arms* nil)
  (dolist (arm *arms*)
    (let ((ri-arm (find arm (send *ri* :objects)
                        :test #'compare-name)))
      (setq *ri-arms* (append *ri-arms* (list ri-arm)))))
  ;; (dolist (arm *arms-85*)
  ;;   (let ((ri-arm (find arm (send *ri* :objects)
  ;;                       :test #'compare-name)))
  ;;     (setq *ri-arms-85* (append *ri-arms-85* (list ri-arm)))))
  (setq *ri-irtviewer* (*ri* . viewer))

  ;; Define spots
  (setq *arms-spot* (make-cascoords :pos #f(4500 -1500 0) :rpy (list pi/2 0 0)))
  (setq *microwave-spot* (make-coords :pos #f(5700 1500 0) :rpy (list pi/2 0 0)))
  )

(defun cloth ()
  (let ((sleeve-1 (make-cube 180 150 3))
        (sleeve-2 (make-cube 180 150 3))
        (cloth-body (make-cube 400 700 3)))
    (send sleeve-1 :translate #f(-220 230 0) :local)
    (send sleeve-1 :rotate (/ pi 4) :z :local)
    (send sleeve-2 :translate #f(220 230 0) :local)
    (send sleeve-2 :rotate (/ -pi 4) :z :local)
    (send cloth-body :assoc sleeve-1)
    (send cloth-body :assoc sleeve-2)
    (instance bodyset :init (make-cascoords)
              :bodies (list cloth-body sleeve-1 sleeve-2)
              :name "cloth")))

;; Do not call twice (I dont know the reason, but error occurs)
(defun hanger ()
  (let* ((hanger-body+ (make-prism '(#f(0 0 0) #f(-20 0 0) #f(-40 -20 0) #f(-280 -140 0)
                                       #f(-280 -150 0) #f(280 -150 0) #f(280 -140 0)
                                       #f(40 -20 0) #f(20 0 0))
                                   #f(0 0 20)))
         (hanger-body- (send (make-prism '(#f(0 0 0) #f(-240 -120 0) #f(240 -120 0)) #f(0 0 20))
                             :translate #f(0 -20 0) :local))
         (hanger-body (body- hanger-body+ hanger-body-))
         (hanger-neck (make-prism '(#f(-10 0 0) #f(-10 20 0) #f(40 60 0) #f(0 100 0)
                                      #f(-40 60 0) #f(-50 70 0)
                                      #f(0 110 0) #f(50 60 0) #f(10 20 0) #f(10 0 0))
                                  #f(0 0 20))))
    (send hanger-body :set-color :white)
    (send hanger-neck :set-color :white)
    (send hanger-body :assoc hanger-neck)
    (instance bodyset :init
              (make-cascoords)
              :bodies (list hanger-body hanger-neck)
              :name "hanger")))

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
  (move-to-spot *arms-spot*)
  (use-tilt-laser-obstacle-cloud t)
  t)

(defun pick-arm (arm-num) ;; arm-num is 0 ~ (len *arms*)
  (let* ((target-arm (elt *arms* arm-num))
         (arm-pose (send target-arm :copy-worldcoords))
         (ri-target-arm (elt *ri-arms* arm-num)))
    ;; Reach
    (send *pr2* :rarm :inverse-kinematics arm-pose :rotation-axis nil)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    ;; Grasp
    (send *ri* :start-grasp :rarm :wait t :objects (list ri-target-arm))
    (send (send *pr2* :rarm :end-coords) :assoc target-arm)))

(defun place-arm (arm-num) ;; arm-num is 0 ~ (len *arms*)
  ;; Place
  (send *pr2* :rarm :move-end-pos #f(-50 200 50) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
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

(defun move-to-pos (x y d) ;; x[m], y[m], d[rad]
  (let ((coords (make-coords :pos (float-vector (* x 1000) (* y 1000) 0)
                             :rpy (float-vector d 0 0))))
    (send *pr2* :move-to coords :local)
    (send *ri* :move-to (send *pr2* :copy-worldcoords))))

(defun move-arm-to-sink ()
  (move-to-spot *arms-spot*) ;; (move-to-table-side)
  (pick-arm 0)
  (move-to-spot (send *room610* :spot-sink-spot)) ;; (move-to-sink)
  (move-to-pos 0 0.4 0)
  (place-arm 0)
  )

(defun set-arm-av (arm-num av-target)
  (let ((avs-arm (send-all *ri-arms* :angle-vector)))
    (setq *avs-goal* avs-arm)
    (setf (elt *avs-goal* arm-num) av-target)))

(defun pile-dish-1 ()
  (let* ((arm-index 0)
         (target-arm (elt *arms* arm-index))
         (target-coords
          (send (send (car *dishes-on-tray*) :copy-worldcoords)
                :rotate pi/2 :y)))
    (set-arm-av arm-index
                (send target-arm :rarm :inverse-kinematics
                      target-coords :rotation-axis :x))
    ))

(defun pile-dish-2 ()
  (let* ((arm-index 0)
         (target-arm (elt *arms* arm-index))
         (ri-target-arm (elt *ri-arms* arm-index)))
    (send (send target-arm :rarm :end-coords) :assoc (car *dishes-on-tray*))
    (send (send ri-target-arm :rarm :end-coords) :assoc (car *ri-dishes-on-tray*))
    (set-arm-av arm-index
                (send target-arm :rarm
                      :move-end-pos (v+ #f(0 400 -400)
                                        (scale (* 2 (length *dishes-on-sink*)) #f(0 0 30)))
                      :world :rotation-axis :x))))

(defun pile-dish-3 ()
  (let* ((arm-index 0)
         (target-arm (elt *arms* arm-index))
         (ri-target-arm (elt *ri-arms* arm-index)))
    (send (send target-arm :rarm :end-coords) :dissoc (car *dishes-on-tray*))
    (send (send ri-target-arm :rarm :end-coords) :dissoc (car *ri-dishes-on-tray*)))
  (setq *dishes-on-sink* (append (list (car *dishes-on-tray*)) *dishes-on-sink*))
  (setq *dishes-on-tray* (cdr *dishes-on-tray*))
  (setq *ri-dishes-on-sink* (append (list (car *ri-dishes-on-tray*)) *ri-dishes-on-sink*))
  (setq *ri-dishes-on-tray* (cdr *ri-dishes-on-tray*)))

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
   (let* ((avs-orig (send-all *ri-arms* :angle-vector))
          (avs-diff (mapcar #'v- *avs-goal* avs-orig))
          ;; (avs-diff-round (mapcar #'(lambda (x) (concatenate float-vector ;; Set each elements more than 5
          ;;                                                    (mapcar #'(lambda (y) (max y 5)) (copncatenate cons x))))
          ;;                         avs-diff))
          (avs-diff-round (mapcar #'round-av avs-diff))
          (avs-plus (mapcar #'(lambda (x) (scale 0.05 x)) avs-diff-round))
          (avs-new (mapcar #'v+ avs-orig avs-plus)))
     ;; Do not update IRT Viewer
     ;; (mapcar #'(lambda (x y) (send x :angle-vector y)) *arms* avs-new)
     (mapcar #'(lambda (x y) (send x :angle-vector y)) *ri-arms* avs-new)
     )
   (send-super :robot-interface-simulation-callback)
   ))

(defun round-av (av)
  ;; If |y| < 0.1, y is 0. Else, |y| >= 1.0.
  (concatenate float-vector
               (mapcar #'(lambda (y)
                           (if (< y 0)
                             (if (> y -0.1) 0 (min y -1))
                             (if (< y 0.1) 0 (max y 1))))
                       (concatenate cons av))))

;; See (demo) function in setup.l
(defun move-tray-to-sink ()
  (connect-dishes-to-tray :assoc)
  ;; Move to tray
  (move-to-spot (send *room610* :spot-tray-spot)) ;; (move-to-table-side)
  ;; Pick tray
  (while (not (pick-tray-table-side)))
  ;; Move to sink
  (move-to-spot (send *room610* :spot-sink-spot)) ;; (move-to-sink)
  ;; Place tray
  (place-tray-sink)
  ;; Move
  (move-to-pos 0 0 -pi/2)
  (connect-dishes-to-tray :dissoc)
  )

(defun move-cup-to-shelf (&optional (parallel-arm-move t))
  ;; Move
  (when parallel-arm-move (pile-dish-1))
  (move-to-pos -1.0 0.3 0)
  (when parallel-arm-move (pile-dish-2))
  (move-to-spot (send *room610* :spot-tray-spot)) ;; (move-to-table-side)
  ;; Reach
  (when parallel-arm-move (pile-dish-3))
  (send *pr2* :rarm :inverse-kinematics *coffee-cup* :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;; Grasp
  (send *ri* :start-grasp :rarm :wait t :objects (list *ri-coffee-cup*))
  (send (send *pr2* :rarm :end-coords) :assoc *coffee-cup*)
  ;; Move
  (when parallel-arm-move (pile-dish-1))
  (move-to-spot *microwave-spot*)
  ;; Place
  (when parallel-arm-move (pile-dish-2))
  (send *pr2* :rarm :move-end-pos #f(200 200 80) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;; Release
  (send *ri* :stop-grasp :rarm :wait t)
  (send (send *pr2* :rarm :end-coords) :dissoc *coffee-cup*)
  ;; Reset pose
  (when parallel-arm-move (pile-dish-3))
  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun move-arm-to-shelf (&optional (parallel-arm-move t))
  ;; Move (move-to-table-side)
  (when parallel-arm-move (pile-dish-1))
  (move-to-pos -0.5 1.0 0)
  (when parallel-arm-move (pile-dish-2))
  (move-to-pos -0.5 0.5 0)
  (when parallel-arm-move (pile-dish-3))
  (move-to-spot *arms-spot*)
  ;; Pick arm
  (when parallel-arm-move (pile-dish-1))
  (pick-arm 1)
  ;; Move (move-to-sink)
  (when parallel-arm-move (pile-dish-2))
  (move-to-pos 0.5 1.0 0)
  (when parallel-arm-move (pile-dish-3))
  (move-to-pos 0.5 0.0 0)
  (when parallel-arm-move (pile-dish-1))
  (move-to-spot *microwave-spot*)
  ;; Place arm
  (when parallel-arm-move (pile-dish-2))
  (move-to-pos -0.1 0.2 0)
  (place-arm 1)
  (when parallel-arm-move (pile-dish-3))
  )

;; Before heating cup
(defun use-microwave-1 () ;; Reach microwave handle
  (let ((target-coords
         (send (send *microwave* :copy-worldcoords) :translate #f(40 -200 340) :world)))
    (send (elt *arms* 1) :angle-vector #f(-124.202 -7.57043 109.933 -148.396 -34.9508 0))
    (set-arm-av 1
                (send (elt *arms* 1) :rarm :inverse-kinematics target-coords :rotation-axis nil))))

(defun use-microwave-2 () ;; Open microwave
  ;; (set-arm-av 1 (send (elt *arms* 1) :angle-vector #f(-121.669 -46.6893 91.55 -145.421 -18.7229 0)))
  (let ((target-coords
         (send (send *microwave* :copy-worldcoords) :translate #f(40 -480 60) :world)))
    (send (elt *arms* 1) :angle-vector #f(-124.202 -7.57043 109.933 -148.396 -34.9508 0))
    (set-arm-av 1
                (send (elt *arms* 1) :rarm :inverse-kinematics target-coords :rotation-axis nil)))
  (send *microwave* :angle-vector #f(90 0))
  (send *ri-microwave* :angle-vector #f(90 0)))

;; From: #<coffee-cup-cup-object #X55ba5bea6070 coffee-cup-cup  6099.989 2199.981 799.997 / 3.142 4.514e-07 -1.995e-06>
(defun use-microwave-3 () ;; Reach coffee cup
  (let ((target-coords (send *coffee-cup* :copy-worldcoords)))
    (set-arm-av 1
                (send (elt *arms* 1) :rarm :inverse-kinematics target-coords :rotation-axis nil))))

(defun use-microwave-4 () ;; Move coffee cup to microwave
  (send (send (elt *arms* 1) :rarm :end-coords) :assoc *coffee-cup*)
  (send (send (elt *ri-arms* 1) :rarm :end-coords) :assoc *ri-coffee-cup*)
  (let ((target-coords
         (send (send *microwave* :copy-worldcoords) :translate #f(0 -100 100) :world)))
    (send (elt *arms* 1) :angle-vector #f(-124.202 -7.57043 109.933 -148.396 -34.9508 0))
    (set-arm-av 1
                (send (elt *arms* 1) :rarm :inverse-kinematics target-coords :rotation-axis nil))))

(defun use-microwave-5 () ;; Reach microwave handle
  (send (send (elt *arms* 1) :rarm :end-coords) :dissoc *coffee-cup*)
  (send (send (elt *ri-arms* 1) :rarm :end-coords) :dissoc *ri-coffee-cup*)
  (use-microwave-2))

(defun use-microwave-6 () ;; Close microwave
  (use-microwave-1)
  (send *microwave* :angle-vector #f(0 0))
  (send *ri-microwave* :angle-vector #f(0 0)))

;; After heating cup
(defun use-microwave-7 () ;; Reach microwave handle
  (use-microwave-1))

(defun use-microwave-8 () ;; Open microwave
  (use-microwave-2))

(defun use-microwave-9 () ;; Reach coffee cup
  (use-microwave-3))

(defun use-microwave-10 () ;; Move coffee cup ouside microwave
  (send (send (elt *arms* 1) :rarm :end-coords) :assoc *coffee-cup*)
  (send (send (elt *ri-arms* 1) :rarm :end-coords) :assoc *ri-coffee-cup*)
  (let ((target-coords
         (send (send *coffee-cup* :copy-worldcoords) :translate #f(200 -300 100) :world)))
    (send (elt *arms* 1) :angle-vector #f(-124.202 -7.57043 109.933 -148.396 -34.9508 0))
    (set-arm-av 1
                (send (elt *arms* 1) :rarm :inverse-kinematics target-coords :rotation-axis nil))))


(defun pick-laundry-cloth (&optional (parallel-arm-move t))
  ;; Move
  (when parallel-arm-move (use-microwave-1))
  (move-to-spot *laundry-spot*)
  ;; Open laundry
  (when parallel-arm-move (use-microwave-2))
  (send *ri* :angle-vector
        (send *pr2* :larm :inverse-kinematics
              (send (car (send *laundry* :handle)) :copy-worldcoords) :rotation-axis nil)
        3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector
        (send *pr2* :larm :move-end-pos #f(-200 -200 0) :world
              :rotation-axis nil)
        3000)
  (send *ri* :wait-interpolation)
  (send *laundry* :angle-vector #f(90 0))
  (send *ri-laundry* :angle-vector #f(90 0))
  ;; Pick cloth
  (when parallel-arm-move (use-microwave-3))
  (send *pr2* :rarm :inverse-kinematics
        (send (send *cloth* :copy-worldcoords) :rotate pi/2 :y :local))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send (send *pr2* :rarm :end-coords) :assoc *cloth*)
  (send *ri* :start-grasp :rarm :objects (list *ri-cloth*))
  (send *pr2* :rarm :angle-vector
        #f(-75.0 -20.0 -70.0 -120.0 -20.0 -30.0 180.0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;; Close laundry
  (when parallel-arm-move (use-microwave-4))
  (send *ri* :angle-vector
        (send *pr2* :larm :inverse-kinematics
              (send (car (send *laundry* :handle)) :copy-worldcoords) :rotation-axis nil)
        3000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector
        (send *pr2* :larm :move-end-pos #f(-200 -200 0) :world
              :rotation-axis nil)
        3000)
  (send *ri* :wait-interpolation)
  (send *laundry* :angle-vector #f(0 0))
  (send *ri-laundry* :angle-vector #f(0 0))
  (when parallel-arm-move (use-microwave-5))
  ;; Reset pose
  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  ;; Move to arms and arm module picks cloth
  (when parallel-arm-move (use-microwave-6))
  (move-to-spot *arms-spot*)
  (move-to-pos -0.3 -0.4 0)
  (send *pr2* :rarm :angle-vector
        #f(-68.0085 8.7191 -87.2775 -95.8579 -181.564 -27.5931 -7.716))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (let ((target-coords-2
         (send (send *cloth* :copy-worldcoords) :translate #f(-200 0 300) :world))
        (target-coords-3
         (send (send *cloth* :copy-worldcoords) :translate #f(200 0 300) :world)))
    (let ((avs-arm (send-all *ri-arms* :angle-vector)))
      (setq *avs-goal* avs-arm)
      (setf (elt *avs-goal* 2)
            (send (elt *arms* 2) :rarm :inverse-kinematics target-coords-2 :rotation-axis nil))
      (setf (elt *avs-goal* 3)
            (send (elt *arms* 3) :rarm :inverse-kinematics target-coords-3 :rotation-axis nil))))
  (send (send *pr2* :rarm :end-coords) :dissoc *cloth*)
  (send *ri* :stop-grasp :rarm)
  ;; Pick hanger
  (move-to-pos 2.0 0.8 -pi/2)
  (let ((av-orig (send *pr2* :angle-vector)))
    (send *pr2* :rarm :inverse-kinematics
          (send (send *hanger* :copy-worldcoords) :rotate pi/2 :y))
    (send *ri* :angle-vector
          (send *pr2* :angle-vector)
          3000)
    (send *ri* :wait-interpolation)
    (send (send *pr2* :rarm :end-coords) :assoc *hanger*)
    (send *ri* :start-grasp :rarm :objects (list *ri-hanger*)) ;; ayasii
    (send *ri* :angle-vector (send *pr2* :angle-vector av-orig))
    (send *ri* :wait-interpolation))
  ;; Hang clothes
  (move-to-spot *arms-spot*)
  (move-to-pos -0.3 -0.5 0)
  (let ((avs-arm (send-all *ri-arms* :angle-vector)))
    (setq *avs-goal* avs-arm)
    (setf (elt *avs-goal* 2)
          (send (elt *arms* 2) :rarm :move-end-pos #f(0 0 100) :world :rotation-axis nil))
    (setf (elt *avs-goal* 3)
          (send (elt *arms* 3) :rarm :move-end-pos #f(0 0 100) :world :rotation-axis nil)))
  (send *pr2* :rarm :move-end-pos #f(0 0 -200) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (let ((avs-arm (send-all *ri-arms* :angle-vector)))
    (setq *avs-goal* avs-arm)
    (setf (elt *avs-goal* 2)
          (send (elt *arms* 2) :rarm :move-end-pos #f(0 0 -100) :world :rotation-axis nil))
    (setf (elt *avs-goal* 3)
          (send (elt *arms* 3) :rarm :move-end-pos #f(0 0 -100) :world :rotation-axis nil)))
  (let ((target-coords (send *cloth* :copy-worldcoords)))
    (send target-coords :translate #f(0 0 400) :world)
    (send target-coords :rotate pi/2 :y)
    (send *pr2* :rarm :inverse-kinematics target-coords))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (send *hanger* :assoc *cloth*)
  (send *ri-hanger* :assoc *ri-cloth*)
  ;; Move to closet
  (let ((target-spot (send *closet* :copy-worldcoords)))
    (send target-spot :translate #f(0 -900 0) :world)
    (send target-spot :rotate pi :z)
    (move-to-spot target-spot))
  ;; Open closet
  (send *pr2* :larm :inverse-kinematics
        (send (car (send *closet* :handle)) :copy-worldcoords)
        :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (when parallel-arm-move (use-microwave-7))
  (send *pr2* :larm :move-end-pos #f(300 -300 0) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *closet* :angle-vector #f(90.0 90.0 0.0 0.0 0.0 0.0))
  (send *ri-closet* :angle-vector #f(90.0 90.0 0.0 0.0 0.0 0.0))
  ;; Set clothes
  (when parallel-arm-move (use-microwave-8))
  (send *pr2* :rarm :inverse-kinematics
        (send (send *closet* :copy-worldcoords) :translate #f(0 0 1000) :world)
        :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send (send *pr2* :rarm :end-coords) :dissoc *hanger*)
  (send *ri* :stop-grasp :rarm)
  (when parallel-arm-move (use-microwave-9))
  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;; Closet closet
  (when parallel-arm-move (use-microwave-10))
  (send *pr2* :larm :move-end-pos #f(-300 300 0) :world :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *closet* :angle-vector #f(0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri-closet* :angle-vector #f(0.0 0.0 0.0 0.0 0.0 0.0))
  )

(defun pick-cup-from-microwave ()
  (move-to-spot (send (send *microwave-spot* :copy-worldcoords) :translate #f(-300 -400 0) :world))
  (send *pr2* :rarm :inverse-kinematics
        (send *coffee-cup* :copy-worldcoords) :rotation-axis nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send (send *pr2* :rarm :end-coords) :assoc *coffee-cup*)
  (send *ri* :start-grasp :rarm :objects (list *ri-coffee-cup*) :wait t)
  (send (send (*ri* . robot) :rarm :end-coords) :assoc *ri-coffee-cup*)
  (send *pr2* :reset-pose)
  (pr2-pick-tray-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;;
  (send (send *pr2* :rarm :end-coords) :dissoc *coffee-cup*)
  (send *ri* :stop-grasp :rarm)
  )

(defun demo-dish-wash ()
  ;; Demo1
  (move-tray-to-sink)
  (move-arm-to-sink)
  ;; Demo2
  ;; If you use the following demo without arm module move, call with argument nil
  (move-cup-to-shelf t)
  (move-arm-to-shelf t)
  ;; Demo3
  (pick-laundry-cloth t)
  (pick-cup-from-microwave)
  )

(setup-kinoko)
;; (demo-dish-wash)
