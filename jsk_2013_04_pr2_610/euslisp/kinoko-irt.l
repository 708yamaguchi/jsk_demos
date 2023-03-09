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
  (send *coffee-cup* :translate #f(5050 -300 720) :world)
  ;; microwave
  (setq *microwave* (toshiba-microwave-oven))
  (send *microwave* :rotate -pi/2 :z :local)
  (send *microwave* :translate #f(5400 2300 800) :world)
  ;; Dishes
  (setq *dishes* nil)
  (dotimes (i 10)
    (setq *dishes*
          (append *dishes*
                  (list (send (room73b2-dish)
                              :translate (float-vector 0 0 (* i 30)) :local)))))
  (dolist (dish *dishes*)
    (send dish :translate #f(5050 0 720) :world))

  ;; Robot Arm object
  (setq *arms* nil)
  (dotimes (i 5)
    (setq *arms*
          (append *arms*
                  (list (send (instance gen3_lite_gen3_lite_2f-robot :init)
                              :translate (float-vector (* i 150) 0 0) :local)))))
  (dolist (arm *arms*)
    (send arm :translate #f(4700 -700 720) :world))

  ;; Show objects on IRT Viewer and Kinematics Simulator
  (setq *all-objects* (append (list *room610* *pr2* *closet* *coffee-cup* *microwave*) *dishes* *arms*))
  (objects *all-objects*)
  (setq *ri* (instance pr2-interface :init :objects *all-objects*))
  )

(setup-kinoko)
