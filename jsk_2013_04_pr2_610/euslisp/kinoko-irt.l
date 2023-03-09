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
    (let ((dish (room73b2-dish)))
      (send dish :translate (float-vector 0 0 (* i 30)) :local)
      (send dish :translate #f(5050 0 720) :world)
      (send dish :name (format nil "dish~A" i))
      (setq *dishes* (append *dishes* (list dish)))))
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
  (setq *all-objects* (append (send *room610* :objects)
			      (list *closet* *coffee-cup* *microwave*)
			      *dishes* *arms*))
  (objects (append *all-objects* (list *pr2*)))
  (setq *ri* (instance pr2-interface :init :objects *all-objects*))

  ;; Define spots
  (setq *arms-spot* (make-cascoords :pos #f(4500 -1500 0) :rpy (list pi/2 0 0)))
  )

(defun connect-dishes-to-tray (command) ;; command is :assoc or :dissoc
  (let ((ri-tray (find *tray* (send *ri* :objects)
		       :test #'(lambda (a b) (string= (send a :name) (send b :name))))))
    (print ri-tray)
    ;; Assoc/Dissoc objects in IRT viewer
    (dolist (dish *dishes*)
      (send *tray* command dish))
    ;; Assoc/Dissoc objects in Kinematics Simulator
    (dolist (dish *dishes*)
      (send ri-tray command
	    (find dish (send *ri* :objects)
		  :test #'(lambda (a b) (string= (send a :name) (send b :name))))))
    ))

;; See (demo) function in setup.l
(defun demo-tray-kinoko ()
  (connect-dishes-to-tray :assoc)
  (while (not (move-to-table-side)))
  (while (not (pick-tray-table-side)))
  (while (not (move-to-sink)))
  (place-tray-sink)
  (send *ri* :go-pos-unsafe 0 0 -90)
  (connect-dishes-to-tray :dissoc)
  )

(setup-kinoko)
