;; Auto-generated. Do not edit!


(when (boundp 'tic_tac_toebot::ManualMoveTo)
  (if (not (find-package "TIC_TAC_TOEBOT"))
    (make-package "TIC_TAC_TOEBOT"))
  (shadow 'ManualMoveTo (find-package "TIC_TAC_TOEBOT")))
(unless (find-package "TIC_TAC_TOEBOT::MANUALMOVETO")
  (make-package "TIC_TAC_TOEBOT::MANUALMOVETO"))

(in-package "ROS")
;;//! \htmlinclude ManualMoveTo.msg.html


(defclass tic_tac_toebot::ManualMoveTo
  :super ros::object
  :slots (_x _y _z ))

(defmethod tic_tac_toebot::ManualMoveTo
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tic_tac_toebot::ManualMoveTo :md5sum-) "cc153912f1453b708d221682bc23d9ac")
(setf (get tic_tac_toebot::ManualMoveTo :datatype-) "tic_tac_toebot/ManualMoveTo")
(setf (get tic_tac_toebot::ManualMoveTo :definition-)
      "float32 x # xyz positions of where we want to move the robot in cm 
float32 y
float32 z
")



(provide :tic_tac_toebot/ManualMoveTo "cc153912f1453b708d221682bc23d9ac")


