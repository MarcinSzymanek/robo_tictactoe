;; Auto-generated. Do not edit!


(when (boundp 'tic_tac_toebot::StartGame)
  (if (not (find-package "TIC_TAC_TOEBOT"))
    (make-package "TIC_TAC_TOEBOT"))
  (shadow 'StartGame (find-package "TIC_TAC_TOEBOT")))
(unless (find-package "TIC_TAC_TOEBOT::STARTGAME")
  (make-package "TIC_TAC_TOEBOT::STARTGAME"))

(in-package "ROS")
;;//! \htmlinclude StartGame.msg.html


(defclass tic_tac_toebot::StartGame
  :super ros::object
  :slots (_start ))

(defmethod tic_tac_toebot::StartGame
  (:init
   (&key
    ((:start __start) nil)
    )
   (send-super :init)
   (setq _start __start)
   self)
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:serialization-length
   ()
   (+
    ;; bool _start
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _start
       (if _start (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _start
     (setq _start (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get tic_tac_toebot::StartGame :md5sum-) "676aa7bfb3ec2071e814f2368dfd5fb5")
(setf (get tic_tac_toebot::StartGame :datatype-) "tic_tac_toebot/StartGame")
(setf (get tic_tac_toebot::StartGame :definition-)
      "bool start # Technically we don't care what this value is, just need a signal to start
")



(provide :tic_tac_toebot/StartGame "676aa7bfb3ec2071e814f2368dfd5fb5")


