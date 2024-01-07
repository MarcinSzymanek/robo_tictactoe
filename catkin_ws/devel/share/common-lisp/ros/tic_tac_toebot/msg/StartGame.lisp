; Auto-generated. Do not edit!


(cl:in-package tic_tac_toebot-msg)


;//! \htmlinclude StartGame.msg.html

(cl:defclass <StartGame> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StartGame (<StartGame>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartGame>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartGame)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tic_tac_toebot-msg:<StartGame> is deprecated: use tic_tac_toebot-msg:StartGame instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <StartGame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tic_tac_toebot-msg:start-val is deprecated.  Use tic_tac_toebot-msg:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartGame>) ostream)
  "Serializes a message object of type '<StartGame>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartGame>) istream)
  "Deserializes a message object of type '<StartGame>"
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartGame>)))
  "Returns string type for a message object of type '<StartGame>"
  "tic_tac_toebot/StartGame")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartGame)))
  "Returns string type for a message object of type 'StartGame"
  "tic_tac_toebot/StartGame")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartGame>)))
  "Returns md5sum for a message object of type '<StartGame>"
  "676aa7bfb3ec2071e814f2368dfd5fb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartGame)))
  "Returns md5sum for a message object of type 'StartGame"
  "676aa7bfb3ec2071e814f2368dfd5fb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartGame>)))
  "Returns full string definition for message of type '<StartGame>"
  (cl:format cl:nil "bool start # Technically we don't care what this value is, just need a signal to start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartGame)))
  "Returns full string definition for message of type 'StartGame"
  (cl:format cl:nil "bool start # Technically we don't care what this value is, just need a signal to start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartGame>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartGame>))
  "Converts a ROS message object to a list"
  (cl:list 'StartGame
    (cl:cons ':start (start msg))
))
