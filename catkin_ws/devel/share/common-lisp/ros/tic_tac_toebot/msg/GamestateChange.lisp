; Auto-generated. Do not edit!


(cl:in-package tic_tac_toebot-msg)


;//! \htmlinclude GamestateChange.msg.html

(cl:defclass <GamestateChange> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GamestateChange (<GamestateChange>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GamestateChange>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GamestateChange)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tic_tac_toebot-msg:<GamestateChange> is deprecated: use tic_tac_toebot-msg:GamestateChange instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <GamestateChange>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tic_tac_toebot-msg:start-val is deprecated.  Use tic_tac_toebot-msg:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GamestateChange>) ostream)
  "Serializes a message object of type '<GamestateChange>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GamestateChange>) istream)
  "Deserializes a message object of type '<GamestateChange>"
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GamestateChange>)))
  "Returns string type for a message object of type '<GamestateChange>"
  "tic_tac_toebot/GamestateChange")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GamestateChange)))
  "Returns string type for a message object of type 'GamestateChange"
  "tic_tac_toebot/GamestateChange")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GamestateChange>)))
  "Returns md5sum for a message object of type '<GamestateChange>"
  "676aa7bfb3ec2071e814f2368dfd5fb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GamestateChange)))
  "Returns md5sum for a message object of type 'GamestateChange"
  "676aa7bfb3ec2071e814f2368dfd5fb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GamestateChange>)))
  "Returns full string definition for message of type '<GamestateChange>"
  (cl:format cl:nil "bool start # Technically we don't care what this value is, just need a signal to start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GamestateChange)))
  "Returns full string definition for message of type 'GamestateChange"
  (cl:format cl:nil "bool start # Technically we don't care what this value is, just need a signal to start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GamestateChange>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GamestateChange>))
  "Converts a ROS message object to a list"
  (cl:list 'GamestateChange
    (cl:cons ':start (start msg))
))
