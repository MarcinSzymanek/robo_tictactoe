; Auto-generated. Do not edit!


(cl:in-package tic_tac_toebot-msg)


;//! \htmlinclude ManualMoveTo.msg.html

(cl:defclass <ManualMoveTo> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass ManualMoveTo (<ManualMoveTo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ManualMoveTo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ManualMoveTo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tic_tac_toebot-msg:<ManualMoveTo> is deprecated: use tic_tac_toebot-msg:ManualMoveTo instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <ManualMoveTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tic_tac_toebot-msg:x-val is deprecated.  Use tic_tac_toebot-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <ManualMoveTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tic_tac_toebot-msg:y-val is deprecated.  Use tic_tac_toebot-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <ManualMoveTo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tic_tac_toebot-msg:z-val is deprecated.  Use tic_tac_toebot-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ManualMoveTo>) ostream)
  "Serializes a message object of type '<ManualMoveTo>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ManualMoveTo>) istream)
  "Deserializes a message object of type '<ManualMoveTo>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ManualMoveTo>)))
  "Returns string type for a message object of type '<ManualMoveTo>"
  "tic_tac_toebot/ManualMoveTo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ManualMoveTo)))
  "Returns string type for a message object of type 'ManualMoveTo"
  "tic_tac_toebot/ManualMoveTo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ManualMoveTo>)))
  "Returns md5sum for a message object of type '<ManualMoveTo>"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ManualMoveTo)))
  "Returns md5sum for a message object of type 'ManualMoveTo"
  "cc153912f1453b708d221682bc23d9ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ManualMoveTo>)))
  "Returns full string definition for message of type '<ManualMoveTo>"
  (cl:format cl:nil "float32 x # xyz positions of where we want to move the robot in cm ~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ManualMoveTo)))
  "Returns full string definition for message of type 'ManualMoveTo"
  (cl:format cl:nil "float32 x # xyz positions of where we want to move the robot in cm ~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ManualMoveTo>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ManualMoveTo>))
  "Converts a ROS message object to a list"
  (cl:list 'ManualMoveTo
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
