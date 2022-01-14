; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude Speed_tick.msg.html

(cl:defclass <Speed_tick> (roslisp-msg-protocol:ros-message)
  ((right_speed
    :reader right_speed
    :initarg :right_speed
    :type cl:float
    :initform 0.0)
   (left_speed
    :reader left_speed
    :initarg :left_speed
    :type cl:float
    :initform 0.0)
   (right_tick
    :reader right_tick
    :initarg :right_tick
    :type cl:integer
    :initform 0)
   (left_tick
    :reader left_tick
    :initarg :left_tick
    :type cl:integer
    :initform 0))
)

(cl:defclass Speed_tick (<Speed_tick>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Speed_tick>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Speed_tick)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<Speed_tick> is deprecated: use custom_msgs-msg:Speed_tick instead.")))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <Speed_tick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:right_speed-val is deprecated.  Use custom_msgs-msg:right_speed instead.")
  (right_speed m))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <Speed_tick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:left_speed-val is deprecated.  Use custom_msgs-msg:left_speed instead.")
  (left_speed m))

(cl:ensure-generic-function 'right_tick-val :lambda-list '(m))
(cl:defmethod right_tick-val ((m <Speed_tick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:right_tick-val is deprecated.  Use custom_msgs-msg:right_tick instead.")
  (right_tick m))

(cl:ensure-generic-function 'left_tick-val :lambda-list '(m))
(cl:defmethod left_tick-val ((m <Speed_tick>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:left_tick-val is deprecated.  Use custom_msgs-msg:left_tick instead.")
  (left_tick m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Speed_tick>) ostream)
  "Serializes a message object of type '<Speed_tick>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'right_tick)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'left_tick)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Speed_tick>) istream)
  "Deserializes a message object of type '<Speed_tick>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_tick) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_tick) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Speed_tick>)))
  "Returns string type for a message object of type '<Speed_tick>"
  "custom_msgs/Speed_tick")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Speed_tick)))
  "Returns string type for a message object of type 'Speed_tick"
  "custom_msgs/Speed_tick")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Speed_tick>)))
  "Returns md5sum for a message object of type '<Speed_tick>"
  "49db339e4cdd08bb4996af3a92cbcc3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Speed_tick)))
  "Returns md5sum for a message object of type 'Speed_tick"
  "49db339e4cdd08bb4996af3a92cbcc3d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Speed_tick>)))
  "Returns full string definition for message of type '<Speed_tick>"
  (cl:format cl:nil "# This represents the right and left speed plus the right and left tick~%~%float64 right_speed~%float64 left_speed~%int32 right_tick~%int32 left_tick~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Speed_tick)))
  "Returns full string definition for message of type 'Speed_tick"
  (cl:format cl:nil "# This represents the right and left speed plus the right and left tick~%~%float64 right_speed~%float64 left_speed~%int32 right_tick~%int32 left_tick~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Speed_tick>))
  (cl:+ 0
     8
     8
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Speed_tick>))
  "Converts a ROS message object to a list"
  (cl:list 'Speed_tick
    (cl:cons ':right_speed (right_speed msg))
    (cl:cons ':left_speed (left_speed msg))
    (cl:cons ':right_tick (right_tick msg))
    (cl:cons ':left_tick (left_tick msg))
))
