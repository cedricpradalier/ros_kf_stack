; Auto-generated. Do not edit!


(cl:in-package kf_yaw_kf-msg)


;//! \htmlinclude Compass.msg.html

(cl:defclass <Compass> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (compass
    :reader compass
    :initarg :compass
    :type cl:float
    :initform 0.0)
   (heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0)
   (stddev
    :reader stddev
    :initarg :stddev
    :type cl:float
    :initform 0.0))
)

(cl:defclass Compass (<Compass>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Compass>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Compass)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kf_yaw_kf-msg:<Compass> is deprecated: use kf_yaw_kf-msg:Compass instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kf_yaw_kf-msg:header-val is deprecated.  Use kf_yaw_kf-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'compass-val :lambda-list '(m))
(cl:defmethod compass-val ((m <Compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kf_yaw_kf-msg:compass-val is deprecated.  Use kf_yaw_kf-msg:compass instead.")
  (compass m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <Compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kf_yaw_kf-msg:heading-val is deprecated.  Use kf_yaw_kf-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'stddev-val :lambda-list '(m))
(cl:defmethod stddev-val ((m <Compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kf_yaw_kf-msg:stddev-val is deprecated.  Use kf_yaw_kf-msg:stddev instead.")
  (stddev m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Compass>) ostream)
  "Serializes a message object of type '<Compass>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'compass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'stddev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Compass>) istream)
  "Deserializes a message object of type '<Compass>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'compass) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stddev) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Compass>)))
  "Returns string type for a message object of type '<Compass>"
  "kf_yaw_kf/Compass")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Compass)))
  "Returns string type for a message object of type 'Compass"
  "kf_yaw_kf/Compass")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Compass>)))
  "Returns md5sum for a message object of type '<Compass>"
  "6128f9329b98d64881b65764b2d6fdbe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Compass)))
  "Returns md5sum for a message object of type 'Compass"
  "6128f9329b98d64881b65764b2d6fdbe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Compass>)))
  "Returns full string definition for message of type '<Compass>"
  (cl:format cl:nil "Header header~%float32 compass~%float32 heading~%float32 stddev~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Compass)))
  "Returns full string definition for message of type 'Compass"
  (cl:format cl:nil "Header header~%float32 compass~%float32 heading~%float32 stddev~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Compass>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Compass>))
  "Converts a ROS message object to a list"
  (cl:list 'Compass
    (cl:cons ':header (header msg))
    (cl:cons ':compass (compass msg))
    (cl:cons ':heading (heading msg))
    (cl:cons ':stddev (stddev msg))
))
