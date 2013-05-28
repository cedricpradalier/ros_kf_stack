; Auto-generated. Do not edit!


(cl:in-package kf_yaw_kf-srv)


;//! \htmlinclude SetMagOffset-request.msg.html

(cl:defclass <SetMagOffset-request> (roslisp-msg-protocol:ros-message)
  ((mag_x_offset
    :reader mag_x_offset
    :initarg :mag_x_offset
    :type cl:float
    :initform 0.0)
   (mag_y_offset
    :reader mag_y_offset
    :initarg :mag_y_offset
    :type cl:float
    :initform 0.0)
   (mag_z_offset
    :reader mag_z_offset
    :initarg :mag_z_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetMagOffset-request (<SetMagOffset-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMagOffset-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMagOffset-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kf_yaw_kf-srv:<SetMagOffset-request> is deprecated: use kf_yaw_kf-srv:SetMagOffset-request instead.")))

(cl:ensure-generic-function 'mag_x_offset-val :lambda-list '(m))
(cl:defmethod mag_x_offset-val ((m <SetMagOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kf_yaw_kf-srv:mag_x_offset-val is deprecated.  Use kf_yaw_kf-srv:mag_x_offset instead.")
  (mag_x_offset m))

(cl:ensure-generic-function 'mag_y_offset-val :lambda-list '(m))
(cl:defmethod mag_y_offset-val ((m <SetMagOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kf_yaw_kf-srv:mag_y_offset-val is deprecated.  Use kf_yaw_kf-srv:mag_y_offset instead.")
  (mag_y_offset m))

(cl:ensure-generic-function 'mag_z_offset-val :lambda-list '(m))
(cl:defmethod mag_z_offset-val ((m <SetMagOffset-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kf_yaw_kf-srv:mag_z_offset-val is deprecated.  Use kf_yaw_kf-srv:mag_z_offset instead.")
  (mag_z_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMagOffset-request>) ostream)
  "Serializes a message object of type '<SetMagOffset-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mag_x_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mag_y_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mag_z_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMagOffset-request>) istream)
  "Deserializes a message object of type '<SetMagOffset-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag_x_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag_y_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag_z_offset) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMagOffset-request>)))
  "Returns string type for a service object of type '<SetMagOffset-request>"
  "kf_yaw_kf/SetMagOffsetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMagOffset-request)))
  "Returns string type for a service object of type 'SetMagOffset-request"
  "kf_yaw_kf/SetMagOffsetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMagOffset-request>)))
  "Returns md5sum for a message object of type '<SetMagOffset-request>"
  "658dd2586f9c7d77f232884f7526eb51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMagOffset-request)))
  "Returns md5sum for a message object of type 'SetMagOffset-request"
  "658dd2586f9c7d77f232884f7526eb51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMagOffset-request>)))
  "Returns full string definition for message of type '<SetMagOffset-request>"
  (cl:format cl:nil "float32 mag_x_offset~%float32 mag_y_offset~%float32 mag_z_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMagOffset-request)))
  "Returns full string definition for message of type 'SetMagOffset-request"
  (cl:format cl:nil "float32 mag_x_offset~%float32 mag_y_offset~%float32 mag_z_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMagOffset-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMagOffset-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMagOffset-request
    (cl:cons ':mag_x_offset (mag_x_offset msg))
    (cl:cons ':mag_y_offset (mag_y_offset msg))
    (cl:cons ':mag_z_offset (mag_z_offset msg))
))
;//! \htmlinclude SetMagOffset-response.msg.html

(cl:defclass <SetMagOffset-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetMagOffset-response (<SetMagOffset-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMagOffset-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMagOffset-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kf_yaw_kf-srv:<SetMagOffset-response> is deprecated: use kf_yaw_kf-srv:SetMagOffset-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMagOffset-response>) ostream)
  "Serializes a message object of type '<SetMagOffset-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMagOffset-response>) istream)
  "Deserializes a message object of type '<SetMagOffset-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMagOffset-response>)))
  "Returns string type for a service object of type '<SetMagOffset-response>"
  "kf_yaw_kf/SetMagOffsetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMagOffset-response)))
  "Returns string type for a service object of type 'SetMagOffset-response"
  "kf_yaw_kf/SetMagOffsetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMagOffset-response>)))
  "Returns md5sum for a message object of type '<SetMagOffset-response>"
  "658dd2586f9c7d77f232884f7526eb51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMagOffset-response)))
  "Returns md5sum for a message object of type 'SetMagOffset-response"
  "658dd2586f9c7d77f232884f7526eb51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMagOffset-response>)))
  "Returns full string definition for message of type '<SetMagOffset-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMagOffset-response)))
  "Returns full string definition for message of type 'SetMagOffset-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMagOffset-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMagOffset-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMagOffset-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMagOffset)))
  'SetMagOffset-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMagOffset)))
  'SetMagOffset-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMagOffset)))
  "Returns string type for a service object of type '<SetMagOffset>"
  "kf_yaw_kf/SetMagOffset")