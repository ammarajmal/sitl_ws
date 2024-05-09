; Auto-generated. Do not edit!


(cl:in-package fast_cam-srv)


;//! \htmlinclude SetGain-request.msg.html

(cl:defclass <SetGain-request> (roslisp-msg-protocol:ros-message)
  ((gain
    :reader gain
    :initarg :gain
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetGain-request (<SetGain-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGain-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGain-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fast_cam-srv:<SetGain-request> is deprecated: use fast_cam-srv:SetGain-request instead.")))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <SetGain-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:gain-val is deprecated.  Use fast_cam-srv:gain instead.")
  (gain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGain-request>) ostream)
  "Serializes a message object of type '<SetGain-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGain-request>) istream)
  "Deserializes a message object of type '<SetGain-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gain) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGain-request>)))
  "Returns string type for a service object of type '<SetGain-request>"
  "fast_cam/SetGainRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGain-request)))
  "Returns string type for a service object of type 'SetGain-request"
  "fast_cam/SetGainRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGain-request>)))
  "Returns md5sum for a message object of type '<SetGain-request>"
  "a488df2b1501fa6d6dcdbca9c188eeff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGain-request)))
  "Returns md5sum for a message object of type 'SetGain-request"
  "a488df2b1501fa6d6dcdbca9c188eeff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGain-request>)))
  "Returns full string definition for message of type '<SetGain-request>"
  (cl:format cl:nil "# SetGain.srv~%# Request part~%float64 gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGain-request)))
  "Returns full string definition for message of type 'SetGain-request"
  (cl:format cl:nil "# SetGain.srv~%# Request part~%float64 gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGain-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGain-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGain-request
    (cl:cons ':gain (gain msg))
))
;//! \htmlinclude SetGain-response.msg.html

(cl:defclass <SetGain-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetGain-response (<SetGain-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGain-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGain-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fast_cam-srv:<SetGain-response> is deprecated: use fast_cam-srv:SetGain-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetGain-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:success-val is deprecated.  Use fast_cam-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetGain-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:message-val is deprecated.  Use fast_cam-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGain-response>) ostream)
  "Serializes a message object of type '<SetGain-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGain-response>) istream)
  "Deserializes a message object of type '<SetGain-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGain-response>)))
  "Returns string type for a service object of type '<SetGain-response>"
  "fast_cam/SetGainResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGain-response)))
  "Returns string type for a service object of type 'SetGain-response"
  "fast_cam/SetGainResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGain-response>)))
  "Returns md5sum for a message object of type '<SetGain-response>"
  "a488df2b1501fa6d6dcdbca9c188eeff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGain-response)))
  "Returns md5sum for a message object of type 'SetGain-response"
  "a488df2b1501fa6d6dcdbca9c188eeff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGain-response>)))
  "Returns full string definition for message of type '<SetGain-response>"
  (cl:format cl:nil "# Response part~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGain-response)))
  "Returns full string definition for message of type 'SetGain-response"
  (cl:format cl:nil "# Response part~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGain-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGain-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGain-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGain)))
  'SetGain-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGain)))
  'SetGain-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGain)))
  "Returns string type for a service object of type '<SetGain>"
  "fast_cam/SetGain")