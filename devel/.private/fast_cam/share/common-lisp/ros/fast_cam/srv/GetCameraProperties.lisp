; Auto-generated. Do not edit!


(cl:in-package fast_cam-srv)


;//! \htmlinclude GetCameraProperties-request.msg.html

(cl:defclass <GetCameraProperties-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCameraProperties-request (<GetCameraProperties-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCameraProperties-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCameraProperties-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fast_cam-srv:<GetCameraProperties-request> is deprecated: use fast_cam-srv:GetCameraProperties-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCameraProperties-request>) ostream)
  "Serializes a message object of type '<GetCameraProperties-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCameraProperties-request>) istream)
  "Deserializes a message object of type '<GetCameraProperties-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCameraProperties-request>)))
  "Returns string type for a service object of type '<GetCameraProperties-request>"
  "fast_cam/GetCameraPropertiesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCameraProperties-request)))
  "Returns string type for a service object of type 'GetCameraProperties-request"
  "fast_cam/GetCameraPropertiesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCameraProperties-request>)))
  "Returns md5sum for a message object of type '<GetCameraProperties-request>"
  "ca77d29133bf5a9bfbfacc01b41e087c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCameraProperties-request)))
  "Returns md5sum for a message object of type 'GetCameraProperties-request"
  "ca77d29133bf5a9bfbfacc01b41e087c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCameraProperties-request>)))
  "Returns full string definition for message of type '<GetCameraProperties-request>"
  (cl:format cl:nil "# string set_gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCameraProperties-request)))
  "Returns full string definition for message of type 'GetCameraProperties-request"
  (cl:format cl:nil "# string set_gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCameraProperties-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCameraProperties-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCameraProperties-request
))
;//! \htmlinclude GetCameraProperties-response.msg.html

(cl:defclass <GetCameraProperties-response> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (model
    :reader model
    :initarg :model
    :type cl:string
    :initform "")
   (serial_number
    :reader serial_number
    :initarg :serial_number
    :type cl:string
    :initform "")
   (ip_address
    :reader ip_address
    :initarg :ip_address
    :type cl:string
    :initform "")
   (resolution
    :reader resolution
    :initarg :resolution
    :type cl:string
    :initform "")
   (target_fps
    :reader target_fps
    :initarg :target_fps
    :type cl:fixnum
    :initform 0)
   (exposure_time
    :reader exposure_time
    :initarg :exposure_time
    :type cl:float
    :initform 0.0)
   (gain
    :reader gain
    :initarg :gain
    :type cl:string
    :initform ""))
)

(cl:defclass GetCameraProperties-response (<GetCameraProperties-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCameraProperties-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCameraProperties-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fast_cam-srv:<GetCameraProperties-response> is deprecated: use fast_cam-srv:GetCameraProperties-response instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:name-val is deprecated.  Use fast_cam-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:model-val is deprecated.  Use fast_cam-srv:model instead.")
  (model m))

(cl:ensure-generic-function 'serial_number-val :lambda-list '(m))
(cl:defmethod serial_number-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:serial_number-val is deprecated.  Use fast_cam-srv:serial_number instead.")
  (serial_number m))

(cl:ensure-generic-function 'ip_address-val :lambda-list '(m))
(cl:defmethod ip_address-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:ip_address-val is deprecated.  Use fast_cam-srv:ip_address instead.")
  (ip_address m))

(cl:ensure-generic-function 'resolution-val :lambda-list '(m))
(cl:defmethod resolution-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:resolution-val is deprecated.  Use fast_cam-srv:resolution instead.")
  (resolution m))

(cl:ensure-generic-function 'target_fps-val :lambda-list '(m))
(cl:defmethod target_fps-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:target_fps-val is deprecated.  Use fast_cam-srv:target_fps instead.")
  (target_fps m))

(cl:ensure-generic-function 'exposure_time-val :lambda-list '(m))
(cl:defmethod exposure_time-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:exposure_time-val is deprecated.  Use fast_cam-srv:exposure_time instead.")
  (exposure_time m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <GetCameraProperties-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fast_cam-srv:gain-val is deprecated.  Use fast_cam-srv:gain instead.")
  (gain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCameraProperties-response>) ostream)
  "Serializes a message object of type '<GetCameraProperties-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'serial_number))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'serial_number))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ip_address))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ip_address))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'resolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'resolution))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_fps)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'exposure_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'gain))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCameraProperties-response>) istream)
  "Deserializes a message object of type '<GetCameraProperties-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'serial_number) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'serial_number) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ip_address) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ip_address) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resolution) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'resolution) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_fps)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'exposure_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gain) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'gain) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCameraProperties-response>)))
  "Returns string type for a service object of type '<GetCameraProperties-response>"
  "fast_cam/GetCameraPropertiesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCameraProperties-response)))
  "Returns string type for a service object of type 'GetCameraProperties-response"
  "fast_cam/GetCameraPropertiesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCameraProperties-response>)))
  "Returns md5sum for a message object of type '<GetCameraProperties-response>"
  "ca77d29133bf5a9bfbfacc01b41e087c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCameraProperties-response)))
  "Returns md5sum for a message object of type 'GetCameraProperties-response"
  "ca77d29133bf5a9bfbfacc01b41e087c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCameraProperties-response>)))
  "Returns full string definition for message of type '<GetCameraProperties-response>"
  (cl:format cl:nil "string name~%string model~%string serial_number~%string ip_address~%string resolution~%uint8 target_fps~%float32 exposure_time~%string gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCameraProperties-response)))
  "Returns full string definition for message of type 'GetCameraProperties-response"
  (cl:format cl:nil "string name~%string model~%string serial_number~%string ip_address~%string resolution~%uint8 target_fps~%float32 exposure_time~%string gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCameraProperties-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'model))
     4 (cl:length (cl:slot-value msg 'serial_number))
     4 (cl:length (cl:slot-value msg 'ip_address))
     4 (cl:length (cl:slot-value msg 'resolution))
     1
     4
     4 (cl:length (cl:slot-value msg 'gain))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCameraProperties-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCameraProperties-response
    (cl:cons ':name (name msg))
    (cl:cons ':model (model msg))
    (cl:cons ':serial_number (serial_number msg))
    (cl:cons ':ip_address (ip_address msg))
    (cl:cons ':resolution (resolution msg))
    (cl:cons ':target_fps (target_fps msg))
    (cl:cons ':exposure_time (exposure_time msg))
    (cl:cons ':gain (gain msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCameraProperties)))
  'GetCameraProperties-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCameraProperties)))
  'GetCameraProperties-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCameraProperties)))
  "Returns string type for a service object of type '<GetCameraProperties>"
  "fast_cam/GetCameraProperties")