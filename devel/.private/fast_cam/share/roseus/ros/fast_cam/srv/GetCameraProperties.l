;; Auto-generated. Do not edit!


(when (boundp 'fast_cam::GetCameraProperties)
  (if (not (find-package "FAST_CAM"))
    (make-package "FAST_CAM"))
  (shadow 'GetCameraProperties (find-package "FAST_CAM")))
(unless (find-package "FAST_CAM::GETCAMERAPROPERTIES")
  (make-package "FAST_CAM::GETCAMERAPROPERTIES"))
(unless (find-package "FAST_CAM::GETCAMERAPROPERTIESREQUEST")
  (make-package "FAST_CAM::GETCAMERAPROPERTIESREQUEST"))
(unless (find-package "FAST_CAM::GETCAMERAPROPERTIESRESPONSE")
  (make-package "FAST_CAM::GETCAMERAPROPERTIESRESPONSE"))

(in-package "ROS")





(defclass fast_cam::GetCameraPropertiesRequest
  :super ros::object
  :slots ())

(defmethod fast_cam::GetCameraPropertiesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass fast_cam::GetCameraPropertiesResponse
  :super ros::object
  :slots (_name _model _serial_number _ip_address _resolution _target_fps _exposure_time _gain ))

(defmethod fast_cam::GetCameraPropertiesResponse
  (:init
   (&key
    ((:name __name) "")
    ((:model __model) "")
    ((:serial_number __serial_number) "")
    ((:ip_address __ip_address) "")
    ((:resolution __resolution) "")
    ((:target_fps __target_fps) 0)
    ((:exposure_time __exposure_time) 0.0)
    ((:gain __gain) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _model (string __model))
   (setq _serial_number (string __serial_number))
   (setq _ip_address (string __ip_address))
   (setq _resolution (string __resolution))
   (setq _target_fps (round __target_fps))
   (setq _exposure_time (float __exposure_time))
   (setq _gain (string __gain))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:model
   (&optional __model)
   (if __model (setq _model __model)) _model)
  (:serial_number
   (&optional __serial_number)
   (if __serial_number (setq _serial_number __serial_number)) _serial_number)
  (:ip_address
   (&optional __ip_address)
   (if __ip_address (setq _ip_address __ip_address)) _ip_address)
  (:resolution
   (&optional __resolution)
   (if __resolution (setq _resolution __resolution)) _resolution)
  (:target_fps
   (&optional __target_fps)
   (if __target_fps (setq _target_fps __target_fps)) _target_fps)
  (:exposure_time
   (&optional __exposure_time)
   (if __exposure_time (setq _exposure_time __exposure_time)) _exposure_time)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _model
    4 (length _model)
    ;; string _serial_number
    4 (length _serial_number)
    ;; string _ip_address
    4 (length _ip_address)
    ;; string _resolution
    4 (length _resolution)
    ;; uint8 _target_fps
    1
    ;; float32 _exposure_time
    4
    ;; string _gain
    4 (length _gain)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _model
       (write-long (length _model) s) (princ _model s)
     ;; string _serial_number
       (write-long (length _serial_number) s) (princ _serial_number s)
     ;; string _ip_address
       (write-long (length _ip_address) s) (princ _ip_address s)
     ;; string _resolution
       (write-long (length _resolution) s) (princ _resolution s)
     ;; uint8 _target_fps
       (write-byte _target_fps s)
     ;; float32 _exposure_time
       (sys::poke _exposure_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _gain
       (write-long (length _gain) s) (princ _gain s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _serial_number
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _serial_number (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _ip_address
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip_address (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _resolution
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _resolution (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _target_fps
     (setq _target_fps (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _exposure_time
     (setq _exposure_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _gain
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gain (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass fast_cam::GetCameraProperties
  :super ros::object
  :slots ())

(setf (get fast_cam::GetCameraProperties :md5sum-) "ca77d29133bf5a9bfbfacc01b41e087c")
(setf (get fast_cam::GetCameraProperties :datatype-) "fast_cam/GetCameraProperties")
(setf (get fast_cam::GetCameraProperties :request) fast_cam::GetCameraPropertiesRequest)
(setf (get fast_cam::GetCameraProperties :response) fast_cam::GetCameraPropertiesResponse)

(defmethod fast_cam::GetCameraPropertiesRequest
  (:response () (instance fast_cam::GetCameraPropertiesResponse :init)))

(setf (get fast_cam::GetCameraPropertiesRequest :md5sum-) "ca77d29133bf5a9bfbfacc01b41e087c")
(setf (get fast_cam::GetCameraPropertiesRequest :datatype-) "fast_cam/GetCameraPropertiesRequest")
(setf (get fast_cam::GetCameraPropertiesRequest :definition-)
      "# string set_gain
---
string name
string model
string serial_number
string ip_address
string resolution
uint8 target_fps
float32 exposure_time
string gain
")

(setf (get fast_cam::GetCameraPropertiesResponse :md5sum-) "ca77d29133bf5a9bfbfacc01b41e087c")
(setf (get fast_cam::GetCameraPropertiesResponse :datatype-) "fast_cam/GetCameraPropertiesResponse")
(setf (get fast_cam::GetCameraPropertiesResponse :definition-)
      "# string set_gain
---
string name
string model
string serial_number
string ip_address
string resolution
uint8 target_fps
float32 exposure_time
string gain
")



(provide :fast_cam/GetCameraProperties "ca77d29133bf5a9bfbfacc01b41e087c")


