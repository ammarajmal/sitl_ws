;; Auto-generated. Do not edit!


(when (boundp 'fast_cam::CameraSpecs)
  (if (not (find-package "FAST_CAM"))
    (make-package "FAST_CAM"))
  (shadow 'CameraSpecs (find-package "FAST_CAM")))
(unless (find-package "FAST_CAM::CAMERASPECS")
  (make-package "FAST_CAM::CAMERASPECS"))

(in-package "ROS")
;;//! \htmlinclude CameraSpecs.msg.html


(defclass fast_cam::CameraSpecs
  :super ros::object
  :slots (_name _model _serial_number _ip_address _resolution _frame_rate _exposure_time _gain ))

(defmethod fast_cam::CameraSpecs
  (:init
   (&key
    ((:name __name) "")
    ((:model __model) "")
    ((:serial_number __serial_number) "")
    ((:ip_address __ip_address) "")
    ((:resolution __resolution) "")
    ((:frame_rate __frame_rate) 0.0)
    ((:exposure_time __exposure_time) 0.0)
    ((:gain __gain) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _model (string __model))
   (setq _serial_number (string __serial_number))
   (setq _ip_address (string __ip_address))
   (setq _resolution (string __resolution))
   (setq _frame_rate (float __frame_rate))
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
  (:frame_rate
   (&optional __frame_rate)
   (if __frame_rate (setq _frame_rate __frame_rate)) _frame_rate)
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
    ;; float64 _frame_rate
    8
    ;; float64 _exposure_time
    8
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
     ;; float64 _frame_rate
       (sys::poke _frame_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _exposure_time
       (sys::poke _exposure_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
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
   ;; float64 _frame_rate
     (setq _frame_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _exposure_time
     (setq _exposure_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _gain
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gain (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get fast_cam::CameraSpecs :md5sum-) "5d47430f74f6ea64f601ee2640f4e549")
(setf (get fast_cam::CameraSpecs :datatype-) "fast_cam/CameraSpecs")
(setf (get fast_cam::CameraSpecs :definition-)
      "# CameraSpecs.msg
string name
string model
string serial_number
string ip_address
string resolution
float64 frame_rate
float64 exposure_time
string gain
")



(provide :fast_cam/CameraSpecs "5d47430f74f6ea64f601ee2640f4e549")


