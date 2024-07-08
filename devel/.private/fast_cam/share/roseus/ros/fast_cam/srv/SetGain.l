;; Auto-generated. Do not edit!


(when (boundp 'fast_cam::SetGain)
  (if (not (find-package "FAST_CAM"))
    (make-package "FAST_CAM"))
  (shadow 'SetGain (find-package "FAST_CAM")))
(unless (find-package "FAST_CAM::SETGAIN")
  (make-package "FAST_CAM::SETGAIN"))
(unless (find-package "FAST_CAM::SETGAINREQUEST")
  (make-package "FAST_CAM::SETGAINREQUEST"))
(unless (find-package "FAST_CAM::SETGAINRESPONSE")
  (make-package "FAST_CAM::SETGAINRESPONSE"))

(in-package "ROS")





(defclass fast_cam::SetGainRequest
  :super ros::object
  :slots (_gain ))

(defmethod fast_cam::SetGainRequest
  (:init
   (&key
    ((:gain __gain) 0.0)
    )
   (send-super :init)
   (setq _gain (float __gain))
   self)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:serialization-length
   ()
   (+
    ;; float64 _gain
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _gain
       (sys::poke _gain (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _gain
     (setq _gain (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass fast_cam::SetGainResponse
  :super ros::object
  :slots (_success _message ))

(defmethod fast_cam::SetGainResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass fast_cam::SetGain
  :super ros::object
  :slots ())

(setf (get fast_cam::SetGain :md5sum-) "a488df2b1501fa6d6dcdbca9c188eeff")
(setf (get fast_cam::SetGain :datatype-) "fast_cam/SetGain")
(setf (get fast_cam::SetGain :request) fast_cam::SetGainRequest)
(setf (get fast_cam::SetGain :response) fast_cam::SetGainResponse)

(defmethod fast_cam::SetGainRequest
  (:response () (instance fast_cam::SetGainResponse :init)))

(setf (get fast_cam::SetGainRequest :md5sum-) "a488df2b1501fa6d6dcdbca9c188eeff")
(setf (get fast_cam::SetGainRequest :datatype-) "fast_cam/SetGainRequest")
(setf (get fast_cam::SetGainRequest :definition-)
      "# SetGain.srv
# Request part
float64 gain
---
# Response part
bool success
string message
")

(setf (get fast_cam::SetGainResponse :md5sum-) "a488df2b1501fa6d6dcdbca9c188eeff")
(setf (get fast_cam::SetGainResponse :datatype-) "fast_cam/SetGainResponse")
(setf (get fast_cam::SetGainResponse :definition-)
      "# SetGain.srv
# Request part
float64 gain
---
# Response part
bool success
string message
")



(provide :fast_cam/SetGain "a488df2b1501fa6d6dcdbca9c188eeff")


