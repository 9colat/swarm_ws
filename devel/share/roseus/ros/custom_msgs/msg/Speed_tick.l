;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::Speed_tick)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'Speed_tick (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::SPEED_TICK")
  (make-package "CUSTOM_MSGS::SPEED_TICK"))

(in-package "ROS")
;;//! \htmlinclude Speed_tick.msg.html


(defclass custom_msgs::Speed_tick
  :super ros::object
  :slots (_right_speed _left_speed _right_tick _left_tick ))

(defmethod custom_msgs::Speed_tick
  (:init
   (&key
    ((:right_speed __right_speed) 0.0)
    ((:left_speed __left_speed) 0.0)
    ((:right_tick __right_tick) 0)
    ((:left_tick __left_tick) 0)
    )
   (send-super :init)
   (setq _right_speed (float __right_speed))
   (setq _left_speed (float __left_speed))
   (setq _right_tick (round __right_tick))
   (setq _left_tick (round __left_tick))
   self)
  (:right_speed
   (&optional __right_speed)
   (if __right_speed (setq _right_speed __right_speed)) _right_speed)
  (:left_speed
   (&optional __left_speed)
   (if __left_speed (setq _left_speed __left_speed)) _left_speed)
  (:right_tick
   (&optional __right_tick)
   (if __right_tick (setq _right_tick __right_tick)) _right_tick)
  (:left_tick
   (&optional __left_tick)
   (if __left_tick (setq _left_tick __left_tick)) _left_tick)
  (:serialization-length
   ()
   (+
    ;; float64 _right_speed
    8
    ;; float64 _left_speed
    8
    ;; int32 _right_tick
    4
    ;; int32 _left_tick
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _right_speed
       (sys::poke _right_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_speed
       (sys::poke _left_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _right_tick
       (write-long _right_tick s)
     ;; int32 _left_tick
       (write-long _left_tick s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _right_speed
     (setq _right_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_speed
     (setq _left_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _right_tick
     (setq _right_tick (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _left_tick
     (setq _left_tick (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msgs::Speed_tick :md5sum-) "49db339e4cdd08bb4996af3a92cbcc3d")
(setf (get custom_msgs::Speed_tick :datatype-) "custom_msgs/Speed_tick")
(setf (get custom_msgs::Speed_tick :definition-)
      "# This represents the right and left speed plus the right and left tick

float64 right_speed
float64 left_speed
int32 right_tick
int32 left_tick

")



(provide :custom_msgs/Speed_tick "49db339e4cdd08bb4996af3a92cbcc3d")


