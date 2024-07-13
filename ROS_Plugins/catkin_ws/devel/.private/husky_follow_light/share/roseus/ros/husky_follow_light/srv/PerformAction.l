;; Auto-generated. Do not edit!


(when (boundp 'husky_follow_light::PerformAction)
  (if (not (find-package "HUSKY_FOLLOW_LIGHT"))
    (make-package "HUSKY_FOLLOW_LIGHT"))
  (shadow 'PerformAction (find-package "HUSKY_FOLLOW_LIGHT")))
(unless (find-package "HUSKY_FOLLOW_LIGHT::PERFORMACTION")
  (make-package "HUSKY_FOLLOW_LIGHT::PERFORMACTION"))
(unless (find-package "HUSKY_FOLLOW_LIGHT::PERFORMACTIONREQUEST")
  (make-package "HUSKY_FOLLOW_LIGHT::PERFORMACTIONREQUEST"))
(unless (find-package "HUSKY_FOLLOW_LIGHT::PERFORMACTIONRESPONSE")
  (make-package "HUSKY_FOLLOW_LIGHT::PERFORMACTIONRESPONSE"))

(in-package "ROS")





(defclass husky_follow_light::PerformActionRequest
  :super ros::object
  :slots (_action ))

(defmethod husky_follow_light::PerformActionRequest
  (:init
   (&key
    ((:action __action) "")
    )
   (send-super :init)
   (setq _action (string __action))
   self)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; string _action
    4 (length _action)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _action
       (write-long (length _action) s) (princ _action s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _action
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass husky_follow_light::PerformActionResponse
  :super ros::object
  :slots (_success ))

(defmethod husky_follow_light::PerformActionResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass husky_follow_light::PerformAction
  :super ros::object
  :slots ())

(setf (get husky_follow_light::PerformAction :md5sum-) "02058b7d55716526fae62eb68abd6f31")
(setf (get husky_follow_light::PerformAction :datatype-) "husky_follow_light/PerformAction")
(setf (get husky_follow_light::PerformAction :request) husky_follow_light::PerformActionRequest)
(setf (get husky_follow_light::PerformAction :response) husky_follow_light::PerformActionResponse)

(defmethod husky_follow_light::PerformActionRequest
  (:response () (instance husky_follow_light::PerformActionResponse :init)))

(setf (get husky_follow_light::PerformActionRequest :md5sum-) "02058b7d55716526fae62eb68abd6f31")
(setf (get husky_follow_light::PerformActionRequest :datatype-) "husky_follow_light/PerformActionRequest")
(setf (get husky_follow_light::PerformActionRequest :definition-)
      "string action
---
bool success
")

(setf (get husky_follow_light::PerformActionResponse :md5sum-) "02058b7d55716526fae62eb68abd6f31")
(setf (get husky_follow_light::PerformActionResponse :datatype-) "husky_follow_light/PerformActionResponse")
(setf (get husky_follow_light::PerformActionResponse :definition-)
      "string action
---
bool success
")



(provide :husky_follow_light/PerformAction "02058b7d55716526fae62eb68abd6f31")


