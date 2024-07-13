; Auto-generated. Do not edit!


(cl:in-package husky_follow_light-srv)


;//! \htmlinclude PerformAction-request.msg.html

(cl:defclass <PerformAction-request> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type cl:string
    :initform ""))
)

(cl:defclass PerformAction-request (<PerformAction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerformAction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerformAction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_follow_light-srv:<PerformAction-request> is deprecated: use husky_follow_light-srv:PerformAction-request instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <PerformAction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_follow_light-srv:action-val is deprecated.  Use husky_follow_light-srv:action instead.")
  (action m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerformAction-request>) ostream)
  "Serializes a message object of type '<PerformAction-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'action))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'action))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerformAction-request>) istream)
  "Deserializes a message object of type '<PerformAction-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'action) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerformAction-request>)))
  "Returns string type for a service object of type '<PerformAction-request>"
  "husky_follow_light/PerformActionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformAction-request)))
  "Returns string type for a service object of type 'PerformAction-request"
  "husky_follow_light/PerformActionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerformAction-request>)))
  "Returns md5sum for a message object of type '<PerformAction-request>"
  "02058b7d55716526fae62eb68abd6f31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerformAction-request)))
  "Returns md5sum for a message object of type 'PerformAction-request"
  "02058b7d55716526fae62eb68abd6f31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerformAction-request>)))
  "Returns full string definition for message of type '<PerformAction-request>"
  (cl:format cl:nil "string action~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerformAction-request)))
  "Returns full string definition for message of type 'PerformAction-request"
  (cl:format cl:nil "string action~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerformAction-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'action))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerformAction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PerformAction-request
    (cl:cons ':action (action msg))
))
;//! \htmlinclude PerformAction-response.msg.html

(cl:defclass <PerformAction-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PerformAction-response (<PerformAction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerformAction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerformAction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_follow_light-srv:<PerformAction-response> is deprecated: use husky_follow_light-srv:PerformAction-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PerformAction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_follow_light-srv:success-val is deprecated.  Use husky_follow_light-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerformAction-response>) ostream)
  "Serializes a message object of type '<PerformAction-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerformAction-response>) istream)
  "Deserializes a message object of type '<PerformAction-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerformAction-response>)))
  "Returns string type for a service object of type '<PerformAction-response>"
  "husky_follow_light/PerformActionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformAction-response)))
  "Returns string type for a service object of type 'PerformAction-response"
  "husky_follow_light/PerformActionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerformAction-response>)))
  "Returns md5sum for a message object of type '<PerformAction-response>"
  "02058b7d55716526fae62eb68abd6f31")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerformAction-response)))
  "Returns md5sum for a message object of type 'PerformAction-response"
  "02058b7d55716526fae62eb68abd6f31")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerformAction-response>)))
  "Returns full string definition for message of type '<PerformAction-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerformAction-response)))
  "Returns full string definition for message of type 'PerformAction-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerformAction-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerformAction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PerformAction-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PerformAction)))
  'PerformAction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PerformAction)))
  'PerformAction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerformAction)))
  "Returns string type for a service object of type '<PerformAction>"
  "husky_follow_light/PerformAction")