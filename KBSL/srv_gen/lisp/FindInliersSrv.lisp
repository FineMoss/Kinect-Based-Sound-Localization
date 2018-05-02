; Auto-generated. Do not edit!


(cl:in-package KBSL-srv)


;//! \htmlinclude FindInliersSrv-request.msg.html

(cl:defclass <FindInliersSrv-request> (roslisp-msg-protocol:ros-message)
  ((P
    :reader P
    :initarg :P
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32)))
   (P0
    :reader P0
    :initarg :P0
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (n
    :reader n
    :initarg :n
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (epsilon
    :reader epsilon
    :initarg :epsilon
    :type cl:float
    :initform 0.0))
)

(cl:defclass FindInliersSrv-request (<FindInliersSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindInliersSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindInliersSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<FindInliersSrv-request> is deprecated: use KBSL-srv:FindInliersSrv-request instead.")))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <FindInliersSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P-val is deprecated.  Use KBSL-srv:P instead.")
  (P m))

(cl:ensure-generic-function 'P0-val :lambda-list '(m))
(cl:defmethod P0-val ((m <FindInliersSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P0-val is deprecated.  Use KBSL-srv:P0 instead.")
  (P0 m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <FindInliersSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:n-val is deprecated.  Use KBSL-srv:n instead.")
  (n m))

(cl:ensure-generic-function 'epsilon-val :lambda-list '(m))
(cl:defmethod epsilon-val ((m <FindInliersSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:epsilon-val is deprecated.  Use KBSL-srv:epsilon instead.")
  (epsilon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindInliersSrv-request>) ostream)
  "Serializes a message object of type '<FindInliersSrv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'P))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'n) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'epsilon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindInliersSrv-request>) istream)
  "Deserializes a message object of type '<FindInliersSrv-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'P) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'P)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'n) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'epsilon) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindInliersSrv-request>)))
  "Returns string type for a service object of type '<FindInliersSrv-request>"
  "KBSL/FindInliersSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindInliersSrv-request)))
  "Returns string type for a service object of type 'FindInliersSrv-request"
  "KBSL/FindInliersSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindInliersSrv-request>)))
  "Returns md5sum for a message object of type '<FindInliersSrv-request>"
  "fcedaa1fba47f5b21446e97098d7d060")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindInliersSrv-request)))
  "Returns md5sum for a message object of type 'FindInliersSrv-request"
  "fcedaa1fba47f5b21446e97098d7d060")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindInliersSrv-request>)))
  "Returns full string definition for message of type '<FindInliersSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point32[] P~%geometry_msgs/Point32 P0~%geometry_msgs/Point32 n~%float32 epsilon~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindInliersSrv-request)))
  "Returns full string definition for message of type 'FindInliersSrv-request"
  (cl:format cl:nil "geometry_msgs/Point32[] P~%geometry_msgs/Point32 P0~%geometry_msgs/Point32 n~%float32 epsilon~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindInliersSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'P) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'n))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindInliersSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FindInliersSrv-request
    (cl:cons ':P (P msg))
    (cl:cons ':P0 (P0 msg))
    (cl:cons ':n (n msg))
    (cl:cons ':epsilon (epsilon msg))
))
;//! \htmlinclude FindInliersSrv-response.msg.html

(cl:defclass <FindInliersSrv-response> (roslisp-msg-protocol:ros-message)
  ((P
    :reader P
    :initarg :P
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass FindInliersSrv-response (<FindInliersSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindInliersSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindInliersSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<FindInliersSrv-response> is deprecated: use KBSL-srv:FindInliersSrv-response instead.")))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <FindInliersSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P-val is deprecated.  Use KBSL-srv:P instead.")
  (P m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindInliersSrv-response>) ostream)
  "Serializes a message object of type '<FindInliersSrv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'P))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindInliersSrv-response>) istream)
  "Deserializes a message object of type '<FindInliersSrv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'P) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'P)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindInliersSrv-response>)))
  "Returns string type for a service object of type '<FindInliersSrv-response>"
  "KBSL/FindInliersSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindInliersSrv-response)))
  "Returns string type for a service object of type 'FindInliersSrv-response"
  "KBSL/FindInliersSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindInliersSrv-response>)))
  "Returns md5sum for a message object of type '<FindInliersSrv-response>"
  "fcedaa1fba47f5b21446e97098d7d060")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindInliersSrv-response)))
  "Returns md5sum for a message object of type 'FindInliersSrv-response"
  "fcedaa1fba47f5b21446e97098d7d060")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindInliersSrv-response>)))
  "Returns full string definition for message of type '<FindInliersSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point32[] P~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindInliersSrv-response)))
  "Returns full string definition for message of type 'FindInliersSrv-response"
  (cl:format cl:nil "geometry_msgs/Point32[] P~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindInliersSrv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'P) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindInliersSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FindInliersSrv-response
    (cl:cons ':P (P msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FindInliersSrv)))
  'FindInliersSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FindInliersSrv)))
  'FindInliersSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindInliersSrv)))
  "Returns string type for a service object of type '<FindInliersSrv>"
  "KBSL/FindInliersSrv")