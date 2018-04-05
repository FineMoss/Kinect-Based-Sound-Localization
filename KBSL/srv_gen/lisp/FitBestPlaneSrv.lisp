; Auto-generated. Do not edit!


(cl:in-package KBSL-srv)


;//! \htmlinclude FitBestPlaneSrv-request.msg.html

(cl:defclass <FitBestPlaneSrv-request> (roslisp-msg-protocol:ros-message)
  ((P
    :reader P
    :initarg :P
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32))))
)

(cl:defclass FitBestPlaneSrv-request (<FitBestPlaneSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FitBestPlaneSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FitBestPlaneSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<FitBestPlaneSrv-request> is deprecated: use KBSL-srv:FitBestPlaneSrv-request instead.")))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <FitBestPlaneSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P-val is deprecated.  Use KBSL-srv:P instead.")
  (P m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FitBestPlaneSrv-request>) ostream)
  "Serializes a message object of type '<FitBestPlaneSrv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'P))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FitBestPlaneSrv-request>) istream)
  "Deserializes a message object of type '<FitBestPlaneSrv-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FitBestPlaneSrv-request>)))
  "Returns string type for a service object of type '<FitBestPlaneSrv-request>"
  "KBSL/FitBestPlaneSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FitBestPlaneSrv-request)))
  "Returns string type for a service object of type 'FitBestPlaneSrv-request"
  "KBSL/FitBestPlaneSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FitBestPlaneSrv-request>)))
  "Returns md5sum for a message object of type '<FitBestPlaneSrv-request>"
  "dc3f7ccf7e8285059dcdda5775cca1b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FitBestPlaneSrv-request)))
  "Returns md5sum for a message object of type 'FitBestPlaneSrv-request"
  "dc3f7ccf7e8285059dcdda5775cca1b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FitBestPlaneSrv-request>)))
  "Returns full string definition for message of type '<FitBestPlaneSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point32[] P~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FitBestPlaneSrv-request)))
  "Returns full string definition for message of type 'FitBestPlaneSrv-request"
  (cl:format cl:nil "geometry_msgs/Point32[] P~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FitBestPlaneSrv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'P) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FitBestPlaneSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FitBestPlaneSrv-request
    (cl:cons ':P (P msg))
))
;//! \htmlinclude FitBestPlaneSrv-response.msg.html

(cl:defclass <FitBestPlaneSrv-response> (roslisp-msg-protocol:ros-message)
  ((P0
    :reader P0
    :initarg :P0
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (n
    :reader n
    :initarg :n
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass FitBestPlaneSrv-response (<FitBestPlaneSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FitBestPlaneSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FitBestPlaneSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<FitBestPlaneSrv-response> is deprecated: use KBSL-srv:FitBestPlaneSrv-response instead.")))

(cl:ensure-generic-function 'P0-val :lambda-list '(m))
(cl:defmethod P0-val ((m <FitBestPlaneSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P0-val is deprecated.  Use KBSL-srv:P0 instead.")
  (P0 m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <FitBestPlaneSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:n-val is deprecated.  Use KBSL-srv:n instead.")
  (n m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FitBestPlaneSrv-response>) ostream)
  "Serializes a message object of type '<FitBestPlaneSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'n) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FitBestPlaneSrv-response>) istream)
  "Deserializes a message object of type '<FitBestPlaneSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'n) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FitBestPlaneSrv-response>)))
  "Returns string type for a service object of type '<FitBestPlaneSrv-response>"
  "KBSL/FitBestPlaneSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FitBestPlaneSrv-response)))
  "Returns string type for a service object of type 'FitBestPlaneSrv-response"
  "KBSL/FitBestPlaneSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FitBestPlaneSrv-response>)))
  "Returns md5sum for a message object of type '<FitBestPlaneSrv-response>"
  "dc3f7ccf7e8285059dcdda5775cca1b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FitBestPlaneSrv-response)))
  "Returns md5sum for a message object of type 'FitBestPlaneSrv-response"
  "dc3f7ccf7e8285059dcdda5775cca1b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FitBestPlaneSrv-response>)))
  "Returns full string definition for message of type '<FitBestPlaneSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point32 P0~%geometry_msgs/Point32 n~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FitBestPlaneSrv-response)))
  "Returns full string definition for message of type 'FitBestPlaneSrv-response"
  (cl:format cl:nil "geometry_msgs/Point32 P0~%geometry_msgs/Point32 n~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FitBestPlaneSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'n))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FitBestPlaneSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FitBestPlaneSrv-response
    (cl:cons ':P0 (P0 msg))
    (cl:cons ':n (n msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FitBestPlaneSrv)))
  'FitBestPlaneSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FitBestPlaneSrv)))
  'FitBestPlaneSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FitBestPlaneSrv)))
  "Returns string type for a service object of type '<FitBestPlaneSrv>"
  "KBSL/FitBestPlaneSrv")