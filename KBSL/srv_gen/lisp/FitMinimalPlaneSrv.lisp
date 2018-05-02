; Auto-generated. Do not edit!


(cl:in-package KBSL-srv)


;//! \htmlinclude FitMinimalPlaneSrv-request.msg.html

(cl:defclass <FitMinimalPlaneSrv-request> (roslisp-msg-protocol:ros-message)
  ((P1
    :reader P1
    :initarg :P1
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (P2
    :reader P2
    :initarg :P2
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (P3
    :reader P3
    :initarg :P3
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass FitMinimalPlaneSrv-request (<FitMinimalPlaneSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FitMinimalPlaneSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FitMinimalPlaneSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<FitMinimalPlaneSrv-request> is deprecated: use KBSL-srv:FitMinimalPlaneSrv-request instead.")))

(cl:ensure-generic-function 'P1-val :lambda-list '(m))
(cl:defmethod P1-val ((m <FitMinimalPlaneSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P1-val is deprecated.  Use KBSL-srv:P1 instead.")
  (P1 m))

(cl:ensure-generic-function 'P2-val :lambda-list '(m))
(cl:defmethod P2-val ((m <FitMinimalPlaneSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P2-val is deprecated.  Use KBSL-srv:P2 instead.")
  (P2 m))

(cl:ensure-generic-function 'P3-val :lambda-list '(m))
(cl:defmethod P3-val ((m <FitMinimalPlaneSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P3-val is deprecated.  Use KBSL-srv:P3 instead.")
  (P3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FitMinimalPlaneSrv-request>) ostream)
  "Serializes a message object of type '<FitMinimalPlaneSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P3) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FitMinimalPlaneSrv-request>) istream)
  "Deserializes a message object of type '<FitMinimalPlaneSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P3) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FitMinimalPlaneSrv-request>)))
  "Returns string type for a service object of type '<FitMinimalPlaneSrv-request>"
  "KBSL/FitMinimalPlaneSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FitMinimalPlaneSrv-request)))
  "Returns string type for a service object of type 'FitMinimalPlaneSrv-request"
  "KBSL/FitMinimalPlaneSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FitMinimalPlaneSrv-request>)))
  "Returns md5sum for a message object of type '<FitMinimalPlaneSrv-request>"
  "0a7e6160a47703bcae7741304dcb479d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FitMinimalPlaneSrv-request)))
  "Returns md5sum for a message object of type 'FitMinimalPlaneSrv-request"
  "0a7e6160a47703bcae7741304dcb479d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FitMinimalPlaneSrv-request>)))
  "Returns full string definition for message of type '<FitMinimalPlaneSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point32 P1~%geometry_msgs/Point32 P2~%geometry_msgs/Point32 P3~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FitMinimalPlaneSrv-request)))
  "Returns full string definition for message of type 'FitMinimalPlaneSrv-request"
  (cl:format cl:nil "geometry_msgs/Point32 P1~%geometry_msgs/Point32 P2~%geometry_msgs/Point32 P3~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FitMinimalPlaneSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P3))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FitMinimalPlaneSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FitMinimalPlaneSrv-request
    (cl:cons ':P1 (P1 msg))
    (cl:cons ':P2 (P2 msg))
    (cl:cons ':P3 (P3 msg))
))
;//! \htmlinclude FitMinimalPlaneSrv-response.msg.html

(cl:defclass <FitMinimalPlaneSrv-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass FitMinimalPlaneSrv-response (<FitMinimalPlaneSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FitMinimalPlaneSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FitMinimalPlaneSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<FitMinimalPlaneSrv-response> is deprecated: use KBSL-srv:FitMinimalPlaneSrv-response instead.")))

(cl:ensure-generic-function 'P0-val :lambda-list '(m))
(cl:defmethod P0-val ((m <FitMinimalPlaneSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P0-val is deprecated.  Use KBSL-srv:P0 instead.")
  (P0 m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <FitMinimalPlaneSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:n-val is deprecated.  Use KBSL-srv:n instead.")
  (n m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FitMinimalPlaneSrv-response>) ostream)
  "Serializes a message object of type '<FitMinimalPlaneSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'n) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FitMinimalPlaneSrv-response>) istream)
  "Deserializes a message object of type '<FitMinimalPlaneSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'n) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FitMinimalPlaneSrv-response>)))
  "Returns string type for a service object of type '<FitMinimalPlaneSrv-response>"
  "KBSL/FitMinimalPlaneSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FitMinimalPlaneSrv-response)))
  "Returns string type for a service object of type 'FitMinimalPlaneSrv-response"
  "KBSL/FitMinimalPlaneSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FitMinimalPlaneSrv-response>)))
  "Returns md5sum for a message object of type '<FitMinimalPlaneSrv-response>"
  "0a7e6160a47703bcae7741304dcb479d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FitMinimalPlaneSrv-response)))
  "Returns md5sum for a message object of type 'FitMinimalPlaneSrv-response"
  "0a7e6160a47703bcae7741304dcb479d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FitMinimalPlaneSrv-response>)))
  "Returns full string definition for message of type '<FitMinimalPlaneSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point32 P0~%geometry_msgs/Point32 n~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FitMinimalPlaneSrv-response)))
  "Returns full string definition for message of type 'FitMinimalPlaneSrv-response"
  (cl:format cl:nil "geometry_msgs/Point32 P0~%geometry_msgs/Point32 n~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FitMinimalPlaneSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'n))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FitMinimalPlaneSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FitMinimalPlaneSrv-response
    (cl:cons ':P0 (P0 msg))
    (cl:cons ':n (n msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FitMinimalPlaneSrv)))
  'FitMinimalPlaneSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FitMinimalPlaneSrv)))
  'FitMinimalPlaneSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FitMinimalPlaneSrv)))
  "Returns string type for a service object of type '<FitMinimalPlaneSrv>"
  "KBSL/FitMinimalPlaneSrv")