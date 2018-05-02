; Auto-generated. Do not edit!


(cl:in-package KBSL-srv)


;//! \htmlinclude TransformPointSrv-request.msg.html

(cl:defclass <TransformPointSrv-request> (roslisp-msg-protocol:ros-message)
  ((P
    :reader P
    :initarg :P
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (R
    :reader R
    :initarg :R
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (T
    :reader T
    :initarg :T
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass TransformPointSrv-request (<TransformPointSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformPointSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformPointSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<TransformPointSrv-request> is deprecated: use KBSL-srv:TransformPointSrv-request instead.")))

(cl:ensure-generic-function 'P-val :lambda-list '(m))
(cl:defmethod P-val ((m <TransformPointSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P-val is deprecated.  Use KBSL-srv:P instead.")
  (P m))

(cl:ensure-generic-function 'R-val :lambda-list '(m))
(cl:defmethod R-val ((m <TransformPointSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:R-val is deprecated.  Use KBSL-srv:R instead.")
  (R m))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <TransformPointSrv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:T-val is deprecated.  Use KBSL-srv:T instead.")
  (T m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformPointSrv-request>) ostream)
  "Serializes a message object of type '<TransformPointSrv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'R))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'T) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformPointSrv-request>) istream)
  "Deserializes a message object of type '<TransformPointSrv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P) istream)
  (cl:setf (cl:slot-value msg 'R) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'R)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'T) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformPointSrv-request>)))
  "Returns string type for a service object of type '<TransformPointSrv-request>"
  "KBSL/TransformPointSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformPointSrv-request)))
  "Returns string type for a service object of type 'TransformPointSrv-request"
  "KBSL/TransformPointSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformPointSrv-request>)))
  "Returns md5sum for a message object of type '<TransformPointSrv-request>"
  "1e5903ff15280c4ec9bfe3c87782bdd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformPointSrv-request)))
  "Returns md5sum for a message object of type 'TransformPointSrv-request"
  "1e5903ff15280c4ec9bfe3c87782bdd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformPointSrv-request>)))
  "Returns full string definition for message of type '<TransformPointSrv-request>"
  (cl:format cl:nil "geometry_msgs/Point32 P~%float32[9] R~%geometry_msgs/Point32 T~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformPointSrv-request)))
  "Returns full string definition for message of type 'TransformPointSrv-request"
  (cl:format cl:nil "geometry_msgs/Point32 P~%float32[9] R~%geometry_msgs/Point32 T~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformPointSrv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'R) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'T))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformPointSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformPointSrv-request
    (cl:cons ':P (P msg))
    (cl:cons ':R (R msg))
    (cl:cons ':T (T msg))
))
;//! \htmlinclude TransformPointSrv-response.msg.html

(cl:defclass <TransformPointSrv-response> (roslisp-msg-protocol:ros-message)
  ((P_prime
    :reader P_prime
    :initarg :P_prime
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass TransformPointSrv-response (<TransformPointSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransformPointSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransformPointSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-srv:<TransformPointSrv-response> is deprecated: use KBSL-srv:TransformPointSrv-response instead.")))

(cl:ensure-generic-function 'P_prime-val :lambda-list '(m))
(cl:defmethod P_prime-val ((m <TransformPointSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-srv:P_prime-val is deprecated.  Use KBSL-srv:P_prime instead.")
  (P_prime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransformPointSrv-response>) ostream)
  "Serializes a message object of type '<TransformPointSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P_prime) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransformPointSrv-response>) istream)
  "Deserializes a message object of type '<TransformPointSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P_prime) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransformPointSrv-response>)))
  "Returns string type for a service object of type '<TransformPointSrv-response>"
  "KBSL/TransformPointSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformPointSrv-response)))
  "Returns string type for a service object of type 'TransformPointSrv-response"
  "KBSL/TransformPointSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransformPointSrv-response>)))
  "Returns md5sum for a message object of type '<TransformPointSrv-response>"
  "1e5903ff15280c4ec9bfe3c87782bdd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransformPointSrv-response)))
  "Returns md5sum for a message object of type 'TransformPointSrv-response"
  "1e5903ff15280c4ec9bfe3c87782bdd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransformPointSrv-response>)))
  "Returns full string definition for message of type '<TransformPointSrv-response>"
  (cl:format cl:nil "geometry_msgs/Point32 P_prime~%~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransformPointSrv-response)))
  "Returns full string definition for message of type 'TransformPointSrv-response"
  (cl:format cl:nil "geometry_msgs/Point32 P_prime~%~%~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransformPointSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P_prime))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransformPointSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TransformPointSrv-response
    (cl:cons ':P_prime (P_prime msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TransformPointSrv)))
  'TransformPointSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TransformPointSrv)))
  'TransformPointSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransformPointSrv)))
  "Returns string type for a service object of type '<TransformPointSrv>"
  "KBSL/TransformPointSrv")