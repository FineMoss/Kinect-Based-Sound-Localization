; Auto-generated. Do not edit!


(cl:in-package KBSL-msg)


;//! \htmlinclude PlaneParametersMsg.msg.html

(cl:defclass <PlaneParametersMsg> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (P0
    :reader P0
    :initarg :P0
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32)))
)

(cl:defclass PlaneParametersMsg (<PlaneParametersMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlaneParametersMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlaneParametersMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name KBSL-msg:<PlaneParametersMsg> is deprecated: use KBSL-msg:PlaneParametersMsg instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <PlaneParametersMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-msg:n-val is deprecated.  Use KBSL-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'P0-val :lambda-list '(m))
(cl:defmethod P0-val ((m <PlaneParametersMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader KBSL-msg:P0-val is deprecated.  Use KBSL-msg:P0 instead.")
  (P0 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlaneParametersMsg>) ostream)
  "Serializes a message object of type '<PlaneParametersMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'n) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'P0) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlaneParametersMsg>) istream)
  "Deserializes a message object of type '<PlaneParametersMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'n) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'P0) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlaneParametersMsg>)))
  "Returns string type for a message object of type '<PlaneParametersMsg>"
  "KBSL/PlaneParametersMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlaneParametersMsg)))
  "Returns string type for a message object of type 'PlaneParametersMsg"
  "KBSL/PlaneParametersMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlaneParametersMsg>)))
  "Returns md5sum for a message object of type '<PlaneParametersMsg>"
  "4fb5fd99bb835e5ad2523b4094d56e8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlaneParametersMsg)))
  "Returns md5sum for a message object of type 'PlaneParametersMsg"
  "4fb5fd99bb835e5ad2523b4094d56e8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlaneParametersMsg>)))
  "Returns full string definition for message of type '<PlaneParametersMsg>"
  (cl:format cl:nil "geometry_msgs/Point32 n~%geometry_msgs/Point32 P0~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlaneParametersMsg)))
  "Returns full string definition for message of type 'PlaneParametersMsg"
  (cl:format cl:nil "geometry_msgs/Point32 n~%geometry_msgs/Point32 P0~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlaneParametersMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'n))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'P0))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlaneParametersMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PlaneParametersMsg
    (cl:cons ':n (n msg))
    (cl:cons ':P0 (P0 msg))
))
