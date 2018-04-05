# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from KBSL/FitMinimalPlaneSrvRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class FitMinimalPlaneSrvRequest(genpy.Message):
  _md5sum = "31aaecb61f65fa1b1cf756f63eacadf1"
  _type = "KBSL/FitMinimalPlaneSrvRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Point32 P1
geometry_msgs/Point32 P2
geometry_msgs/Point32 P3

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"""
  __slots__ = ['P1','P2','P3']
  _slot_types = ['geometry_msgs/Point32','geometry_msgs/Point32','geometry_msgs/Point32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       P1,P2,P3

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FitMinimalPlaneSrvRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.P1 is None:
        self.P1 = geometry_msgs.msg.Point32()
      if self.P2 is None:
        self.P2 = geometry_msgs.msg.Point32()
      if self.P3 is None:
        self.P3 = geometry_msgs.msg.Point32()
    else:
      self.P1 = geometry_msgs.msg.Point32()
      self.P2 = geometry_msgs.msg.Point32()
      self.P3 = geometry_msgs.msg.Point32()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_9f().pack(_x.P1.x, _x.P1.y, _x.P1.z, _x.P2.x, _x.P2.y, _x.P2.z, _x.P3.x, _x.P3.y, _x.P3.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.P1 is None:
        self.P1 = geometry_msgs.msg.Point32()
      if self.P2 is None:
        self.P2 = geometry_msgs.msg.Point32()
      if self.P3 is None:
        self.P3 = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 36
      (_x.P1.x, _x.P1.y, _x.P1.z, _x.P2.x, _x.P2.y, _x.P2.z, _x.P3.x, _x.P3.y, _x.P3.z,) = _get_struct_9f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_9f().pack(_x.P1.x, _x.P1.y, _x.P1.z, _x.P2.x, _x.P2.y, _x.P2.z, _x.P3.x, _x.P3.y, _x.P3.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.P1 is None:
        self.P1 = geometry_msgs.msg.Point32()
      if self.P2 is None:
        self.P2 = geometry_msgs.msg.Point32()
      if self.P3 is None:
        self.P3 = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 36
      (_x.P1.x, _x.P1.y, _x.P1.z, _x.P2.x, _x.P2.y, _x.P2.z, _x.P3.x, _x.P3.y, _x.P3.z,) = _get_struct_9f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_9f = None
def _get_struct_9f():
    global _struct_9f
    if _struct_9f is None:
        _struct_9f = struct.Struct("<9f")
    return _struct_9f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from KBSL/FitMinimalPlaneSrvResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class FitMinimalPlaneSrvResponse(genpy.Message):
  _md5sum = "185eea9c3fedfb05e4edf26c506b38ef"
  _type = "KBSL/FitMinimalPlaneSrvResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Point32 P0
geometry_msgs/Point32 n


================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"""
  __slots__ = ['P0','n']
  _slot_types = ['geometry_msgs/Point32','geometry_msgs/Point32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       P0,n

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(FitMinimalPlaneSrvResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.P0 is None:
        self.P0 = geometry_msgs.msg.Point32()
      if self.n is None:
        self.n = geometry_msgs.msg.Point32()
    else:
      self.P0 = geometry_msgs.msg.Point32()
      self.n = geometry_msgs.msg.Point32()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6f().pack(_x.P0.x, _x.P0.y, _x.P0.z, _x.n.x, _x.n.y, _x.n.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.P0 is None:
        self.P0 = geometry_msgs.msg.Point32()
      if self.n is None:
        self.n = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.P0.x, _x.P0.y, _x.P0.z, _x.n.x, _x.n.y, _x.n.z,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6f().pack(_x.P0.x, _x.P0.y, _x.P0.z, _x.n.x, _x.n.y, _x.n.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.P0 is None:
        self.P0 = geometry_msgs.msg.Point32()
      if self.n is None:
        self.n = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 24
      (_x.P0.x, _x.P0.y, _x.P0.z, _x.n.x, _x.n.y, _x.n.z,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
class FitMinimalPlaneSrv(object):
  _type          = 'KBSL/FitMinimalPlaneSrv'
  _md5sum = '0a7e6160a47703bcae7741304dcb479d'
  _request_class  = FitMinimalPlaneSrvRequest
  _response_class = FitMinimalPlaneSrvResponse