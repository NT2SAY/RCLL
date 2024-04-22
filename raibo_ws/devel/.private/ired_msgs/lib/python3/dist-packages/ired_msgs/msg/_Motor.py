# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ired_msgs/Motor.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Motor(genpy.Message):
  _md5sum = "ec2526e9a146e07c0941072e0ef16c7b"
  _type = "ired_msgs/Motor"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64[4] speed_sp
float64[4] speed_fb
float64[3] pid_motor_front_left
float64[3] pid_motor_front_right
float64[3] pid_motor_rear_left
float64[3] pid_motor_rear_right"""
  __slots__ = ['speed_sp','speed_fb','pid_motor_front_left','pid_motor_front_right','pid_motor_rear_left','pid_motor_rear_right']
  _slot_types = ['float64[4]','float64[4]','float64[3]','float64[3]','float64[3]','float64[3]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       speed_sp,speed_fb,pid_motor_front_left,pid_motor_front_right,pid_motor_rear_left,pid_motor_rear_right

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Motor, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.speed_sp is None:
        self.speed_sp = [0.] * 4
      if self.speed_fb is None:
        self.speed_fb = [0.] * 4
      if self.pid_motor_front_left is None:
        self.pid_motor_front_left = [0.] * 3
      if self.pid_motor_front_right is None:
        self.pid_motor_front_right = [0.] * 3
      if self.pid_motor_rear_left is None:
        self.pid_motor_rear_left = [0.] * 3
      if self.pid_motor_rear_right is None:
        self.pid_motor_rear_right = [0.] * 3
    else:
      self.speed_sp = [0.] * 4
      self.speed_fb = [0.] * 4
      self.pid_motor_front_left = [0.] * 3
      self.pid_motor_front_right = [0.] * 3
      self.pid_motor_rear_left = [0.] * 3
      self.pid_motor_rear_right = [0.] * 3

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
      buff.write(_get_struct_4d().pack(*self.speed_sp))
      buff.write(_get_struct_4d().pack(*self.speed_fb))
      buff.write(_get_struct_3d().pack(*self.pid_motor_front_left))
      buff.write(_get_struct_3d().pack(*self.pid_motor_front_right))
      buff.write(_get_struct_3d().pack(*self.pid_motor_rear_left))
      buff.write(_get_struct_3d().pack(*self.pid_motor_rear_right))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 32
      self.speed_sp = _get_struct_4d().unpack(str[start:end])
      start = end
      end += 32
      self.speed_fb = _get_struct_4d().unpack(str[start:end])
      start = end
      end += 24
      self.pid_motor_front_left = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 24
      self.pid_motor_front_right = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 24
      self.pid_motor_rear_left = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 24
      self.pid_motor_rear_right = _get_struct_3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(self.speed_sp.tostring())
      buff.write(self.speed_fb.tostring())
      buff.write(self.pid_motor_front_left.tostring())
      buff.write(self.pid_motor_front_right.tostring())
      buff.write(self.pid_motor_rear_left.tostring())
      buff.write(self.pid_motor_rear_right.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 32
      self.speed_sp = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      start = end
      end += 32
      self.speed_fb = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=4)
      start = end
      end += 24
      self.pid_motor_front_left = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.pid_motor_front_right = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.pid_motor_rear_left = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.pid_motor_rear_right = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d