# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from amrl_msgs/ErrorReport.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class ErrorReport(genpy.Message):
  _md5sum = "9898087bf4de62612995185ab9cc18ca"
  _type = "amrl_msgs/ErrorReport"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header

# Header from the laser scan that was received closest to the creation of this report
Header laser_header

# Severity level enum
uint8 INFO=0 # Information
uint8 SUBOPTIMAL=1 # Suboptimal but safe performance
uint8 RISKY=2 # risky, potentially unsafe (got away with it, but could have been catastrophic)
uint8 CATASTROPHIC=3 # catastrohpic (e.g. hit an obstacle)

# Subsystem enum
uint8 OTHER=0 # for forward compatibility -- details can be added to the free-text field or a new type can be added if we find there is demand
uint8 LOCALIZATION=1 # robot is mislocalized
uint8 NAVIGATION_PERCEPTION=2 # perception failure, e.g. it mis-classified grass as traversible
uint8 NAVIGATION_LOCAL_PLANNING=3 # problems with local planning
uint8 NAVIGATION_GLOBAL_PLANNING=4 # problems with global planning

# Should be one of the above severity levels
uint8 severity_level

# Should be one of the above subsystems
uint8 failed_subsystem

# Detailed text providing details about the error
string detailed_error_msg
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  # Pseudo-constants
  INFO = 0
  SUBOPTIMAL = 1
  RISKY = 2
  CATASTROPHIC = 3
  OTHER = 0
  LOCALIZATION = 1
  NAVIGATION_PERCEPTION = 2
  NAVIGATION_LOCAL_PLANNING = 3
  NAVIGATION_GLOBAL_PLANNING = 4

  __slots__ = ['header','laser_header','severity_level','failed_subsystem','detailed_error_msg']
  _slot_types = ['std_msgs/Header','std_msgs/Header','uint8','uint8','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,laser_header,severity_level,failed_subsystem,detailed_error_msg

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ErrorReport, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.laser_header is None:
        self.laser_header = std_msgs.msg.Header()
      if self.severity_level is None:
        self.severity_level = 0
      if self.failed_subsystem is None:
        self.failed_subsystem = 0
      if self.detailed_error_msg is None:
        self.detailed_error_msg = ''
    else:
      self.header = std_msgs.msg.Header()
      self.laser_header = std_msgs.msg.Header()
      self.severity_level = 0
      self.failed_subsystem = 0
      self.detailed_error_msg = ''

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.laser_header.seq, _x.laser_header.stamp.secs, _x.laser_header.stamp.nsecs))
      _x = self.laser_header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.severity_level, _x.failed_subsystem))
      _x = self.detailed_error_msg
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.laser_header is None:
        self.laser_header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.laser_header.seq, _x.laser_header.stamp.secs, _x.laser_header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.laser_header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.laser_header.frame_id = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.severity_level, _x.failed_subsystem,) = _get_struct_2B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.detailed_error_msg = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.detailed_error_msg = str[start:end]
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
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3I().pack(_x.laser_header.seq, _x.laser_header.stamp.secs, _x.laser_header.stamp.nsecs))
      _x = self.laser_header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2B().pack(_x.severity_level, _x.failed_subsystem))
      _x = self.detailed_error_msg
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.laser_header is None:
        self.laser_header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.laser_header.seq, _x.laser_header.stamp.secs, _x.laser_header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.laser_header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.laser_header.frame_id = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.severity_level, _x.failed_subsystem,) = _get_struct_2B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.detailed_error_msg = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.detailed_error_msg = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
