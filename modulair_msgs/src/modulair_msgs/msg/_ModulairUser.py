"""autogenerated by genmsg_py from ModulairUser.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg
import std_msgs.msg

class ModulairUser(roslib.message.Message):
  _md5sum = "65938d00ae8cdff05c33e4117765914a"
  _type = "modulair_msgs/ModulairUser"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# This message contains a openni_msgs/user message with and ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint.
#### 
Header header
uint8 uid
string[] frames
float64[] confs
geometry_msgs/Transform[] transforms
geometry_msgs/Vector3[] projective
geometry_msgs/Vector3[] translation
geometry_msgs/Vector3[] translation_mm
geometry_msgs/Vector3 center_of_mass

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['header','uid','frames','confs','transforms','projective','translation','translation_mm','center_of_mass']
  _slot_types = ['Header','uint8','string[]','float64[]','geometry_msgs/Transform[]','geometry_msgs/Vector3[]','geometry_msgs/Vector3[]','geometry_msgs/Vector3[]','geometry_msgs/Vector3']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,uid,frames,confs,transforms,projective,translation,translation_mm,center_of_mass
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(ModulairUser, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.uid is None:
        self.uid = 0
      if self.frames is None:
        self.frames = []
      if self.confs is None:
        self.confs = []
      if self.transforms is None:
        self.transforms = []
      if self.projective is None:
        self.projective = []
      if self.translation is None:
        self.translation = []
      if self.translation_mm is None:
        self.translation_mm = []
      if self.center_of_mass is None:
        self.center_of_mass = geometry_msgs.msg.Vector3()
    else:
      self.header = std_msgs.msg._Header.Header()
      self.uid = 0
      self.frames = []
      self.confs = []
      self.transforms = []
      self.projective = []
      self.translation = []
      self.translation_mm = []
      self.center_of_mass = geometry_msgs.msg.Vector3()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.uid))
      length = len(self.frames)
      buff.write(_struct_I.pack(length))
      for val1 in self.frames:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.confs)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.confs))
      length = len(self.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.transforms:
        _v1 = val1.translation
        _x = _v1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v2 = val1.rotation
        _x = _v2
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.projective)
      buff.write(_struct_I.pack(length))
      for val1 in self.projective:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      length = len(self.translation)
      buff.write(_struct_I.pack(length))
      for val1 in self.translation:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      length = len(self.translation_mm)
      buff.write(_struct_I.pack(length))
      for val1 in self.translation_mm:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_3d.pack(_x.center_of_mass.x, _x.center_of_mass.y, _x.center_of_mass.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.center_of_mass is None:
        self.center_of_mass = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 1
      (self.uid,) = _struct_B.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.frames = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.frames.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.confs = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v3 = val1.translation
        _x = _v3
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v4 = val1.rotation
        _x = _v4
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.transforms.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.projective = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.projective.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.translation = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.translation.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.translation_mm = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.translation_mm.append(val1)
      _x = self
      start = end
      end += 24
      (_x.center_of_mass.x, _x.center_of_mass.y, _x.center_of_mass.z,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_B.pack(self.uid))
      length = len(self.frames)
      buff.write(_struct_I.pack(length))
      for val1 in self.frames:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.confs)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.confs.tostring())
      length = len(self.transforms)
      buff.write(_struct_I.pack(length))
      for val1 in self.transforms:
        _v5 = val1.translation
        _x = _v5
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v6 = val1.rotation
        _x = _v6
        buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
      length = len(self.projective)
      buff.write(_struct_I.pack(length))
      for val1 in self.projective:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      length = len(self.translation)
      buff.write(_struct_I.pack(length))
      for val1 in self.translation:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      length = len(self.translation_mm)
      buff.write(_struct_I.pack(length))
      for val1 in self.translation_mm:
        _x = val1
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_struct_3d.pack(_x.center_of_mass.x, _x.center_of_mass.y, _x.center_of_mass.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.center_of_mass is None:
        self.center_of_mass = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 1
      (self.uid,) = _struct_B.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.frames = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.frames.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.confs = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.transforms = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Transform()
        _v7 = val1.translation
        _x = _v7
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        _v8 = val1.rotation
        _x = _v8
        start = end
        end += 32
        (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
        self.transforms.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.projective = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.projective.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.translation = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.translation.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.translation_mm = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Vector3()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.translation_mm.append(val1)
      _x = self
      start = end
      end += 24
      (_x.center_of_mass.x, _x.center_of_mass.y, _x.center_of_mass.z,) = _struct_3d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_B = struct.Struct("<B")
_struct_4d = struct.Struct("<4d")
_struct_3d = struct.Struct("<3d")
