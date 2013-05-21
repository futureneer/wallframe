"""autogenerated by genmsg_py from ModulairUserArray.msg. Do not edit."""
import roslib.message
import struct

import modulair_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class ModulairUserArray(roslib.message.Message):
  _md5sum = "d81bc6820d5b040b9ab15a79a118b478"
  _type = "modulair_msgs/ModulairUserArray"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This message contains a vector of openni_msgs/User messages.
modulair_msgs/ModulairUser[] users
int8 numUsers

================================================================================
MSG: modulair_msgs/ModulairUser
# This message contains a openni_msgs/user message with and ID, a vector of frames corresponding to each joint, a vector of confidences corresponding to each joint, and a vector of geometry_msgs/Transform messages corresponding to each joint.
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
  __slots__ = ['users','numUsers']
  _slot_types = ['modulair_msgs/ModulairUser[]','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       users,numUsers
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(ModulairUserArray, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.users is None:
        self.users = []
      if self.numUsers is None:
        self.numUsers = 0
    else:
      self.users = []
      self.numUsers = 0

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
      length = len(self.users)
      buff.write(_struct_I.pack(length))
      for val1 in self.users:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_B.pack(val1.uid))
        length = len(val1.frames)
        buff.write(_struct_I.pack(length))
        for val2 in val1.frames:
          length = len(val2)
          buff.write(struct.pack('<I%ss'%length, length, val2))
        length = len(val1.confs)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(struct.pack(pattern, *val1.confs))
        length = len(val1.transforms)
        buff.write(_struct_I.pack(length))
        for val2 in val1.transforms:
          _v3 = val2.translation
          _x = _v3
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v4 = val2.rotation
          _x = _v4
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.projective)
        buff.write(_struct_I.pack(length))
        for val2 in val1.projective:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.translation)
        buff.write(_struct_I.pack(length))
        for val2 in val1.translation:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.translation_mm)
        buff.write(_struct_I.pack(length))
        for val2 in val1.translation_mm:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v5 = val1.center_of_mass
        _x = _v5
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      buff.write(_struct_b.pack(self.numUsers))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.users = []
      for i in range(0, length):
        val1 = modulair_msgs.msg.ModulairUser()
        _v6 = val1.header
        start = end
        end += 4
        (_v6.seq,) = _struct_I.unpack(str[start:end])
        _v7 = _v6.stamp
        _x = _v7
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v6.frame_id = str[start:end]
        start = end
        end += 1
        (val1.uid,) = _struct_B.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.frames = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          val2 = str[start:end]
          val1.frames.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        val1.confs = struct.unpack(pattern, str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.transforms = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Transform()
          _v8 = val2.translation
          _x = _v8
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v9 = val2.rotation
          _x = _v9
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          val1.transforms.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.projective = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.projective.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.translation = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.translation.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.translation_mm = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.translation_mm.append(val2)
        _v10 = val1.center_of_mass
        _x = _v10
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.users.append(val1)
      start = end
      end += 1
      (self.numUsers,) = _struct_b.unpack(str[start:end])
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
      length = len(self.users)
      buff.write(_struct_I.pack(length))
      for val1 in self.users:
        _v11 = val1.header
        buff.write(_struct_I.pack(_v11.seq))
        _v12 = _v11.stamp
        _x = _v12
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v11.frame_id
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        buff.write(_struct_B.pack(val1.uid))
        length = len(val1.frames)
        buff.write(_struct_I.pack(length))
        for val2 in val1.frames:
          length = len(val2)
          buff.write(struct.pack('<I%ss'%length, length, val2))
        length = len(val1.confs)
        buff.write(_struct_I.pack(length))
        pattern = '<%sd'%length
        buff.write(val1.confs.tostring())
        length = len(val1.transforms)
        buff.write(_struct_I.pack(length))
        for val2 in val1.transforms:
          _v13 = val2.translation
          _x = _v13
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
          _v14 = val2.rotation
          _x = _v14
          buff.write(_struct_4d.pack(_x.x, _x.y, _x.z, _x.w))
        length = len(val1.projective)
        buff.write(_struct_I.pack(length))
        for val2 in val1.projective:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.translation)
        buff.write(_struct_I.pack(length))
        for val2 in val1.translation:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        length = len(val1.translation_mm)
        buff.write(_struct_I.pack(length))
        for val2 in val1.translation_mm:
          _x = val2
          buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
        _v15 = val1.center_of_mass
        _x = _v15
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.z))
      buff.write(_struct_b.pack(self.numUsers))
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
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.users = []
      for i in range(0, length):
        val1 = modulair_msgs.msg.ModulairUser()
        _v16 = val1.header
        start = end
        end += 4
        (_v16.seq,) = _struct_I.unpack(str[start:end])
        _v17 = _v16.stamp
        _x = _v17
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v16.frame_id = str[start:end]
        start = end
        end += 1
        (val1.uid,) = _struct_B.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.frames = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          val2 = str[start:end]
          val1.frames.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        pattern = '<%sd'%length
        start = end
        end += struct.calcsize(pattern)
        val1.confs = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.transforms = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Transform()
          _v18 = val2.translation
          _x = _v18
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          _v19 = val2.rotation
          _x = _v19
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _struct_4d.unpack(str[start:end])
          val1.transforms.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.projective = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.projective.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.translation = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.translation.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.translation_mm = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.Vector3()
          _x = val2
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
          val1.translation_mm.append(val2)
        _v20 = val1.center_of_mass
        _x = _v20
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _struct_3d.unpack(str[start:end])
        self.users.append(val1)
      start = end
      end += 1
      (self.numUsers,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
_struct_b = struct.Struct("<b")
_struct_4d = struct.Struct("<4d")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
