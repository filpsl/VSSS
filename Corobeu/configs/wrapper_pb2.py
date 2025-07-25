# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: wrapper.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import configs.messages_robocup_ssl_detection_pb2 as messages__robocup__ssl__detection__pb2
import configs.messages_robocup_ssl_geometry_pb2 as messages__robocup__ssl__geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='wrapper.proto',
  package='',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\rwrapper.proto\x1a$messages_robocup_ssl_detection.proto\x1a#messages_robocup_ssl_geometry.proto\"`\n\x11SSL_WrapperPacket\x12&\n\tdetection\x18\x01 \x01(\x0b\x32\x13.SSL_DetectionFrame\x12#\n\x08geometry\x18\x02 \x01(\x0b\x32\x11.SSL_GeometryData')
  ,
  dependencies=[messages__robocup__ssl__detection__pb2.DESCRIPTOR,messages__robocup__ssl__geometry__pb2.DESCRIPTOR,])




_SSL_WRAPPERPACKET = _descriptor.Descriptor(
  name='SSL_WrapperPacket',
  full_name='SSL_WrapperPacket',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='detection', full_name='SSL_WrapperPacket.detection', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='geometry', full_name='SSL_WrapperPacket.geometry', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=92,
  serialized_end=188,
)

_SSL_WRAPPERPACKET.fields_by_name['detection'].message_type = messages__robocup__ssl__detection__pb2._SSL_DETECTIONFRAME
_SSL_WRAPPERPACKET.fields_by_name['geometry'].message_type = messages__robocup__ssl__geometry__pb2._SSL_GEOMETRYDATA
DESCRIPTOR.message_types_by_name['SSL_WrapperPacket'] = _SSL_WRAPPERPACKET
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SSL_WrapperPacket = _reflection.GeneratedProtocolMessageType('SSL_WrapperPacket', (_message.Message,), dict(
  DESCRIPTOR = _SSL_WRAPPERPACKET,
  __module__ = 'wrapper_pb2'
  # @@protoc_insertion_point(class_scope:SSL_WrapperPacket)
  ))
_sym_db.RegisterMessage(SSL_WrapperPacket)


# @@protoc_insertion_point(module_scope)
