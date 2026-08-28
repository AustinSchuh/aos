#ifndef AOS_PROTOBUF_FLATBUFFER_TO_PROTO_H_
#define AOS_PROTOBUF_FLATBUFFER_TO_PROTO_H_

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "flatbuffers/reflection.h"

namespace aos {

// Encodes a flatbuffer as protobuf wire format, driven entirely by the
// flatbuffer's reflection schema.  No generated code, and nothing to write per
// message type.
//
// Encoding allocates nothing.  Everything that needs memory happens in the
// constructor; Encode() writes into a buffer the caller owns and walks the
// message on the stack.  NetworkTables itself still allocates on every publish
// (NetworkTableValue::MakeRaw copies into a shared vector), so this being
// clean does not make the whole path realtime -- it just keeps the translator
// from being a second reason it is not.
//
// # Matching an existing .proto
//
// The mapping is mechanical, so a flatbuffer produces the protobuf its schema
// describes -- which is only the protobuf you already have if the .fbs was
// written to mirror it:
//
//   * A field's protobuf number is its flatbuffer id plus one, because
//     protobuf numbers from 1 and flatbuffers from 0.  Where the proto has a
//     gap -- what deleting a field leaves behind -- the .fbs reserves the
//     matching id with a deprecated field:
//
//       reserved_0:byte (id: 0, deprecated);  // proto field 1, deleted
//
//     Deprecated fields cost a vtable slot and no wire bytes, and reflection
//     still carries them, so this keeps every live field lined up without any
//     of them having to say so.
//   * Anything protobuf packs that flatbuffers does not -- a bitfield word, a
//     nibble-packed array -- has to be a single scalar field on the flatbuffer
//     side too.  Reflection cannot invent that packing.
//   * protobuf has wire types flatbuffers has no notion of.  A `proto_type`
//     attribute picks one:
//
//       axes:[short] (id: 4, proto_type: "sint32");
//
//     Accepted values are sint32, sint64 (zigzag varint), fixed32, fixed64,
//     sfixed32 and sfixed64 (fixed width).  Without it, integers are plain
//     varints, floats are fixed32, and doubles are fixed64.
//
// Scalars equal to their default are omitted, matching proto3's implicit
// presence.  Repeated scalars are packed, which is also proto3's default.
class FlatbufferToProto {
 public:
  // The schema must outlive this, and is walked here rather than at encode
  // time.  Dies if the schema contains something with no protobuf equivalent,
  // rather than silently dropping it.
  explicit FlatbufferToProto(const reflection::Schema *schema);

  // The schema being translated.
  const reflection::Schema *schema() const { return schema_; }

  // An object's fields, in protobuf field-number order.  Built at
  // construction, so reading it allocates nothing.
  const std::vector<const reflection::Field *> &FieldsById(
      const reflection::Object &object) const {
    return fields_by_id_.at(&object);
  }

  // Bytes Encode() would write.  Allocates nothing.
  size_t EncodedSize(const uint8_t *flatbuffer) const;

  // Writes the message into `buffer`, which must be at least EncodedSize()
  // bytes.  Returns how many bytes were written, or 0 if the buffer was too
  // small.  Allocates nothing.
  size_t Encode(const uint8_t *flatbuffer, std::span<uint8_t> buffer) const;

 private:
  // reflection::Object::fields is sorted by name, not by id, so walking it
  // directly would emit protobuf fields in whatever order their names happen
  // to fall in.  Valid, since a decoder does not care, but not what anyone
  // reading the bytes expects.  Sorted once here rather than per message,
  // which is also what keeps encoding allocation-free.
  absl::flat_hash_map<const reflection::Object *,
                      std::vector<const reflection::Field *>>
      fields_by_id_;

  const reflection::Schema *const schema_;
};

// The protobuf full name of a schema's root table, which is its flatbuffer
// name unchanged -- both spell a namespaced name with dots.  This is what goes
// in a NetworkTables topic's type string, after "proto:".
std::string_view ProtoMessageName(const reflection::Schema *schema);

// Builds the FileDescriptorProto describing `schema`, serialized.
//
// This is what NetworkTables wants in its schema registry: AddSchema() with a
// type of "proto:FileDescriptorProto", under the name "proto:<file_name>".
// With it published, a client that has never heard of the type can decode the
// messages; without it they are opaque bytes.
//
// Allocates, and is meant to be called once per schema at startup.  Every
// object in the schema has to share the root table's namespace, since a
// FileDescriptorProto has exactly one package.
std::vector<uint8_t> FileDescriptorProtoForSchema(
    const reflection::Schema *schema, std::string_view file_name);

}  // namespace aos

#endif  // AOS_PROTOBUF_FLATBUFFER_TO_PROTO_H_
