#include "aos/protobuf/flatbuffer_to_proto.h"

#include <algorithm>
#include <cstring>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/die_if_null.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/descriptor.pb.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/wire_format_lite.h"

namespace aos {
namespace {

namespace pb = ::google::protobuf;
namespace pbio = ::google::protobuf::io;
using WireFormatLite = pb::internal::WireFormatLite;

// flatbuffers has IsInteger, IsFloat and IsLong, but nothing for signedness,
// so this is the one predicate left to spell out.
bool IsSigned(reflection::BaseType type) {
  return type == reflection::BaseType::Byte ||
         type == reflection::BaseType::Short ||
         type == reflection::BaseType::Int ||
         type == reflection::BaseType::Long;
}

// The protobuf type a field encodes as: its `proto_type` attribute, or the
// default for its flatbuffer type.  Dies on an unrecognized attribute value
// rather than quietly encoding it as something else.
//
// This is the only type decision made here, and everything else is derived
// from it -- the wire type by WireTypeFor() below, the bytes by WriteScalar(),
// and the descriptor by FillField().  They have to agree: a decoder told
// sint32 and handed a plain varint reads a different number than the one that
// was written, and a second enum beside this one is a second place for them to
// disagree.
// The `proto_type` attribute's value, or empty if the field has none.
std::string_view ProtoTypeAttribute(const reflection::Field &field) {
  if (field.attributes() == nullptr) {
    return {};
  }
  const reflection::KeyValue *const attribute =
      field.attributes()->LookupByKey("proto_type");
  if (attribute == nullptr || attribute->value() == nullptr) {
    return {};
  }
  return attribute->value()->string_view();
}

pb::FieldDescriptorProto::Type ProtoTypeFor(const reflection::Field &field) {
  const reflection::BaseType type =
      field.type()->base_type() == reflection::BaseType::Vector
          ? field.type()->element()
          : field.type()->base_type();
  const std::string_view attribute = ProtoTypeAttribute(field);

  // Only an integer has more than one protobuf type it could be.  Everything
  // else has exactly one, so an attribute on it is not a choice being made --
  // it is a claim the descriptor is about to contradict, and saying so beats
  // ignoring it and encoding something the schema's author did not ask for.
  switch (type) {
    case reflection::BaseType::String:
    case reflection::BaseType::Obj:
    case reflection::BaseType::Bool:
    case reflection::BaseType::Float:
    case reflection::BaseType::Double:
      ABSL_CHECK(attribute.empty())
          << ": Field " << field.name()->string_view() << " has proto_type \""
          << attribute
          << "\", but its flatbuffer type has exactly one protobuf type, so "
             "there is nothing for the attribute to pick.  Drop it.";
      break;
    default:
      break;
  }

  switch (type) {
    case reflection::BaseType::String:
      return pb::FieldDescriptorProto::TYPE_STRING;
    case reflection::BaseType::Obj:
      return pb::FieldDescriptorProto::TYPE_MESSAGE;
    case reflection::BaseType::Bool:
      return pb::FieldDescriptorProto::TYPE_BOOL;
    case reflection::BaseType::Float:
      return pb::FieldDescriptorProto::TYPE_FLOAT;
    case reflection::BaseType::Double:
      return pb::FieldDescriptorProto::TYPE_DOUBLE;
    default:
      break;
  }

  ABSL_CHECK(flatbuffers::IsInteger(type))
      << ": Field " << field.name()->string_view()
      << " has a type with no protobuf equivalent";

  if (!attribute.empty()) {
    // The width comes from the flatbuffer field rather than from the name,
    // so that sint32 on a long field is a sint64 rather than a declared type
    // the value does not fit in.
    if (attribute == "sint32" || attribute == "sint64") {
      return flatbuffers::IsLong(type) ? pb::FieldDescriptorProto::TYPE_SINT64
                                       : pb::FieldDescriptorProto::TYPE_SINT32;
    }
    if (attribute == "fixed32" || attribute == "sfixed32") {
      return IsSigned(type) ? pb::FieldDescriptorProto::TYPE_SFIXED32
                            : pb::FieldDescriptorProto::TYPE_FIXED32;
    }
    if (attribute == "fixed64" || attribute == "sfixed64") {
      return IsSigned(type) ? pb::FieldDescriptorProto::TYPE_SFIXED64
                            : pb::FieldDescriptorProto::TYPE_FIXED64;
    }
    ABSL_LOG(FATAL)
        << ": Field " << field.name()->string_view() << " has proto_type \""
        << attribute
        << "\", which is not a protobuf wire type this understands.  Use "
           "sint32, sint64, fixed32, fixed64, sfixed32 or sfixed64, or drop "
           "the attribute to get a plain varint.";
  }

  if (flatbuffers::IsLong(type)) {
    return IsSigned(type) ? pb::FieldDescriptorProto::TYPE_INT64
                          : pb::FieldDescriptorProto::TYPE_UINT64;
  }
  return IsSigned(type) ? pb::FieldDescriptorProto::TYPE_INT32
                        : pb::FieldDescriptorProto::TYPE_UINT32;
}

// protobuf numbers WireFormatLite::FieldType to match
// FieldDescriptorProto::Type, and does this same cast internally in
// wire_format.cc.  Spot-checked at both ends so a renumbering does not go
// unnoticed.
static_assert(static_cast<int>(pb::FieldDescriptorProto::TYPE_DOUBLE) ==
              static_cast<int>(WireFormatLite::TYPE_DOUBLE));
static_assert(static_cast<int>(pb::FieldDescriptorProto::TYPE_SINT64) ==
              static_cast<int>(WireFormatLite::TYPE_SINT64));

WireFormatLite::WireType WireTypeFor(pb::FieldDescriptorProto::Type type) {
  return WireFormatLite::WireTypeForFieldType(
      static_cast<WireFormatLite::FieldType>(type));
}

// A cursor that either counts bytes or writes them, so sizing and writing walk
// the message with one body of code rather than two that can disagree about a
// length.  Both halves defer to protobuf's own primitives -- CodedOutputStream
// to write, its VarintSize helpers to measure -- so nothing here reimplements
// the wire format.
class Cursor {
 public:
  // Counting.
  Cursor() = default;
  // Writing.
  explicit Cursor(pbio::CodedOutputStream *output) : output_(output) {}

  size_t size() const { return size_; }

  void Varint(uint64_t value) {
    if (output_ != nullptr) {
      output_->WriteVarint64(value);
    }
    size_ += pbio::CodedOutputStream::VarintSize64(value);
  }

  void Tag(uint32_t field_number, WireFormatLite::WireType wire_type) {
    Varint(WireFormatLite::MakeTag(static_cast<int>(field_number), wire_type));
  }

  void Fixed32(uint32_t value) {
    if (output_ != nullptr) {
      output_->WriteLittleEndian32(value);
    }
    size_ += 4;
  }

  void Fixed64(uint64_t value) {
    if (output_ != nullptr) {
      output_->WriteLittleEndian64(value);
    }
    size_ += 8;
  }

  void Bytes(const uint8_t *data, size_t size) {
    if (output_ != nullptr) {
      output_->WriteRaw(data, static_cast<int>(size));
    }
    size_ += size;
  }

 private:
  pbio::CodedOutputStream *output_ = nullptr;
  size_t size_ = 0;
};

// The protobuf field number for a field.  protobuf numbers from 1 and
// flatbuffers from 0, so they are the same sequence offset by one; what that
// asks of a schema mirroring an existing .proto is in flatbuffer_to_proto.h.
uint32_t ProtoFieldNumber(const reflection::Field &field) {
  return field.id() + 1;
}

// Writes one scalar's payload, given the value already read out of the
// flatbuffer.  Whether to zigzag, and whether the bits are a float's, is the
// protobuf type's to say -- which is why this takes it rather than the
// flatbuffer type beside it.
void WriteScalar(Cursor *writer, pb::FieldDescriptorProto::Type type,
                 int64_t integer, double real) {
  switch (type) {
    case pb::FieldDescriptorProto::TYPE_SINT32:
    case pb::FieldDescriptorProto::TYPE_SINT64:
      writer->Varint(WireFormatLite::ZigZagEncode64(integer));
      return;
    case pb::FieldDescriptorProto::TYPE_INT32:
    case pb::FieldDescriptorProto::TYPE_INT64:
    case pb::FieldDescriptorProto::TYPE_UINT32:
    case pb::FieldDescriptorProto::TYPE_UINT64:
    case pb::FieldDescriptorProto::TYPE_BOOL:
    case pb::FieldDescriptorProto::TYPE_ENUM:
      writer->Varint(static_cast<uint64_t>(integer));
      return;
    case pb::FieldDescriptorProto::TYPE_FLOAT: {
      const float value = static_cast<float>(real);
      uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      writer->Fixed32(bits);
      return;
    }
    case pb::FieldDescriptorProto::TYPE_FIXED32:
    case pb::FieldDescriptorProto::TYPE_SFIXED32:
      writer->Fixed32(static_cast<uint32_t>(integer));
      return;
    case pb::FieldDescriptorProto::TYPE_DOUBLE: {
      const double value = real;
      uint64_t bits;
      std::memcpy(&bits, &value, sizeof(bits));
      writer->Fixed64(bits);
      return;
    }
    case pb::FieldDescriptorProto::TYPE_FIXED64:
    case pb::FieldDescriptorProto::TYPE_SFIXED64:
      writer->Fixed64(static_cast<uint64_t>(integer));
      return;
    default:
      ABSL_LOG(FATAL) << ": Not a scalar: "
                      << pb::FieldDescriptorProto::Type_Name(type);
  }
}

size_t ScalarSize(pb::FieldDescriptorProto::Type type, int64_t integer,
                  double real) {
  Cursor counter;
  WriteScalar(&counter, type, integer, real);
  return counter.size();
}

// Forward declaration: tables nest.
void WriteTable(Cursor *writer, const FlatbufferToProto &translator,
                const reflection::Object &object,
                const flatbuffers::Table &table);

size_t TableSize(const FlatbufferToProto &translator,
                 const reflection::Object &object,
                 const flatbuffers::Table &table) {
  Cursor counter;
  WriteTable(&counter, translator, object, table);
  return counter.size();
}

void WriteStruct(Cursor *writer, const FlatbufferToProto &translator,
                 const reflection::Object &object,
                 const flatbuffers::Struct &value);

size_t StructSize(const FlatbufferToProto &translator,
                  const reflection::Object &object,
                  const flatbuffers::Struct &value) {
  Cursor counter;
  WriteStruct(&counter, translator, object, value);
  return counter.size();
}

void WriteStruct(Cursor *writer, const FlatbufferToProto &translator,
                 const reflection::Object &object,
                 const flatbuffers::Struct &value) {
  const reflection::Schema *const schema = translator.schema();
  for (const reflection::Field *field : translator.FieldsById(object)) {
    const uint32_t number = ProtoFieldNumber(*field);
    const reflection::BaseType type = field->type()->base_type();

    if (type == reflection::BaseType::Obj) {
      // A struct inside a struct is always present.
      const reflection::Object &nested =
          *schema->objects()->Get(field->type()->index());
      const flatbuffers::Struct *const inner =
          flatbuffers::GetFieldStruct(value, *field);
      writer->Tag(number, WireFormatLite::WIRETYPE_LENGTH_DELIMITED);
      writer->Varint(StructSize(translator, nested, *inner));
      WriteStruct(writer, translator, nested, *inner);
      continue;
    }

    const pb::FieldDescriptorProto::Type proto_type = ProtoTypeFor(*field);
    const int64_t integer = flatbuffers::GetAnyFieldI(value, *field);
    const double real = flatbuffers::GetAnyFieldF(value, *field);
    // Struct fields are always present, so unlike a table there is no default
    // to skip: a zero here is a real zero.
    writer->Tag(number, WireTypeFor(proto_type));
    WriteScalar(writer, proto_type, integer, real);
  }
}

void WriteTable(Cursor *writer, const FlatbufferToProto &translator,
                const reflection::Object &object,
                const flatbuffers::Table &table) {
  const reflection::Schema *const schema = translator.schema();
  for (const reflection::Field *field : translator.FieldsById(object)) {
    if (field->deprecated()) {
      continue;
    }
    const uint32_t number = ProtoFieldNumber(*field);
    const reflection::BaseType type = field->type()->base_type();

    switch (type) {
      case reflection::BaseType::String: {
        const flatbuffers::String *const value =
            flatbuffers::GetFieldS(table, *field);
        if (value == nullptr || value->size() == 0) {
          // proto3 omits an empty string.
          break;
        }
        writer->Tag(number, WireFormatLite::WIRETYPE_LENGTH_DELIMITED);
        writer->Varint(value->size());
        writer->Bytes(reinterpret_cast<const uint8_t *>(value->data()),
                      value->size());
        break;
      }

      case reflection::BaseType::Obj: {
        const reflection::Object &nested =
            *schema->objects()->Get(field->type()->index());
        if (nested.is_struct()) {
          const flatbuffers::Struct *const value =
              flatbuffers::GetFieldStruct(table, *field);
          if (value == nullptr) {
            break;
          }
          writer->Tag(number, WireFormatLite::WIRETYPE_LENGTH_DELIMITED);
          writer->Varint(StructSize(translator, nested, *value));
          WriteStruct(writer, translator, nested, *value);
          break;
        }
        const flatbuffers::Table *const value =
            flatbuffers::GetFieldT(table, *field);
        if (value == nullptr) {
          break;
        }
        writer->Tag(number, WireFormatLite::WIRETYPE_LENGTH_DELIMITED);
        writer->Varint(TableSize(translator, nested, *value));
        WriteTable(writer, translator, nested, *value);
        break;
      }

      case reflection::BaseType::Vector: {
        const flatbuffers::VectorOfAny *const vector =
            flatbuffers::GetFieldAnyV(table, *field);
        if (vector == nullptr || vector->size() == 0) {
          break;
        }
        const reflection::BaseType element = field->type()->element();

        if (element == reflection::BaseType::String) {
          const auto *const strings =
              reinterpret_cast<const flatbuffers::Vector<
                  flatbuffers::Offset<flatbuffers::String>> *>(vector);
          for (const flatbuffers::String *value : *strings) {
            writer->Tag(number, WireFormatLite::WIRETYPE_LENGTH_DELIMITED);
            writer->Varint(value->size());
            writer->Bytes(reinterpret_cast<const uint8_t *>(value->data()),
                          value->size());
          }
          break;
        }

        if (element == reflection::BaseType::Obj) {
          const reflection::Object &nested =
              *schema->objects()->Get(field->type()->index());
          for (size_t i = 0; i < vector->size(); ++i) {
            writer->Tag(number, WireFormatLite::WIRETYPE_LENGTH_DELIMITED);
            if (nested.is_struct()) {
              const auto *const value = flatbuffers::GetAnyVectorElemAddressOf<
                  const flatbuffers::Struct>(vector, i, nested.bytesize());
              writer->Varint(StructSize(translator, nested, *value));
              WriteStruct(writer, translator, nested, *value);
            } else {
              const flatbuffers::Table *const value =
                  flatbuffers::GetAnyVectorElemPointer<
                      const flatbuffers::Table>(vector, i);
              writer->Varint(TableSize(translator, nested, *value));
              WriteTable(writer, translator, nested, *value);
            }
          }
          break;
        }

        // Scalars, packed -- which is proto3's default for repeated scalars.
        const pb::FieldDescriptorProto::Type proto_type = ProtoTypeFor(*field);
        size_t payload = 0;
        for (size_t i = 0; i < vector->size(); ++i) {
          payload += ScalarSize(
              proto_type, flatbuffers::GetAnyVectorElemI(vector, element, i),
              flatbuffers::GetAnyVectorElemF(vector, element, i));
        }
        writer->Tag(number, WireFormatLite::WIRETYPE_LENGTH_DELIMITED);
        writer->Varint(payload);
        for (size_t i = 0; i < vector->size(); ++i) {
          WriteScalar(writer, proto_type,
                      flatbuffers::GetAnyVectorElemI(vector, element, i),
                      flatbuffers::GetAnyVectorElemF(vector, element, i));
        }
        break;
      }

      case reflection::BaseType::Union:
        ABSL_LOG(FATAL)
            << ": Field " << field->name()->string_view()
            << " is a union.  protobuf's nearest equivalent is a oneof, which "
               "needs a mapping this does not have yet.";
        break;

      default: {
        // A scalar.  proto3 has implicit presence, so a value equal to the
        // default is left off the wire -- which is also what the flatbuffer
        // means by not having it in the vtable.
        const pb::FieldDescriptorProto::Type proto_type = ProtoTypeFor(*field);
        const int64_t integer = flatbuffers::GetAnyFieldI(table, *field);
        const double real = flatbuffers::GetAnyFieldF(table, *field);
        if (flatbuffers::IsFloat(type)) {
          if (real == field->default_real()) {
            break;
          }
        } else if (integer == field->default_integer()) {
          break;
        }
        writer->Tag(number, WireTypeFor(proto_type));
        WriteScalar(writer, proto_type, integer, real);
        break;
      }
    }
  }
}

}  // namespace

FlatbufferToProto::FlatbufferToProto(const reflection::Schema *schema)
    : schema_(ABSL_DIE_IF_NULL(schema)) {
  ABSL_CHECK(schema_->root_table() != nullptr)
      << ": Schema has no root table, so there is nothing to encode.";
  // Walking the schema here means a type protobuf cannot express is found at
  // construction rather than on the first message, and it is where the field
  // ordering gets built.
  //
  // root_table() is walked as well as objects(): in a schema fresh out of
  // flatc it aliases one of the objects() entries, but a schema that has been
  // through a reflection-level copy -- a channel schema inside a flattened
  // config, say -- can carry a duplicate table instead, and encoding starts
  // at root_table(), looked up by pointer.
  std::vector<const reflection::Object *> objects(schema_->objects()->begin(),
                                                  schema_->objects()->end());
  objects.push_back(schema_->root_table());
  for (const reflection::Object *object : objects) {
    std::vector<const reflection::Field *> &sorted = fields_by_id_[object];
    sorted.assign(object->fields()->begin(), object->fields()->end());
    std::sort(sorted.begin(), sorted.end(),
              [](const reflection::Field *a, const reflection::Field *b) {
                return a->id() < b->id();
              });

    for (const reflection::Field *field : *object->fields()) {
      if (field->deprecated()) {
        continue;
      }
      ABSL_CHECK(field->type()->base_type() != reflection::BaseType::Union)
          << ": Field " << field->name()->string_view() << " of "
          << object->name()->string_view()
          << " is a union, which has no protobuf equivalent here.";
      // Validates the proto_type attribute at construction rather than at the
      // first encode: an unknown value, or one on a field with nothing to
      // pick, is a schema bug and should be found by whoever builds the
      // translator.  Message fields are checked too -- an attribute there has
      // nothing to pick either.
      ProtoTypeFor(*field);
    }
  }
}

namespace {

// The part of a flatbuffer name after the last dot, which is the protobuf
// message name; everything before it is the package.
std::string_view LastComponent(std::string_view name) {
  const size_t dot = name.rfind('.');
  return dot == std::string_view::npos ? name : name.substr(dot + 1);
}

std::string_view Namespace(std::string_view name) {
  const size_t dot = name.rfind('.');
  return dot == std::string_view::npos ? std::string_view()
                                       : name.substr(0, dot);
}

void FillField(const reflection::Schema *schema, const reflection::Field &field,
               pb::FieldDescriptorProto *out) {
  out->set_name(std::string(field.name()->string_view()));
  out->set_number(static_cast<int>(ProtoFieldNumber(field)));
  out->set_label(field.type()->base_type() == reflection::BaseType::Vector
                     ? pb::FieldDescriptorProto::LABEL_REPEATED
                     : pb::FieldDescriptorProto::LABEL_OPTIONAL);
  out->set_type(ProtoTypeFor(field));

  if (field.type()->base_type() == reflection::BaseType::Obj ||
      field.type()->element() == reflection::BaseType::Obj) {
    // Fully qualified, which protobuf spells with a leading dot.
    out->set_type_name(absl::StrCat(
        ".",
        schema->objects()->Get(field.type()->index())->name()->string_view()));
  }
}

void FillMessage(const reflection::Schema *schema,
                 const reflection::Object &object, pb::DescriptorProto *out) {
  out->set_name(std::string(LastComponent(object.name()->string_view())));

  // Same reason WriteTable sorts: reflection lists fields by name.  Sorting
  // locally rather than sharing the translator's cache, because building a
  // descriptor happens once and is allowed to allocate.
  std::vector<const reflection::Field *> sorted(object.fields()->begin(),
                                                object.fields()->end());
  std::sort(sorted.begin(), sorted.end(),
            [](const reflection::Field *a, const reflection::Field *b) {
              return a->id() < b->id();
            });

  for (const reflection::Field *field : sorted) {
    if (field->deprecated()) {
      // Deprecated fields hold a place in the flatbuffer's numbering and have
      // no counterpart in the proto, which is the whole point of them here.
      continue;
    }
    FillField(schema, *field, out->add_field());
  }
}

}  // namespace

std::string_view ProtoMessageName(const reflection::Schema *schema) {
  ABSL_CHECK(schema->root_table() != nullptr) << ": Schema has no root table";
  return schema->root_table()->name()->string_view();
}

std::vector<uint8_t> FileDescriptorProtoForSchema(
    const reflection::Schema *schema, std::string_view file_name) {
  ABSL_CHECK(schema->root_table() != nullptr) << ": Schema has no root table";
  const std::string_view package =
      Namespace(schema->root_table()->name()->string_view());

  for (const reflection::Object *object : *schema->objects()) {
    ABSL_CHECK_EQ(Namespace(object->name()->string_view()), package)
        << ": " << object->name()->string_view()
        << " is not in the root table's namespace (" << package
        << ").  Describing a schema that spans namespaces is not supported.  "
           "A FileDescriptorProto names one package, so it would have to "
           "become several files -- one per namespace, each listing the "
           "others in `dependency` and naming their messages by a fully "
           "qualified type_name, which is what a .proto's `import` compiles "
           "to and what NetworkTables carries by registering each file under "
           "proto:<file name>.  Nothing needs that yet, so this dies rather "
           "than emitting a descriptor whose package does not cover half its "
           "messages.  Encoding is unaffected -- the wire format has no "
           "notion of a package.";
  }

  pb::FileDescriptorProto file;
  file.set_name(std::string(file_name));
  if (!package.empty()) {
    file.set_package(std::string(package));
  }
  file.set_syntax("proto3");
  for (const reflection::Object *object : *schema->objects()) {
    FillMessage(schema, *object, file.add_message_type());
  }

  std::string serialized;
  ABSL_CHECK(file.SerializeToString(&serialized));
  return std::vector<uint8_t>(serialized.begin(), serialized.end());
}

size_t FlatbufferToProto::EncodedSize(const uint8_t *flatbuffer) const {
  Cursor counter;
  WriteTable(&counter, *this, *schema_->root_table(),
             *flatbuffers::GetAnyRoot(flatbuffer));
  return counter.size();
}

size_t FlatbufferToProto::Encode(const uint8_t *flatbuffer,
                                 std::span<uint8_t> buffer) const {
  const size_t needed = EncodedSize(flatbuffer);
  if (needed > buffer.size()) {
    return 0;
  }

  // Both of these wrap the caller's buffer and live on the stack, so writing
  // touches no allocator.
  pbio::ArrayOutputStream array(buffer.data(), static_cast<int>(buffer.size()));
  pbio::CodedOutputStream output(&array);
  Cursor writer(&output);
  WriteTable(&writer, *this, *schema_->root_table(),
             *flatbuffers::GetAnyRoot(flatbuffer));
  ABSL_CHECK_EQ(writer.size(), needed)
      << ": the sizing and writing passes disagreed";
  return writer.size();
}

}  // namespace aos
