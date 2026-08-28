#include "aos/protobuf/flatbuffer_to_proto.h"

#include <array>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "google/protobuf/dynamic_message.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/wire_format_lite.h"
#include "gtest/gtest.h"

#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers/builder.h"
#include "aos/protobuf/flatbuffer_to_proto_bad_attribute_test_generated.h"
#include "aos/protobuf/flatbuffer_to_proto_bad_attribute_test_schema.h"
#include "aos/protobuf/flatbuffer_to_proto_scalars_test_generated.h"
#include "aos/protobuf/flatbuffer_to_proto_scalars_test_schema.h"
#include "aos/protobuf/flatbuffer_to_proto_spanning_test_generated.h"
#include "aos/protobuf/flatbuffer_to_proto_spanning_test_schema.h"
#include "aos/protobuf/flatbuffer_to_proto_test_generated.h"
#include "aos/protobuf/flatbuffer_to_proto_test_schema.h"
#include "aos/protobuf/flatbuffer_to_proto_test_static.h"
#include "aos/realtime.h"
#include "aos/sanitizers.h"

namespace aos::testing {

const reflection::Schema *Schema() {
  static const flatbuffers::span<const uint8_t> span = ControlDataSchema();
  return reflection::GetSchema(reinterpret_cast<const char *>(span.data()));
}

// The tests below assert on the bytes the encoder produced, walked with
// protobuf's own CodedInputStream rather than parsed by hand.
//
// Deliberately not DynamicMessage over our generated descriptor -- that is
// ProtobufDecodesUsingTheGeneratedDescriptor's job, and it can only ever say
// the two halves agree with each other.  A wire type declared wrong *and*
// written the same wrong way round-trips through it perfectly.  These say what
// is actually on the wire, so the descriptor is not a party to the assertion.
struct Field {
  uint32_t number = 0;
  uint32_t wire_type = 0;
  uint64_t value = 0;
  std::vector<uint8_t> bytes;
};

namespace pbio = ::google::protobuf::io;
using WireFormatLite = ::google::protobuf::internal::WireFormatLite;

std::vector<Field> Decode(std::span<const uint8_t> message) {
  std::vector<Field> result;
  pbio::CodedInputStream input(message.data(), message.size());
  while (const uint32_t tag = input.ReadTag()) {
    Field field;
    field.number = WireFormatLite::GetTagFieldNumber(tag);
    field.wire_type = WireFormatLite::GetTagWireType(tag);
    switch (field.wire_type) {
      case WireFormatLite::WIRETYPE_VARINT:
        EXPECT_TRUE(input.ReadVarint64(&field.value));
        break;
      case WireFormatLite::WIRETYPE_FIXED64: {
        uint64_t value = 0;
        EXPECT_TRUE(input.ReadLittleEndian64(&value));
        field.value = value;
        break;
      }
      case WireFormatLite::WIRETYPE_FIXED32: {
        uint32_t value = 0;
        EXPECT_TRUE(input.ReadLittleEndian32(&value));
        field.value = value;
        break;
      }
      case WireFormatLite::WIRETYPE_LENGTH_DELIMITED: {
        uint32_t length = 0;
        EXPECT_TRUE(input.ReadVarint32(&length));
        std::string bytes;
        EXPECT_TRUE(input.ReadString(&bytes, length));
        field.bytes.assign(bytes.begin(), bytes.end());
        break;
      }
      default:
        ADD_FAILURE() << "Unknown wire type " << field.wire_type;
        return result;
    }
    result.push_back(std::move(field));
  }
  EXPECT_TRUE(input.ConsumedEntireMessage())
      << ": trailing bytes after the last field";
  return result;
}

// The payload of a packed repeated field, which is the elements back to back
// with no tags of their own.
std::vector<uint64_t> DecodePackedVarints(std::span<const uint8_t> payload) {
  std::vector<uint64_t> result;
  pbio::CodedInputStream input(payload.data(), payload.size());
  uint64_t value = 0;
  while (input.BytesUntilLimit() > 0 && input.ReadVarint64(&value)) {
    result.push_back(value);
  }
  EXPECT_EQ(input.BytesUntilLimit(), 0) << ": trailing bytes in the payload";
  return result;
}

std::vector<uint32_t> DecodePackedFixed32(std::span<const uint8_t> payload) {
  std::vector<uint32_t> result;
  pbio::CodedInputStream input(payload.data(), payload.size());
  uint32_t value = 0;
  while (input.BytesUntilLimit() > 0 && input.ReadLittleEndian32(&value)) {
    result.push_back(value);
  }
  EXPECT_EQ(input.BytesUntilLimit(), 0) << ": trailing bytes in the payload";
  return result;
}

std::vector<uint64_t> DecodePackedFixed64(std::span<const uint8_t> payload) {
  std::vector<uint64_t> result;
  pbio::CodedInputStream input(payload.data(), payload.size());
  uint64_t value = 0;
  while (input.BytesUntilLimit() > 0 && input.ReadLittleEndian64(&value)) {
    result.push_back(value);
  }
  EXPECT_EQ(input.BytesUntilLimit(), 0) << ": trailing bytes in the payload";
  return result;
}

const Field *Find(const std::vector<Field> &fields, uint32_t number) {
  for (const Field &field : fields) {
    if (field.number == number) {
      return &field;
    }
  }
  return nullptr;
}

// The result points into `fields`, so a temporary would dangle before it could
// be read.  Deleted rather than left to the reader: Decode() returns by value,
// which makes Find(Decode(out), n) the natural thing to write and a use of
// freed memory to run.
const Field *Find(std::vector<Field> &&fields, uint32_t number) = delete;

flatbuffers::DetachedBuffer MakeControlData() {
  flatbuffers::FlatBufferBuilder fbb;

  const std::array<int32_t, 3> axes{-1, 2, -32768};
  const auto axes_offset = fbb.CreateVector(axes.data(), axes.size());

  JoystickDataBuilder joystick(fbb);
  joystick.add_available_buttons(0xFFFF);
  joystick.add_buttons(0x555);
  joystick.add_available_axes(0x7);
  joystick.add_axes(axes_offset);
  joystick.add_pov_count(2);
  joystick.add_povs(0xF3);
  const auto joystick_offset = joystick.Finish();

  const auto joysticks_offset = fbb.CreateVector(&joystick_offset, 1);
  const auto game_data_offset = fbb.CreateString("LRL");

  ControlDataBuilder builder(fbb);
  builder.add_match_time(135);
  builder.add_joysticks(joysticks_offset);
  builder.add_current_op_mode(0x0300000000000042ULL);
  builder.add_control_word(0x21);
  builder.add_game_data(game_data_offset);
  fbb.Finish(builder.Finish());

  return fbb.Release();
}

// A protobuf field number is the flatbuffer id plus one, and a deprecated
// field holds the place of one the proto deleted -- so a schema mirroring a
// .proto reproduces its numbering without saying anything about it.
TEST(FlatbufferToProtoTest, FieldNumbersFollowTheIds) {
  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  const FlatbufferToProto translator(Schema());

  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));
  ASSERT_EQ(translator.Encode(buffer.data(), out), out.size());
  const std::vector<Field> fields = Decode(out);

  // MrcComm.proto's numbering: MatchTime=2, Joysticks=3, CurrentOpMode=4,
  // ControlWord=5, GameData=6.
  ASSERT_TRUE(Find(fields, 2) != nullptr);
  ASSERT_TRUE(Find(fields, 3) != nullptr);
  ASSERT_TRUE(Find(fields, 4) != nullptr);
  ASSERT_TRUE(Find(fields, 5) != nullptr);
  ASSERT_TRUE(Find(fields, 6) != nullptr);

  EXPECT_EQ(Find(fields, 2)->value, 135u);
  EXPECT_EQ(Find(fields, 5)->value, 0x21u);
  EXPECT_EQ(
      std::string(Find(fields, 6)->bytes.begin(), Find(fields, 6)->bytes.end()),
      "LRL");
}

// Fields come out in protobuf field-number order.
//
// reflection::Object::fields is sorted by name, so walking it directly emits
// them in name order -- valid, since a decoder does not care, but not what
// anyone reading the bytes expects.  The test schema is built so the two
// orders disagree: by name it is control_word, current_op_mode, game_data,
// joysticks, match_time, and by number it is 2, 3, 4, 5, 6.
TEST(FlatbufferToProtoTest, EmitsFieldsInNumberOrder) {
  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  const FlatbufferToProto translator(Schema());

  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));
  ASSERT_EQ(translator.Encode(buffer.data(), out), out.size());

  std::vector<uint32_t> numbers;
  for (const Field &field : Decode(out)) {
    numbers.push_back(field.number);
  }
  EXPECT_EQ(numbers, (std::vector<uint32_t>{2, 3, 4, 5, 6}));
}

// proto_type is followed, not advisory.  current_op_mode is a ulong, which on
// its own would be a plain varint; the attribute says fixed64, and fixed64 is
// what comes out.  A schema mirroring a .proto depends on that -- an ignored
// attribute would encode a field the decoder is expecting eight bytes for as
// a varint, and the message after it would be read at the wrong offset.
TEST(FlatbufferToProtoTest, HonorsProtoTypeAttribute) {
  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  const FlatbufferToProto translator(Schema());

  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));
  ASSERT_EQ(translator.Encode(buffer.data(), out), out.size());

  const std::vector<Field> fields = Decode(out);
  const Field *const op_mode = Find(fields, 4);
  ASSERT_TRUE(op_mode != nullptr);
  EXPECT_EQ(op_mode->wire_type, 1u) << ": fixed64, not a varint";
  EXPECT_EQ(op_mode->value, 0x0300000000000042ULL);
}

// "repeated sint32" is zigzag and packed.  Negative values are where zigzag
// and a plain varint visibly differ.
TEST(FlatbufferToProtoTest, EncodesZigzagPackedRepeated) {
  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  const FlatbufferToProto translator(Schema());

  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));
  ASSERT_EQ(translator.Encode(buffer.data(), out), out.size());

  const std::vector<Field> fields = Decode(out);
  const Field *const joysticks_field = Find(fields, 3);
  ASSERT_TRUE(joysticks_field != nullptr);

  const std::vector<Field> joysticks = Decode(joysticks_field->bytes);
  const Field *const axes = Find(joysticks, 4);
  ASSERT_TRUE(axes != nullptr);
  ASSERT_EQ(axes->wire_type, 2u) << ": packed repeated is length delimited";

  // -1 -> 1, 2 -> 4, -32768 -> 65535.
  EXPECT_EQ(DecodePackedVarints(axes->bytes),
            (std::vector<uint64_t>{1u, 4u, 65535u}));
}

// A schema fresh out of flatc shares its root_table with an objects() entry,
// but one that has been through a reflection-level copy -- the fate of every
// channel schema in a flattened config -- can carry a duplicate table
// instead.  Encoding starts at root_table(), so the translator has to work
// from either.
TEST(FlatbufferToProtoTest, EncodesFromACopiedSchema) {
  const FlatbufferDetachedBuffer<reflection::Schema> copied =
      RecursiveCopyFlatBuffer(Schema());
  ASSERT_NE(copied.message().root_table(), nullptr);

  const flatbuffers::DetachedBuffer buffer = MakeControlData();

  const FlatbufferToProto copied_translator(&copied.message());
  std::vector<uint8_t> from_copy(copied_translator.EncodedSize(buffer.data()));
  ASSERT_GT(from_copy.size(), 0u);
  ASSERT_EQ(copied_translator.Encode(buffer.data(), from_copy),
            from_copy.size());

  const FlatbufferToProto original_translator(Schema());
  std::vector<uint8_t> from_original(
      original_translator.EncodedSize(buffer.data()));
  ASSERT_EQ(original_translator.Encode(buffer.data(), from_original),
            from_original.size());

  // Byte for byte: which of the two tables the walk started from is not
  // allowed to change anything about the result.
  EXPECT_EQ(from_copy, from_original);
}

// The same message built through AOS's static flatbuffer API rather than
// through FlatBufferBuilder.  The translator only ever sees a finished
// flatbuffer, so the two should be indistinguishable to it -- and this is the
// API an AOS caller actually has, so it is the one worth pinning down.
TEST(FlatbufferToProtoTest, EncodesFromTheStaticApi) {
  aos::fbs::Builder<ControlDataStatic> builder;
  ControlDataStatic *const control_data = builder.get();
  control_data->set_match_time(135);
  control_data->set_current_op_mode(0x0300000000000042ULL);
  control_data->set_control_word(0x21);
  aos::fbs::SetStringOrDie(control_data->add_game_data(), "LRL");

  auto *const joysticks = control_data->add_joysticks();
  // Nothing here grows on append -- the static API reserves capacity up front
  // and tells you when it could not, which is the property that makes it
  // usable from realtime code.
  ASSERT_TRUE(joysticks->reserve(1));
  auto *const joystick = joysticks->emplace_back();
  ASSERT_TRUE(joystick != nullptr);
  joystick->set_available_buttons(0xFFFF);
  joystick->set_buttons(0x555);
  joystick->set_available_axes(0x7);
  joystick->set_pov_count(2);
  joystick->set_povs(0xF3);
  auto *const axes = joystick->add_axes();
  ASSERT_TRUE(axes->reserve(3));
  for (const int32_t value : {-1, 2, -32768}) {
    ASSERT_TRUE(axes->emplace_back(value));
  }

  ASSERT_TRUE(builder.Verify());
  // const, so span() picks the reading overload -- the mutable one is a
  // deliberate LOG(FATAL) on a FlatbufferSpan.
  const auto flatbuffer = builder.AsFlatbufferSpan();
  const absl::Span<const uint8_t> span = flatbuffer.span();

  const FlatbufferToProto translator(Schema());
  std::vector<uint8_t> out(translator.EncodedSize(span.data()));
  ASSERT_EQ(translator.Encode(span.data(), out), out.size());

  // The same fields with the same values, so the same bytes as the
  // FlatBufferBuilder version every other test here uses.
  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  std::vector<uint8_t> expected(translator.EncodedSize(buffer.data()));
  ASSERT_EQ(translator.Encode(buffer.data(), expected), expected.size());
  EXPECT_EQ(out, expected);
}

// A schema spanning namespaces has no descriptor here, and says so rather than
// emitting one whose package does not cover half its messages.
//
// A .fbs that includes one from another namespace produces exactly this --
// aos/network/web_proxy.fbs includes aos/configuration.fbs -- so it is a shape
// that exists, not a hypothetical.  Supporting it means emitting a file per
// namespace with dependency edges between them, which nothing needs yet: the
// protos this mirrors keep everything in one package (all of wpimath's are
// wpi.proto, split across files rather than packages).
//
// The line is drawn at the descriptor, not at the encoder, and the encode
// below is what says so: the wire format has no notion of a package, so a
// caller who only wants bytes is unaffected by any of this.
TEST(FlatbufferToProtoDeathTest, RefusesToDescribeASchemaSpanningNamespaces) {
  const flatbuffers::span<const uint8_t> span = spanning::SpanningSchema();
  const reflection::Schema *const schema =
      reflection::GetSchema(reinterpret_cast<const char *>(span.data()));

  EXPECT_DEATH(
      { FileDescriptorProtoForSchema(schema, "spanning.proto"); },
      "is not in the root table's namespace");

  flatbuffers::FlatBufferBuilder fbb;
  spanning::SpanningBuilder builder(fbb);
  fbb.Finish(builder.Finish());
  const flatbuffers::DetachedBuffer buffer = fbb.Release();

  const FlatbufferToProto translator(schema);
  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));
  EXPECT_EQ(translator.Encode(buffer.data(), out), out.size())
      << ": encoding does not care what namespace anything is in";
}

// A proto_type on a field whose flatbuffer type has exactly one protobuf type
// is a schema bug, not a preference -- there is nothing for it to pick, and
// the descriptor would contradict it.  Caught when the translator is built,
// which is where whoever wrote the schema will see it.
TEST(FlatbufferToProtoDeathTest, RejectsAProtoTypeWithNothingToPick) {
  const flatbuffers::span<const uint8_t> span = BadAttributeSchema();
  const reflection::Schema *const schema =
      reflection::GetSchema(reinterpret_cast<const char *>(span.data()));
  EXPECT_DEATH(
      { FlatbufferToProto translator(schema); },
      "has proto_type .*nothing for the attribute to pick");
}

// proto3 leaves a scalar at its default off the wire, which is also what the
// flatbuffer means by not having it in the vtable.
TEST(FlatbufferToProtoTest, OmitsDefaults) {
  flatbuffers::FlatBufferBuilder fbb;
  ControlDataBuilder builder(fbb);
  builder.add_match_time(0);
  builder.add_control_word(7);
  fbb.Finish(builder.Finish());
  const flatbuffers::DetachedBuffer buffer = fbb.Release();

  const FlatbufferToProto translator(Schema());
  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));
  ASSERT_EQ(translator.Encode(buffer.data(), out), out.size());
  const std::vector<Field> fields = Decode(out);

  EXPECT_TRUE(Find(fields, 2) == nullptr) << ": MatchTime was 0";
  ASSERT_TRUE(Find(fields, 5) != nullptr);
  EXPECT_EQ(Find(fields, 5)->value, 7u);
}

// The point of the whole design: translating a message touches no allocator.
//
// aos::ScopedRealtime installs AOS's malloc hook, so an allocation anywhere
// inside Encode -- including inside protobuf's CodedOutputStream -- aborts
// rather than merely being counted.  That the hook catches an allocation at
// all is aos/realtime_test.cc's RealtimeDeathTest.
TEST(FlatbufferToProtoTest, EncodingDoesNotAllocate) {
  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  const FlatbufferToProto translator(Schema());
  // Sized once, up front, the way a caller would -- this is the part that is
  // allowed to allocate.
  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));

  size_t written = 0;
  {
#if !defined(AOS_SANITIZE_MEMORY) && !defined(AOS_SANITIZE_ADDRESS)
    aos::ScopedRealtime realtime;
#endif
    written = translator.Encode(buffer.data(), out);
  }

  EXPECT_EQ(written, out.size());
}

// A buffer that is too small is reported rather than overrun.
TEST(FlatbufferToProtoTest, RefusesASmallBuffer) {
  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  const FlatbufferToProto translator(Schema());

  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()) - 1);
  EXPECT_EQ(translator.Encode(buffer.data(), out), 0u);
}

// The descriptor and the encoder have to agree, and the only way to know that
// is to hand both to a real protobuf implementation: parse the generated
// FileDescriptorProto, build the message type it describes, and decode an
// encoded message with it.  Anything the two disagree about -- a field number,
// a wire type, a name -- shows up here as a decode that does not match.
TEST(FlatbufferToProtoTest, ProtobufDecodesUsingTheGeneratedDescriptor) {
  const std::vector<uint8_t> descriptor_bytes =
      FileDescriptorProtoForSchema(Schema(), "flatbuffer_to_proto_test.proto");

  google::protobuf::FileDescriptorProto file;
  ASSERT_TRUE(
      file.ParseFromArray(descriptor_bytes.data(), descriptor_bytes.size()))
      << ": the generated descriptor is not valid protobuf";

  google::protobuf::DescriptorPool pool;
  const google::protobuf::FileDescriptor *const file_descriptor =
      pool.BuildFile(file);
  ASSERT_TRUE(file_descriptor != nullptr)
      << ": protobuf rejected the generated descriptor";

  const google::protobuf::Descriptor *const control_data =
      pool.FindMessageTypeByName(std::string(ProtoMessageName(Schema())));
  ASSERT_TRUE(control_data != nullptr)
      << ": no " << ProtoMessageName(Schema()) << " in the descriptor";

  const flatbuffers::DetachedBuffer buffer = MakeControlData();
  const FlatbufferToProto translator(Schema());
  std::vector<uint8_t> encoded(translator.EncodedSize(buffer.data()));
  ASSERT_EQ(translator.Encode(buffer.data(), encoded), encoded.size());

  google::protobuf::DynamicMessageFactory factory(&pool);
  std::unique_ptr<google::protobuf::Message> message(
      factory.GetPrototype(control_data)->New());
  ASSERT_TRUE(message->ParseFromArray(encoded.data(), encoded.size()))
      << ": protobuf could not decode what the encoder produced";

  const google::protobuf::Reflection *const reflection =
      message->GetReflection();
  EXPECT_EQ(reflection->GetInt32(*message,
                                 control_data->FindFieldByName("match_time")),
            135);
  EXPECT_EQ(reflection->GetUInt32(
                *message, control_data->FindFieldByName("control_word")),
            0x21u);
  EXPECT_EQ(reflection->GetString(*message,
                                  control_data->FindFieldByName("game_data")),
            "LRL");
  EXPECT_EQ(reflection->GetUInt64(
                *message, control_data->FindFieldByName("current_op_mode")),
            0x0300000000000042ULL);

  // Down into the nested repeated message, where the zigzag axes live.
  const google::protobuf::FieldDescriptor *const joysticks =
      control_data->FindFieldByName("joysticks");
  ASSERT_EQ(reflection->FieldSize(*message, joysticks), 1);
  const google::protobuf::Message &joystick =
      reflection->GetRepeatedMessage(*message, joysticks, 0);
  const google::protobuf::Descriptor *const joystick_type =
      joystick.GetDescriptor();
  const google::protobuf::FieldDescriptor *const axes =
      joystick_type->FindFieldByName("axes");
  ASSERT_EQ(axes->type(), google::protobuf::FieldDescriptor::TYPE_SINT32)
      << ": proto_type should have made this zigzag";
  ASSERT_EQ(joystick.GetReflection()->FieldSize(joystick, axes), 3);
  EXPECT_EQ(joystick.GetReflection()->GetRepeatedInt32(joystick, axes, 0), -1);
  EXPECT_EQ(joystick.GetReflection()->GetRepeatedInt32(joystick, axes, 1), 2);
  EXPECT_EQ(joystick.GetReflection()->GetRepeatedInt32(joystick, axes, 2),
            -32768);
}

// The deprecated field holding the place of the proto's deleted field 1 is not
// in the descriptor, and the fields around it keep the proto's numbering.
TEST(FlatbufferToProtoTest, DeprecatedFieldsAreNotInTheDescriptor) {
  const std::vector<uint8_t> descriptor_bytes =
      FileDescriptorProtoForSchema(Schema(), "flatbuffer_to_proto_test.proto");

  google::protobuf::FileDescriptorProto file;
  ASSERT_TRUE(
      file.ParseFromArray(descriptor_bytes.data(), descriptor_bytes.size()));
  google::protobuf::DescriptorPool pool;
  ASSERT_TRUE(pool.BuildFile(file) != nullptr);

  const google::protobuf::Descriptor *const control_data =
      pool.FindMessageTypeByName(std::string(ProtoMessageName(Schema())));
  ASSERT_TRUE(control_data != nullptr);

  EXPECT_TRUE(control_data->FindFieldByName("reserved_0") == nullptr);
  EXPECT_TRUE(control_data->FindFieldByNumber(1) == nullptr)
      << ": the proto has no field 1";
  EXPECT_EQ(control_data->FindFieldByName("match_time")->number(), 2);
  EXPECT_EQ(control_data->FindFieldByName("game_data")->number(), 6);
}

// Everything below covers flatbuffer_to_proto_scalars_test.fbs, which exists
// to reach every encoding rather than to mirror a real .proto.

const reflection::Schema *ScalarsSchemaFor() {
  static const flatbuffers::span<const uint8_t> span = ScalarsSchema();
  return reflection::GetSchema(reinterpret_cast<const char *>(span.data()));
}

// One message with every field set to a value that is not its default, so a
// test below can assert on exactly the field it is about.  The values are
// chosen to be wrong under a neighbouring encoding: negatives, so zigzag and a
// plain varint disagree; a float whose bits are not its integer value.
flatbuffers::DetachedBuffer MakeScalars() {
  flatbuffers::FlatBufferBuilder fbb;

  const auto string_offset = fbb.CreateString("hi");
  const std::array<Inner, 2> inners{Inner(7, 0.5f), Inner(-1, -0.5f)};
  const auto inners_offset = fbb.CreateVectorOfStructs(inners.data(), 2);
  const std::array<float, 2> floats{1.0f, 2.0f};
  const auto floats_offset = fbb.CreateVector(floats.data(), floats.size());

  // A vector of each scalar, so the packed path is exercised for every
  // encoding rather than only for floats and zigzag ints.
  const std::array<uint8_t, 2> bools{1, 0};
  const auto bools_offset = fbb.CreateVector(bools.data(), bools.size());
  const std::array<int8_t, 2> bytes{-2, 3};
  const auto bytes_offset = fbb.CreateVector(bytes.data(), bytes.size());
  const std::array<uint8_t, 2> ubytes{200, 1};
  const auto ubytes_offset = fbb.CreateVector(ubytes.data(), ubytes.size());
  const std::array<int16_t, 2> shorts{-300, 4};
  const auto shorts_offset = fbb.CreateVector(shorts.data(), shorts.size());
  const std::array<uint16_t, 2> ushorts{40000, 5};
  const auto ushorts_offset = fbb.CreateVector(ushorts.data(), ushorts.size());
  const std::array<int32_t, 2> ints{-70000, 6};
  const auto ints_offset = fbb.CreateVector(ints.data(), ints.size());
  const std::array<uint32_t, 2> uints{3000000000u, 7};
  const auto uints_offset = fbb.CreateVector(uints.data(), uints.size());
  const std::array<int64_t, 2> longs{-5000000000LL, 8};
  const auto longs_offset = fbb.CreateVector(longs.data(), longs.size());
  const std::array<uint64_t, 2> ulongs{0xF000000000000001ULL, 9};
  const auto ulongs_offset = fbb.CreateVector(ulongs.data(), ulongs.size());
  const std::array<double, 2> doubles{-2.25, 4.5};
  const auto doubles_offset = fbb.CreateVector(doubles.data(), doubles.size());
  const std::vector<std::string> strings{"one", "two"};
  const auto strings_offset = fbb.CreateVectorOfStrings(strings);
  const std::array<int32_t, 2> zigzag_ints{-3, 4};
  const auto zigzag_ints_offset =
      fbb.CreateVector(zigzag_ints.data(), zigzag_ints.size());
  const std::array<int64_t, 2> zigzag_longs{-4, 5};
  const auto zigzag_longs_offset =
      fbb.CreateVector(zigzag_longs.data(), zigzag_longs.size());
  const std::array<uint32_t, 2> fixed_uints{0xDEADBEEFu, 1};
  const auto fixed_uints_offset =
      fbb.CreateVector(fixed_uints.data(), fixed_uints.size());
  const std::array<int32_t, 2> fixed_ints{-2, 3};
  const auto fixed_ints_offset =
      fbb.CreateVector(fixed_ints.data(), fixed_ints.size());
  const std::array<uint64_t, 2> fixed_ulongs{0x0123456789ABCDEFULL, 2};
  const auto fixed_ulongs_offset =
      fbb.CreateVector(fixed_ulongs.data(), fixed_ulongs.size());
  const std::array<int64_t, 2> fixed_longs{-2, 3};
  const auto fixed_longs_offset =
      fbb.CreateVector(fixed_longs.data(), fixed_longs.size());

  // Inner.a is deliberately 0: a struct has no vtable, so that zero has to
  // reach the wire rather than being skipped the way a table's default is.
  const Outer outer(Inner(0, 2.5f), -2.25);

  ScalarsBuilder builder(fbb);
  builder.add_the_bool(true);
  builder.add_the_byte(-2);
  builder.add_the_ubyte(200);
  builder.add_the_short(-300);
  builder.add_the_ushort(40000);
  builder.add_the_int(-70000);
  builder.add_the_uint(3000000000u);
  builder.add_the_long(-5000000000LL);
  builder.add_the_ulong(0xF000000000000001ULL);
  builder.add_the_float(1.5f);
  builder.add_the_double(-2.25);
  builder.add_the_string(string_offset);
  builder.add_zigzag_int(-3);
  builder.add_zigzag_long(-4);
  builder.add_fixed_uint(0xDEADBEEFu);
  builder.add_fixed_int(-2);
  builder.add_fixed_ulong(0x0123456789ABCDEFULL);
  builder.add_fixed_long(-2);
  builder.add_outer(&outer);
  builder.add_inners(inners_offset);
  builder.add_bools(bools_offset);
  builder.add_bytes_(bytes_offset);
  builder.add_ubytes(ubytes_offset);
  builder.add_shorts(shorts_offset);
  builder.add_ushorts(ushorts_offset);
  builder.add_ints(ints_offset);
  builder.add_uints(uints_offset);
  builder.add_longs(longs_offset);
  builder.add_ulongs(ulongs_offset);
  builder.add_floats(floats_offset);
  builder.add_doubles(doubles_offset);
  builder.add_strings(strings_offset);
  builder.add_zigzag_ints(zigzag_ints_offset);
  builder.add_zigzag_longs(zigzag_longs_offset);
  builder.add_fixed_uints(fixed_uints_offset);
  builder.add_fixed_ints(fixed_ints_offset);
  builder.add_fixed_ulongs(fixed_ulongs_offset);
  builder.add_fixed_longs(fixed_longs_offset);
  fbb.Finish(builder.Finish());

  return fbb.Release();
}

std::vector<uint8_t> EncodeScalars(const flatbuffers::DetachedBuffer &buffer) {
  const FlatbufferToProto translator(ScalarsSchemaFor());
  std::vector<uint8_t> out(translator.EncodedSize(buffer.data()));
  EXPECT_EQ(translator.Encode(buffer.data(), out), out.size());
  return out;
}

// The float bits protobuf would write, so the expectations below say what is
// on the wire rather than restating memcpy.
uint32_t FloatBits(float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

uint64_t DoubleBits(double value) {
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

// A signed integer narrower than 64 bits is sign-extended before it is
// written, which is what protobuf does with a negative int32: ten bytes of
// varint, not five.
TEST(FlatbufferToProtoScalarsTest, EncodesSignedIntegersAsSignExtendedVarints) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  for (const auto &[number, expected] :
       std::vector<std::pair<uint32_t, uint64_t>>{
           {2, static_cast<uint64_t>(int64_t{-2})},
           {4, static_cast<uint64_t>(int64_t{-300})},
           {6, static_cast<uint64_t>(int64_t{-70000})},
           {8, static_cast<uint64_t>(int64_t{-5000000000LL})}}) {
    const Field *const field = Find(fields, number);
    ASSERT_TRUE(field != nullptr) << ": no field " << number;
    EXPECT_EQ(field->wire_type, 0u) << ": field " << number;
    EXPECT_EQ(field->value, expected) << ": field " << number;
  }
}

TEST(FlatbufferToProtoScalarsTest, EncodesUnsignedIntegersAsPlainVarints) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  for (const auto &[number, expected] :
       std::vector<std::pair<uint32_t, uint64_t>>{{3, 200u},
                                                  {5, 40000u},
                                                  {7, 3000000000u},
                                                  {9, 0xF000000000000001ULL}}) {
    const Field *const field = Find(fields, number);
    ASSERT_TRUE(field != nullptr) << ": no field " << number;
    EXPECT_EQ(field->wire_type, 0u) << ": field " << number;
    EXPECT_EQ(field->value, expected) << ": field " << number;
  }
}

TEST(FlatbufferToProtoScalarsTest, EncodesABoolAsAVarint) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);
  const Field *const field = Find(fields, 1);

  ASSERT_TRUE(field != nullptr);
  EXPECT_EQ(field->wire_type, 0u);
  EXPECT_EQ(field->value, 1u);
}

// A float is its bits in fixed32, not its value in a varint -- the distinction
// the encoding switch has to make between a float and an integer of the same
// width.
TEST(FlatbufferToProtoScalarsTest, EncodesAFloatAsItsBitsInFixed32) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);
  const Field *const field = Find(fields, 10);

  ASSERT_TRUE(field != nullptr);
  EXPECT_EQ(field->wire_type, 5u);
  EXPECT_EQ(field->value, FloatBits(1.5f));
  EXPECT_NE(field->value, 1u) << ": that would be the value, not the bits";
}

TEST(FlatbufferToProtoScalarsTest, EncodesADoubleAsItsBitsInFixed64) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);
  const Field *const field = Find(fields, 11);

  ASSERT_TRUE(field != nullptr);
  EXPECT_EQ(field->wire_type, 1u);
  EXPECT_EQ(field->value, DoubleBits(-2.25));
}

TEST(FlatbufferToProtoScalarsTest, EncodesAStringLengthDelimited) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);
  const Field *const field = Find(fields, 12);

  ASSERT_TRUE(field != nullptr);
  EXPECT_EQ(field->wire_type, 2u);
  EXPECT_EQ(std::string(field->bytes.begin(), field->bytes.end()), "hi");
}

// sint32 and sint64 both zigzag, which is where they visibly differ from a
// plain varint: -3 becomes 5 rather than a ten-byte sign-extended -3.
TEST(FlatbufferToProtoScalarsTest, EncodesZigzagScalars) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  const Field *const zigzag_int = Find(fields, 13);
  ASSERT_TRUE(zigzag_int != nullptr);
  EXPECT_EQ(zigzag_int->wire_type, 0u);
  EXPECT_EQ(zigzag_int->value, 5u);

  const Field *const zigzag_long = Find(fields, 14);
  ASSERT_TRUE(zigzag_long != nullptr);
  EXPECT_EQ(zigzag_long->wire_type, 0u);
  EXPECT_EQ(zigzag_long->value, 7u);
}

// A fixed-width integer is truncated to its width rather than sign-extended,
// which is the arm a float does not take.
TEST(FlatbufferToProtoScalarsTest, EncodesFixedWidthIntegers) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  const Field *const fixed_uint = Find(fields, 15);
  ASSERT_TRUE(fixed_uint != nullptr);
  EXPECT_EQ(fixed_uint->wire_type, 5u);
  EXPECT_EQ(fixed_uint->value, 0xDEADBEEFu);

  const Field *const fixed_int = Find(fields, 16);
  ASSERT_TRUE(fixed_int != nullptr);
  EXPECT_EQ(fixed_int->wire_type, 5u);
  EXPECT_EQ(fixed_int->value, 0xFFFFFFFEu);

  const Field *const fixed_ulong = Find(fields, 17);
  ASSERT_TRUE(fixed_ulong != nullptr);
  EXPECT_EQ(fixed_ulong->wire_type, 1u);
  EXPECT_EQ(fixed_ulong->value, 0x0123456789ABCDEFULL);

  const Field *const fixed_long = Find(fields, 18);
  ASSERT_TRUE(fixed_long != nullptr);
  EXPECT_EQ(fixed_long->wire_type, 1u);
  EXPECT_EQ(fixed_long->value, 0xFFFFFFFFFFFFFFFEULL);
}

// A struct is a nested message, and a struct inside a struct nests again.
// Every field of one is on the wire, including Inner.a's zero, which a table
// would have left off.
TEST(FlatbufferToProtoScalarsTest, EncodesANestedStructWithoutSkippingZeroes) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);

  const std::vector<Field> fields = Decode(out);
  const Field *const outer = Find(fields, 19);
  ASSERT_TRUE(outer != nullptr);
  ASSERT_EQ(outer->wire_type, 2u);

  const std::vector<Field> outer_fields = Decode(outer->bytes);
  ASSERT_EQ(outer_fields.size(), 2u)
      << ": a struct writes every field, so both of Outer's are here";

  const Field *const inner = Find(outer_fields, 1);
  ASSERT_TRUE(inner != nullptr);
  ASSERT_EQ(inner->wire_type, 2u);

  const Field *const c = Find(outer_fields, 2);
  ASSERT_TRUE(c != nullptr);
  EXPECT_EQ(c->wire_type, 1u);
  EXPECT_EQ(c->value, DoubleBits(-2.25));

  const std::vector<Field> inner_fields = Decode(inner->bytes);
  ASSERT_EQ(inner_fields.size(), 2u);

  const Field *const a = Find(inner_fields, 1);
  ASSERT_TRUE(a != nullptr) << ": a struct's zero is not a default to skip";
  EXPECT_EQ(a->wire_type, 0u);
  EXPECT_EQ(a->value, 0u);

  const Field *const b = Find(inner_fields, 2);
  ASSERT_TRUE(b != nullptr);
  EXPECT_EQ(b->wire_type, 5u);
  EXPECT_EQ(b->value, FloatBits(2.5f));
}

// A vector of structs is repeated messages, each length-delimited under the
// same field number -- the other way WriteStruct() is reached.
TEST(FlatbufferToProtoScalarsTest, EncodesAVectorOfStructs) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);

  std::vector<const Field *> inners;
  const std::vector<Field> fields = Decode(out);
  for (const Field &field : fields) {
    if (field.number == 20) {
      inners.push_back(&field);
    }
  }
  ASSERT_EQ(inners.size(), 2u) << ": one length-delimited message each";

  const std::vector<Field> first = Decode(inners[0]->bytes);
  EXPECT_EQ(Find(first, 1)->value, 7u);
  EXPECT_EQ(Find(first, 2)->value, FloatBits(0.5f));

  const std::vector<Field> second = Decode(inners[1]->bytes);
  EXPECT_EQ(Find(second, 1)->value, static_cast<uint64_t>(int64_t{-1}));
  EXPECT_EQ(Find(second, 2)->value, FloatBits(-0.5f));
}

// Every scalar also has a vector, and proto3 packs repeated scalars: one
// length-delimited field holding the elements back to back, whose length is
// measured by walking them before any is written.  A width that disagrees
// between the measuring pass and the writing pass corrupts everything after
// it, so each encoding gets its vector checked as well as its scalar.
TEST(FlatbufferToProtoScalarsTest, EncodesPackedVarintVectors) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  const auto packed = [&fields](uint32_t number) {
    const Field *const field = Find(fields, number);
    EXPECT_TRUE(field != nullptr) << ": no field " << number;
    EXPECT_EQ(field->wire_type, 2u) << ": field " << number << " is packed";
    return DecodePackedVarints(field->bytes);
  };

  EXPECT_EQ(packed(21), (std::vector<uint64_t>{1, 0})) << ": bools";
  EXPECT_EQ(packed(22),
            (std::vector<uint64_t>{static_cast<uint64_t>(int64_t{-2}), 3}))
      << ": signed bytes sign-extend";
  EXPECT_EQ(packed(23), (std::vector<uint64_t>{200, 1}));
  EXPECT_EQ(packed(24),
            (std::vector<uint64_t>{static_cast<uint64_t>(int64_t{-300}), 4}));
  EXPECT_EQ(packed(25), (std::vector<uint64_t>{40000, 5}));
  EXPECT_EQ(packed(26),
            (std::vector<uint64_t>{static_cast<uint64_t>(int64_t{-70000}), 6}));
  EXPECT_EQ(packed(27), (std::vector<uint64_t>{3000000000u, 7}));
  EXPECT_EQ(packed(28), (std::vector<uint64_t>{
                            static_cast<uint64_t>(int64_t{-5000000000LL}), 8}));
  EXPECT_EQ(packed(29), (std::vector<uint64_t>{0xF000000000000001ULL, 9}));
}

TEST(FlatbufferToProtoScalarsTest, EncodesPackedFloatAndDoubleVectors) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  const Field *const floats = Find(fields, 30);
  ASSERT_TRUE(floats != nullptr);
  ASSERT_EQ(floats->wire_type, 2u);
  EXPECT_EQ(DecodePackedFixed32(floats->bytes),
            (std::vector<uint32_t>{FloatBits(1.0f), FloatBits(2.0f)}));

  const Field *const doubles = Find(fields, 31);
  ASSERT_TRUE(doubles != nullptr);
  ASSERT_EQ(doubles->wire_type, 2u);
  EXPECT_EQ(DecodePackedFixed64(doubles->bytes),
            (std::vector<uint64_t>{DoubleBits(-2.25), DoubleBits(4.5)}));
}

// Strings are the one repeated type protobuf cannot pack, so they arrive as
// one tagged field each rather than as a single length-delimited run.
TEST(FlatbufferToProtoScalarsTest, EncodesRepeatedStringsUnpacked) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);

  std::vector<std::string> strings;
  for (const Field &field : Decode(out)) {
    if (field.number == 32) {
      EXPECT_EQ(field.wire_type, 2u);
      strings.emplace_back(field.bytes.begin(), field.bytes.end());
    }
  }
  EXPECT_EQ(strings, (std::vector<std::string>{"one", "two"}));
}

TEST(FlatbufferToProtoScalarsTest, EncodesPackedZigzagVectors) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  const Field *const ints = Find(fields, 33);
  ASSERT_TRUE(ints != nullptr);
  // -3 -> 5, 4 -> 8.
  EXPECT_EQ(DecodePackedVarints(ints->bytes), (std::vector<uint64_t>{5, 8}));

  const Field *const longs = Find(fields, 34);
  ASSERT_TRUE(longs != nullptr);
  // -4 -> 7, 5 -> 10.
  EXPECT_EQ(DecodePackedVarints(longs->bytes), (std::vector<uint64_t>{7, 10}));
}

TEST(FlatbufferToProtoScalarsTest, EncodesPackedFixedWidthVectors) {
  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> out = EncodeScalars(buffer);
  const std::vector<Field> fields = Decode(out);

  const Field *const fixed_uints = Find(fields, 35);
  ASSERT_TRUE(fixed_uints != nullptr);
  EXPECT_EQ(DecodePackedFixed32(fixed_uints->bytes),
            (std::vector<uint32_t>{0xDEADBEEFu, 1}));

  const Field *const fixed_ints = Find(fields, 36);
  ASSERT_TRUE(fixed_ints != nullptr);
  EXPECT_EQ(DecodePackedFixed32(fixed_ints->bytes),
            (std::vector<uint32_t>{0xFFFFFFFEu, 3}))
      << ": truncated to 32 bits, not sign-extended to 64";

  const Field *const fixed_ulongs = Find(fields, 37);
  ASSERT_TRUE(fixed_ulongs != nullptr);
  EXPECT_EQ(DecodePackedFixed64(fixed_ulongs->bytes),
            (std::vector<uint64_t>{0x0123456789ABCDEFULL, 2}));

  const Field *const fixed_longs = Find(fields, 38);
  ASSERT_TRUE(fixed_longs != nullptr);
  EXPECT_EQ(DecodePackedFixed64(fixed_longs->bytes),
            (std::vector<uint64_t>{0xFFFFFFFFFFFFFFFEULL, 3}));
}

// The descriptor and the encoder agree about every one of these encodings, not
// just the ones the MrcComm mirror happens to use.  A wire type declared one
// way and written another shows up here as a value that does not survive the
// round trip.
TEST(FlatbufferToProtoScalarsTest, ProtobufDecodesEveryEncoding) {
  const std::vector<uint8_t> descriptor_bytes = FileDescriptorProtoForSchema(
      ScalarsSchemaFor(), "flatbuffer_to_proto_scalars_test.proto");

  google::protobuf::FileDescriptorProto file;
  ASSERT_TRUE(
      file.ParseFromArray(descriptor_bytes.data(), descriptor_bytes.size()));

  google::protobuf::DescriptorPool pool;
  ASSERT_TRUE(pool.BuildFile(file) != nullptr)
      << ": protobuf rejected the generated descriptor";

  const google::protobuf::Descriptor *const scalars =
      pool.FindMessageTypeByName(
          std::string(ProtoMessageName(ScalarsSchemaFor())));
  ASSERT_TRUE(scalars != nullptr);

  const flatbuffers::DetachedBuffer buffer = MakeScalars();
  const std::vector<uint8_t> encoded = EncodeScalars(buffer);

  google::protobuf::DynamicMessageFactory factory(&pool);
  std::unique_ptr<google::protobuf::Message> message(
      factory.GetPrototype(scalars)->New());
  ASSERT_TRUE(message->ParseFromArray(encoded.data(), encoded.size()))
      << ": protobuf could not decode what the encoder produced";

  const google::protobuf::Reflection *const reflection =
      message->GetReflection();
  EXPECT_TRUE(
      reflection->GetBool(*message, scalars->FindFieldByName("the_bool")));
  EXPECT_EQ(
      reflection->GetInt32(*message, scalars->FindFieldByName("the_byte")), -2);
  EXPECT_EQ(
      reflection->GetUInt32(*message, scalars->FindFieldByName("the_ubyte")),
      200u);
  EXPECT_EQ(
      reflection->GetInt32(*message, scalars->FindFieldByName("the_short")),
      -300);
  EXPECT_EQ(
      reflection->GetInt64(*message, scalars->FindFieldByName("the_long")),
      -5000000000LL);
  EXPECT_EQ(
      reflection->GetUInt64(*message, scalars->FindFieldByName("the_ulong")),
      0xF000000000000001ULL);
  EXPECT_EQ(
      reflection->GetFloat(*message, scalars->FindFieldByName("the_float")),
      1.5f);
  EXPECT_EQ(
      reflection->GetDouble(*message, scalars->FindFieldByName("the_double")),
      -2.25);
  EXPECT_EQ(
      reflection->GetInt32(*message, scalars->FindFieldByName("zigzag_int")),
      -3);
  EXPECT_EQ(
      reflection->GetInt64(*message, scalars->FindFieldByName("zigzag_long")),
      -4LL);
  EXPECT_EQ(
      reflection->GetUInt32(*message, scalars->FindFieldByName("fixed_uint")),
      0xDEADBEEFu);
  EXPECT_EQ(
      reflection->GetInt32(*message, scalars->FindFieldByName("fixed_int")),
      -2);
  EXPECT_EQ(
      reflection->GetInt64(*message, scalars->FindFieldByName("fixed_long")),
      -2LL);

  // The nested struct survives as a nested message, zero and all.
  const google::protobuf::Message &outer =
      reflection->GetMessage(*message, scalars->FindFieldByName("outer"));
  const google::protobuf::Descriptor *const outer_type = outer.GetDescriptor();
  const google::protobuf::Message &inner = outer.GetReflection()->GetMessage(
      outer, outer_type->FindFieldByName("inner"));
  EXPECT_EQ(inner.GetReflection()->GetInt32(
                inner, inner.GetDescriptor()->FindFieldByName("a")),
            0);
  EXPECT_EQ(inner.GetReflection()->GetFloat(
                inner, inner.GetDescriptor()->FindFieldByName("b")),
            2.5f);
}

}  // namespace aos::testing
