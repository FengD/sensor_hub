// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: eth_packet.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_eth_5fpacket_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_eth_5fpacket_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3014000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3014000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_eth_5fpacket_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_eth_5fpacket_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_eth_5fpacket_2eproto;
namespace crdc {
namespace airi {
class Packet;
class PacketDefaultTypeInternal;
extern PacketDefaultTypeInternal _Packet_default_instance_;
class Packets;
class PacketsDefaultTypeInternal;
extern PacketsDefaultTypeInternal _Packets_default_instance_;
}  // namespace airi
}  // namespace crdc
PROTOBUF_NAMESPACE_OPEN
template<> ::crdc::airi::Packet* Arena::CreateMaybeMessage<::crdc::airi::Packet>(Arena*);
template<> ::crdc::airi::Packets* Arena::CreateMaybeMessage<::crdc::airi::Packets>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace crdc {
namespace airi {

// ===================================================================

class Packet PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:crdc.airi.Packet) */ {
 public:
  inline Packet() : Packet(nullptr) {}
  virtual ~Packet();

  Packet(const Packet& from);
  Packet(Packet&& from) noexcept
    : Packet() {
    *this = ::std::move(from);
  }

  inline Packet& operator=(const Packet& from) {
    CopyFrom(from);
    return *this;
  }
  inline Packet& operator=(Packet&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const Packet& default_instance();

  static inline const Packet* internal_default_instance() {
    return reinterpret_cast<const Packet*>(
               &_Packet_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Packet& a, Packet& b) {
    a.Swap(&b);
  }
  inline void Swap(Packet* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Packet* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Packet* New() const final {
    return CreateMaybeMessage<Packet>(nullptr);
  }

  Packet* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Packet>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Packet& from);
  void MergeFrom(const Packet& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Packet* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "crdc.airi.Packet";
  }
  protected:
  explicit Packet(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eth_5fpacket_2eproto);
    return ::descriptor_table_eth_5fpacket_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kDataFieldNumber = 2,
    kSizeFieldNumber = 3,
    kPortFieldNumber = 4,
    kTimeSystemFieldNumber = 5,
    kVersionFieldNumber = 1,
  };
  // optional bytes data = 2;
  bool has_data() const;
  private:
  bool _internal_has_data() const;
  public:
  void clear_data();
  const std::string& data() const;
  void set_data(const std::string& value);
  void set_data(std::string&& value);
  void set_data(const char* value);
  void set_data(const void* value, size_t size);
  std::string* mutable_data();
  std::string* release_data();
  void set_allocated_data(std::string* data);
  private:
  const std::string& _internal_data() const;
  void _internal_set_data(const std::string& value);
  std::string* _internal_mutable_data();
  public:

  // optional uint32 size = 3;
  bool has_size() const;
  private:
  bool _internal_has_size() const;
  public:
  void clear_size();
  ::PROTOBUF_NAMESPACE_ID::uint32 size() const;
  void set_size(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_size() const;
  void _internal_set_size(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint32 port = 4;
  bool has_port() const;
  private:
  bool _internal_has_port() const;
  public:
  void clear_port();
  ::PROTOBUF_NAMESPACE_ID::uint32 port() const;
  void set_port(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_port() const;
  void _internal_set_port(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // optional uint64 time_system = 5;
  bool has_time_system() const;
  private:
  bool _internal_has_time_system() const;
  public:
  void clear_time_system();
  ::PROTOBUF_NAMESPACE_ID::uint64 time_system() const;
  void set_time_system(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_time_system() const;
  void _internal_set_time_system(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // optional uint32 version = 1 [default = 1];
  bool has_version() const;
  private:
  bool _internal_has_version() const;
  public:
  void clear_version();
  ::PROTOBUF_NAMESPACE_ID::uint32 version() const;
  void set_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_version() const;
  void _internal_set_version(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:crdc.airi.Packet)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr data_;
  ::PROTOBUF_NAMESPACE_ID::uint32 size_;
  ::PROTOBUF_NAMESPACE_ID::uint32 port_;
  ::PROTOBUF_NAMESPACE_ID::uint64 time_system_;
  ::PROTOBUF_NAMESPACE_ID::uint32 version_;
  friend struct ::TableStruct_eth_5fpacket_2eproto;
};
// -------------------------------------------------------------------

class Packets PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:crdc.airi.Packets) */ {
 public:
  inline Packets() : Packets(nullptr) {}
  virtual ~Packets();

  Packets(const Packets& from);
  Packets(Packets&& from) noexcept
    : Packets() {
    *this = ::std::move(from);
  }

  inline Packets& operator=(const Packets& from) {
    CopyFrom(from);
    return *this;
  }
  inline Packets& operator=(Packets&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const Packets& default_instance();

  static inline const Packets* internal_default_instance() {
    return reinterpret_cast<const Packets*>(
               &_Packets_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Packets& a, Packets& b) {
    a.Swap(&b);
  }
  inline void Swap(Packets* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Packets* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Packets* New() const final {
    return CreateMaybeMessage<Packets>(nullptr);
  }

  Packets* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Packets>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Packets& from);
  void MergeFrom(const Packets& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Packets* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "crdc.airi.Packets";
  }
  protected:
  explicit Packets(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_eth_5fpacket_2eproto);
    return ::descriptor_table_eth_5fpacket_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kPacketFieldNumber = 1,
  };
  // repeated .crdc.airi.Packet packet = 1;
  int packet_size() const;
  private:
  int _internal_packet_size() const;
  public:
  void clear_packet();
  ::crdc::airi::Packet* mutable_packet(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::crdc::airi::Packet >*
      mutable_packet();
  private:
  const ::crdc::airi::Packet& _internal_packet(int index) const;
  ::crdc::airi::Packet* _internal_add_packet();
  public:
  const ::crdc::airi::Packet& packet(int index) const;
  ::crdc::airi::Packet* add_packet();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::crdc::airi::Packet >&
      packet() const;

  // @@protoc_insertion_point(class_scope:crdc.airi.Packets)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::crdc::airi::Packet > packet_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_eth_5fpacket_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Packet

// optional uint32 version = 1 [default = 1];
inline bool Packet::_internal_has_version() const {
  bool value = (_has_bits_[0] & 0x00000010u) != 0;
  return value;
}
inline bool Packet::has_version() const {
  return _internal_has_version();
}
inline void Packet::clear_version() {
  version_ = 1u;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Packet::_internal_version() const {
  return version_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Packet::version() const {
  // @@protoc_insertion_point(field_get:crdc.airi.Packet.version)
  return _internal_version();
}
inline void Packet::_internal_set_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000010u;
  version_ = value;
}
inline void Packet::set_version(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_version(value);
  // @@protoc_insertion_point(field_set:crdc.airi.Packet.version)
}

// optional bytes data = 2;
inline bool Packet::_internal_has_data() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Packet::has_data() const {
  return _internal_has_data();
}
inline void Packet::clear_data() {
  data_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& Packet::data() const {
  // @@protoc_insertion_point(field_get:crdc.airi.Packet.data)
  return _internal_data();
}
inline void Packet::set_data(const std::string& value) {
  _internal_set_data(value);
  // @@protoc_insertion_point(field_set:crdc.airi.Packet.data)
}
inline std::string* Packet::mutable_data() {
  // @@protoc_insertion_point(field_mutable:crdc.airi.Packet.data)
  return _internal_mutable_data();
}
inline const std::string& Packet::_internal_data() const {
  return data_.Get();
}
inline void Packet::_internal_set_data(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  data_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArena());
}
inline void Packet::set_data(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  data_.Set(
    ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:crdc.airi.Packet.data)
}
inline void Packet::set_data(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  data_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(value), GetArena());
  // @@protoc_insertion_point(field_set_char:crdc.airi.Packet.data)
}
inline void Packet::set_data(const void* value,
    size_t size) {
  _has_bits_[0] |= 0x00000001u;
  data_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:crdc.airi.Packet.data)
}
inline std::string* Packet::_internal_mutable_data() {
  _has_bits_[0] |= 0x00000001u;
  return data_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArena());
}
inline std::string* Packet::release_data() {
  // @@protoc_insertion_point(field_release:crdc.airi.Packet.data)
  if (!_internal_has_data()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return data_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void Packet::set_allocated_data(std::string* data) {
  if (data != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  data_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), data,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:crdc.airi.Packet.data)
}

// optional uint32 size = 3;
inline bool Packet::_internal_has_size() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  return value;
}
inline bool Packet::has_size() const {
  return _internal_has_size();
}
inline void Packet::clear_size() {
  size_ = 0u;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Packet::_internal_size() const {
  return size_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Packet::size() const {
  // @@protoc_insertion_point(field_get:crdc.airi.Packet.size)
  return _internal_size();
}
inline void Packet::_internal_set_size(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000002u;
  size_ = value;
}
inline void Packet::set_size(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_size(value);
  // @@protoc_insertion_point(field_set:crdc.airi.Packet.size)
}

// optional uint32 port = 4;
inline bool Packet::_internal_has_port() const {
  bool value = (_has_bits_[0] & 0x00000004u) != 0;
  return value;
}
inline bool Packet::has_port() const {
  return _internal_has_port();
}
inline void Packet::clear_port() {
  port_ = 0u;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Packet::_internal_port() const {
  return port_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 Packet::port() const {
  // @@protoc_insertion_point(field_get:crdc.airi.Packet.port)
  return _internal_port();
}
inline void Packet::_internal_set_port(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _has_bits_[0] |= 0x00000004u;
  port_ = value;
}
inline void Packet::set_port(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_port(value);
  // @@protoc_insertion_point(field_set:crdc.airi.Packet.port)
}

// optional uint64 time_system = 5;
inline bool Packet::_internal_has_time_system() const {
  bool value = (_has_bits_[0] & 0x00000008u) != 0;
  return value;
}
inline bool Packet::has_time_system() const {
  return _internal_has_time_system();
}
inline void Packet::clear_time_system() {
  time_system_ = PROTOBUF_ULONGLONG(0);
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Packet::_internal_time_system() const {
  return time_system_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 Packet::time_system() const {
  // @@protoc_insertion_point(field_get:crdc.airi.Packet.time_system)
  return _internal_time_system();
}
inline void Packet::_internal_set_time_system(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _has_bits_[0] |= 0x00000008u;
  time_system_ = value;
}
inline void Packet::set_time_system(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_time_system(value);
  // @@protoc_insertion_point(field_set:crdc.airi.Packet.time_system)
}

// -------------------------------------------------------------------

// Packets

// repeated .crdc.airi.Packet packet = 1;
inline int Packets::_internal_packet_size() const {
  return packet_.size();
}
inline int Packets::packet_size() const {
  return _internal_packet_size();
}
inline void Packets::clear_packet() {
  packet_.Clear();
}
inline ::crdc::airi::Packet* Packets::mutable_packet(int index) {
  // @@protoc_insertion_point(field_mutable:crdc.airi.Packets.packet)
  return packet_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::crdc::airi::Packet >*
Packets::mutable_packet() {
  // @@protoc_insertion_point(field_mutable_list:crdc.airi.Packets.packet)
  return &packet_;
}
inline const ::crdc::airi::Packet& Packets::_internal_packet(int index) const {
  return packet_.Get(index);
}
inline const ::crdc::airi::Packet& Packets::packet(int index) const {
  // @@protoc_insertion_point(field_get:crdc.airi.Packets.packet)
  return _internal_packet(index);
}
inline ::crdc::airi::Packet* Packets::_internal_add_packet() {
  return packet_.Add();
}
inline ::crdc::airi::Packet* Packets::add_packet() {
  // @@protoc_insertion_point(field_add:crdc.airi.Packets.packet)
  return _internal_add_packet();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::crdc::airi::Packet >&
Packets::packet() const {
  // @@protoc_insertion_point(field_list:crdc.airi.Packets.packet)
  return packet_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace airi
}  // namespace crdc

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_eth_5fpacket_2eproto
