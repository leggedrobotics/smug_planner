// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ortools/constraint_solver/local_search_stats.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3013000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3013000 < PROTOBUF_MIN_PROTOC_VERSION
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
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto;
namespace operations_research {
class LocalSearchStatistics;
class LocalSearchStatisticsDefaultTypeInternal;
extern LocalSearchStatisticsDefaultTypeInternal _LocalSearchStatistics_default_instance_;
class LocalSearchStatistics_LocalSearchOperatorStatistics;
class LocalSearchStatistics_LocalSearchOperatorStatisticsDefaultTypeInternal;
extern LocalSearchStatistics_LocalSearchOperatorStatisticsDefaultTypeInternal _LocalSearchStatistics_LocalSearchOperatorStatistics_default_instance_;
}  // namespace operations_research
PROTOBUF_NAMESPACE_OPEN
template<> ::operations_research::LocalSearchStatistics* Arena::CreateMaybeMessage<::operations_research::LocalSearchStatistics>(Arena*);
template<> ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics* Arena::CreateMaybeMessage<::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace operations_research {

// ===================================================================

class LocalSearchStatistics_LocalSearchOperatorStatistics PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics) */ {
 public:
  inline LocalSearchStatistics_LocalSearchOperatorStatistics() : LocalSearchStatistics_LocalSearchOperatorStatistics(nullptr) {}
  virtual ~LocalSearchStatistics_LocalSearchOperatorStatistics();

  LocalSearchStatistics_LocalSearchOperatorStatistics(const LocalSearchStatistics_LocalSearchOperatorStatistics& from);
  LocalSearchStatistics_LocalSearchOperatorStatistics(LocalSearchStatistics_LocalSearchOperatorStatistics&& from) noexcept
    : LocalSearchStatistics_LocalSearchOperatorStatistics() {
    *this = ::std::move(from);
  }

  inline LocalSearchStatistics_LocalSearchOperatorStatistics& operator=(const LocalSearchStatistics_LocalSearchOperatorStatistics& from) {
    CopyFrom(from);
    return *this;
  }
  inline LocalSearchStatistics_LocalSearchOperatorStatistics& operator=(LocalSearchStatistics_LocalSearchOperatorStatistics&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
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
  static const LocalSearchStatistics_LocalSearchOperatorStatistics& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LocalSearchStatistics_LocalSearchOperatorStatistics* internal_default_instance() {
    return reinterpret_cast<const LocalSearchStatistics_LocalSearchOperatorStatistics*>(
               &_LocalSearchStatistics_LocalSearchOperatorStatistics_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LocalSearchStatistics_LocalSearchOperatorStatistics& a, LocalSearchStatistics_LocalSearchOperatorStatistics& b) {
    a.Swap(&b);
  }
  inline void Swap(LocalSearchStatistics_LocalSearchOperatorStatistics* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(LocalSearchStatistics_LocalSearchOperatorStatistics* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LocalSearchStatistics_LocalSearchOperatorStatistics* New() const final {
    return CreateMaybeMessage<LocalSearchStatistics_LocalSearchOperatorStatistics>(nullptr);
  }

  LocalSearchStatistics_LocalSearchOperatorStatistics* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LocalSearchStatistics_LocalSearchOperatorStatistics>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LocalSearchStatistics_LocalSearchOperatorStatistics& from);
  void MergeFrom(const LocalSearchStatistics_LocalSearchOperatorStatistics& from);
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
  void InternalSwap(LocalSearchStatistics_LocalSearchOperatorStatistics* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics";
  }
  protected:
  explicit LocalSearchStatistics_LocalSearchOperatorStatistics(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto);
    return ::descriptor_table_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLocalSearchOperatorFieldNumber = 1,
    kNumNeighborsFieldNumber = 2,
    kNumFilteredNeighborsFieldNumber = 3,
    kNumAcceptedNeighborsFieldNumber = 4,
    kDurationSecondsFieldNumber = 5,
  };
  // string local_search_operator = 1;
  void clear_local_search_operator();
  const std::string& local_search_operator() const;
  void set_local_search_operator(const std::string& value);
  void set_local_search_operator(std::string&& value);
  void set_local_search_operator(const char* value);
  void set_local_search_operator(const char* value, size_t size);
  std::string* mutable_local_search_operator();
  std::string* release_local_search_operator();
  void set_allocated_local_search_operator(std::string* local_search_operator);
  private:
  const std::string& _internal_local_search_operator() const;
  void _internal_set_local_search_operator(const std::string& value);
  std::string* _internal_mutable_local_search_operator();
  public:

  // int64 num_neighbors = 2;
  void clear_num_neighbors();
  ::PROTOBUF_NAMESPACE_ID::int64 num_neighbors() const;
  void set_num_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int64 _internal_num_neighbors() const;
  void _internal_set_num_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value);
  public:

  // int64 num_filtered_neighbors = 3;
  void clear_num_filtered_neighbors();
  ::PROTOBUF_NAMESPACE_ID::int64 num_filtered_neighbors() const;
  void set_num_filtered_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int64 _internal_num_filtered_neighbors() const;
  void _internal_set_num_filtered_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value);
  public:

  // int64 num_accepted_neighbors = 4;
  void clear_num_accepted_neighbors();
  ::PROTOBUF_NAMESPACE_ID::int64 num_accepted_neighbors() const;
  void set_num_accepted_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::int64 _internal_num_accepted_neighbors() const;
  void _internal_set_num_accepted_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value);
  public:

  // double duration_seconds = 5;
  void clear_duration_seconds();
  double duration_seconds() const;
  void set_duration_seconds(double value);
  private:
  double _internal_duration_seconds() const;
  void _internal_set_duration_seconds(double value);
  public:

  // @@protoc_insertion_point(class_scope:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr local_search_operator_;
  ::PROTOBUF_NAMESPACE_ID::int64 num_neighbors_;
  ::PROTOBUF_NAMESPACE_ID::int64 num_filtered_neighbors_;
  ::PROTOBUF_NAMESPACE_ID::int64 num_accepted_neighbors_;
  double duration_seconds_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto;
};
// -------------------------------------------------------------------

class LocalSearchStatistics PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:operations_research.LocalSearchStatistics) */ {
 public:
  inline LocalSearchStatistics() : LocalSearchStatistics(nullptr) {}
  virtual ~LocalSearchStatistics();

  LocalSearchStatistics(const LocalSearchStatistics& from);
  LocalSearchStatistics(LocalSearchStatistics&& from) noexcept
    : LocalSearchStatistics() {
    *this = ::std::move(from);
  }

  inline LocalSearchStatistics& operator=(const LocalSearchStatistics& from) {
    CopyFrom(from);
    return *this;
  }
  inline LocalSearchStatistics& operator=(LocalSearchStatistics&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
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
  static const LocalSearchStatistics& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LocalSearchStatistics* internal_default_instance() {
    return reinterpret_cast<const LocalSearchStatistics*>(
               &_LocalSearchStatistics_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LocalSearchStatistics& a, LocalSearchStatistics& b) {
    a.Swap(&b);
  }
  inline void Swap(LocalSearchStatistics* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(LocalSearchStatistics* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LocalSearchStatistics* New() const final {
    return CreateMaybeMessage<LocalSearchStatistics>(nullptr);
  }

  LocalSearchStatistics* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LocalSearchStatistics>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LocalSearchStatistics& from);
  void MergeFrom(const LocalSearchStatistics& from);
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
  void InternalSwap(LocalSearchStatistics* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "operations_research.LocalSearchStatistics";
  }
  protected:
  explicit LocalSearchStatistics(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto);
    return ::descriptor_table_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  typedef LocalSearchStatistics_LocalSearchOperatorStatistics LocalSearchOperatorStatistics;

  // accessors -------------------------------------------------------

  enum : int {
    kLocalSearchOperatorStatisticsFieldNumber = 1,
  };
  // repeated .operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics local_search_operator_statistics = 1;
  int local_search_operator_statistics_size() const;
  private:
  int _internal_local_search_operator_statistics_size() const;
  public:
  void clear_local_search_operator_statistics();
  ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics* mutable_local_search_operator_statistics(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics >*
      mutable_local_search_operator_statistics();
  private:
  const ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics& _internal_local_search_operator_statistics(int index) const;
  ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics* _internal_add_local_search_operator_statistics();
  public:
  const ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics& local_search_operator_statistics(int index) const;
  ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics* add_local_search_operator_statistics();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics >&
      local_search_operator_statistics() const;

  // @@protoc_insertion_point(class_scope:operations_research.LocalSearchStatistics)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics > local_search_operator_statistics_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LocalSearchStatistics_LocalSearchOperatorStatistics

// string local_search_operator = 1;
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::clear_local_search_operator() {
  local_search_operator_.ClearToEmpty(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline const std::string& LocalSearchStatistics_LocalSearchOperatorStatistics::local_search_operator() const {
  // @@protoc_insertion_point(field_get:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
  return _internal_local_search_operator();
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_local_search_operator(const std::string& value) {
  _internal_set_local_search_operator(value);
  // @@protoc_insertion_point(field_set:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
}
inline std::string* LocalSearchStatistics_LocalSearchOperatorStatistics::mutable_local_search_operator() {
  // @@protoc_insertion_point(field_mutable:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
  return _internal_mutable_local_search_operator();
}
inline const std::string& LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_local_search_operator() const {
  return local_search_operator_.Get();
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_set_local_search_operator(const std::string& value) {
  
  local_search_operator_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value, GetArena());
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_local_search_operator(std::string&& value) {
  
  local_search_operator_.Set(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value), GetArena());
  // @@protoc_insertion_point(field_set_rvalue:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_local_search_operator(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  
  local_search_operator_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value),
              GetArena());
  // @@protoc_insertion_point(field_set_char:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_local_search_operator(const char* value,
    size_t size) {
  
  local_search_operator_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(
      reinterpret_cast<const char*>(value), size), GetArena());
  // @@protoc_insertion_point(field_set_pointer:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
}
inline std::string* LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_mutable_local_search_operator() {
  
  return local_search_operator_.Mutable(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline std::string* LocalSearchStatistics_LocalSearchOperatorStatistics::release_local_search_operator() {
  // @@protoc_insertion_point(field_release:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
  return local_search_operator_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_allocated_local_search_operator(std::string* local_search_operator) {
  if (local_search_operator != nullptr) {
    
  } else {
    
  }
  local_search_operator_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), local_search_operator,
      GetArena());
  // @@protoc_insertion_point(field_set_allocated:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.local_search_operator)
}

// int64 num_neighbors = 2;
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::clear_num_neighbors() {
  num_neighbors_ = PROTOBUF_LONGLONG(0);
}
inline ::PROTOBUF_NAMESPACE_ID::int64 LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_num_neighbors() const {
  return num_neighbors_;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 LocalSearchStatistics_LocalSearchOperatorStatistics::num_neighbors() const {
  // @@protoc_insertion_point(field_get:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.num_neighbors)
  return _internal_num_neighbors();
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_set_num_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value) {
  
  num_neighbors_ = value;
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_num_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _internal_set_num_neighbors(value);
  // @@protoc_insertion_point(field_set:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.num_neighbors)
}

// int64 num_filtered_neighbors = 3;
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::clear_num_filtered_neighbors() {
  num_filtered_neighbors_ = PROTOBUF_LONGLONG(0);
}
inline ::PROTOBUF_NAMESPACE_ID::int64 LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_num_filtered_neighbors() const {
  return num_filtered_neighbors_;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 LocalSearchStatistics_LocalSearchOperatorStatistics::num_filtered_neighbors() const {
  // @@protoc_insertion_point(field_get:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.num_filtered_neighbors)
  return _internal_num_filtered_neighbors();
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_set_num_filtered_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value) {
  
  num_filtered_neighbors_ = value;
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_num_filtered_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _internal_set_num_filtered_neighbors(value);
  // @@protoc_insertion_point(field_set:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.num_filtered_neighbors)
}

// int64 num_accepted_neighbors = 4;
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::clear_num_accepted_neighbors() {
  num_accepted_neighbors_ = PROTOBUF_LONGLONG(0);
}
inline ::PROTOBUF_NAMESPACE_ID::int64 LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_num_accepted_neighbors() const {
  return num_accepted_neighbors_;
}
inline ::PROTOBUF_NAMESPACE_ID::int64 LocalSearchStatistics_LocalSearchOperatorStatistics::num_accepted_neighbors() const {
  // @@protoc_insertion_point(field_get:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.num_accepted_neighbors)
  return _internal_num_accepted_neighbors();
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_set_num_accepted_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value) {
  
  num_accepted_neighbors_ = value;
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_num_accepted_neighbors(::PROTOBUF_NAMESPACE_ID::int64 value) {
  _internal_set_num_accepted_neighbors(value);
  // @@protoc_insertion_point(field_set:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.num_accepted_neighbors)
}

// double duration_seconds = 5;
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::clear_duration_seconds() {
  duration_seconds_ = 0;
}
inline double LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_duration_seconds() const {
  return duration_seconds_;
}
inline double LocalSearchStatistics_LocalSearchOperatorStatistics::duration_seconds() const {
  // @@protoc_insertion_point(field_get:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.duration_seconds)
  return _internal_duration_seconds();
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::_internal_set_duration_seconds(double value) {
  
  duration_seconds_ = value;
}
inline void LocalSearchStatistics_LocalSearchOperatorStatistics::set_duration_seconds(double value) {
  _internal_set_duration_seconds(value);
  // @@protoc_insertion_point(field_set:operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics.duration_seconds)
}

// -------------------------------------------------------------------

// LocalSearchStatistics

// repeated .operations_research.LocalSearchStatistics.LocalSearchOperatorStatistics local_search_operator_statistics = 1;
inline int LocalSearchStatistics::_internal_local_search_operator_statistics_size() const {
  return local_search_operator_statistics_.size();
}
inline int LocalSearchStatistics::local_search_operator_statistics_size() const {
  return _internal_local_search_operator_statistics_size();
}
inline void LocalSearchStatistics::clear_local_search_operator_statistics() {
  local_search_operator_statistics_.Clear();
}
inline ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics* LocalSearchStatistics::mutable_local_search_operator_statistics(int index) {
  // @@protoc_insertion_point(field_mutable:operations_research.LocalSearchStatistics.local_search_operator_statistics)
  return local_search_operator_statistics_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics >*
LocalSearchStatistics::mutable_local_search_operator_statistics() {
  // @@protoc_insertion_point(field_mutable_list:operations_research.LocalSearchStatistics.local_search_operator_statistics)
  return &local_search_operator_statistics_;
}
inline const ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics& LocalSearchStatistics::_internal_local_search_operator_statistics(int index) const {
  return local_search_operator_statistics_.Get(index);
}
inline const ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics& LocalSearchStatistics::local_search_operator_statistics(int index) const {
  // @@protoc_insertion_point(field_get:operations_research.LocalSearchStatistics.local_search_operator_statistics)
  return _internal_local_search_operator_statistics(index);
}
inline ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics* LocalSearchStatistics::_internal_add_local_search_operator_statistics() {
  return local_search_operator_statistics_.Add();
}
inline ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics* LocalSearchStatistics::add_local_search_operator_statistics() {
  // @@protoc_insertion_point(field_add:operations_research.LocalSearchStatistics.local_search_operator_statistics)
  return _internal_add_local_search_operator_statistics();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::operations_research::LocalSearchStatistics_LocalSearchOperatorStatistics >&
LocalSearchStatistics::local_search_operator_statistics() const {
  // @@protoc_insertion_point(field_list:operations_research.LocalSearchStatistics.local_search_operator_statistics)
  return local_search_operator_statistics_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace operations_research

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_ortools_2fconstraint_5fsolver_2flocal_5fsearch_5fstats_2eproto
