#include "intrinsic/shared_memory_manager/shared_memory_manager.hpp"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <sys/mman.h>

#include <iostream>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>
#include <chrono>

#include "intrinsic/utils/status.hpp"
#include "intrinsic/utils/time.hpp"
#include "intrinsic/flatbuffers/flatbuffer_utils.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_server.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_utils.hpp"
#include "intrinsic/shared_memory_manager/memory_segment.hpp"
#include "intrinsic/shared_memory_manager/segment_header.hpp"
#include "hwm_fbs/segment_info.fbs.h"
#include "intrinsic/shared_memory_manager/segment_info_utils.hpp"
#include "intrinsic/shared_memory_manager/testing/unique_segment_name.hpp"

namespace intrinsic::hal
{

namespace
{

using ::intrinsic_fbs::FlatbufferArrayNumElements;
using ::testing::Contains;
using ::testing::Eq;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::Key;
using ::testing::NotNull;
using ::testing::UnorderedElementsAre;

constexpr char kDefaultMemoryName[] = "some_int";

template<class T>
void TestInOutWithValue(
  SharedMemoryManager & shm_manager,
  std::string_view shm_name, const T & value)
{
  EXPECT_EQ(shm_manager.AddSegment<T>(shm_name, /*must_be_used=*/false, value).code,
            StatusCode::kOk);

  const T * val_out = shm_manager.GetSegmentValue<T>(shm_name);
  ASSERT_THAT(val_out, NotNull());
  EXPECT_THAT(*val_out, Eq(value));
}

template<class T>
void TestInOut(SharedMemoryManager & shm_manager, std::string_view shm_name)
{
  EXPECT_EQ(shm_manager.AddSegmentWithDefaultValue<T>(shm_name,
                                                      /*must_be_used=*/false).code,
            StatusCode::kOk);

  const T * val_out = shm_manager.GetSegmentValue<T>(shm_name);
  ASSERT_THAT(val_out, NotNull());
  EXPECT_THAT(*val_out, Eq(T()));
}

template<class T>
void TestInOut(
  SharedMemoryManager & shm_manager, std::string_view shm_name,
  const std::string & type_id)
{
  EXPECT_EQ(shm_manager.AddSegmentWithDefaultValue<T>(
      shm_name, /*must_be_used=*/false, type_id).code,
            StatusCode::kOk);

  const T * val_out = shm_manager.GetSegmentValue<T>(shm_name);
  auto header = shm_manager.GetSegmentHeader(shm_name);
  ASSERT_THAT(val_out, NotNull());
  EXPECT_THAT(*val_out, Eq(T()));
  EXPECT_THAT(header->Type(), Eq(SegmentHeader(type_id.c_str()).Type()))
      << header->Type().TypeID() << " vs "
      << SegmentHeader(type_id.c_str()).Type().TypeID();
}

// Convenience function to get a pointer to the segment data.
template<class T>
T * SegmentFromFdMap(
  SegmentNameToFileDescriptorMap fd_map,
  const std::string & shm_name)
{
  auto fd_it = fd_map.find(shm_name);
  if (fd_it == fd_map.end()) {
    return nullptr;
  }
  int shm_fd = fd_it->second;

  if (shm_fd < 0) {
    return nullptr;
  }
  auto * data = static_cast<uint8_t *>(
    mmap(nullptr, sizeof(SegmentHeader) + sizeof(T), PROT_READ | PROT_WRITE,
           MAP_SHARED, shm_fd, 0));
  return reinterpret_cast<T *>(data + sizeof(SegmentHeader));
}

TEST(SharedMemorymanagerTest, AddPrimitives) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "module_name");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;

  TestInOutWithValue<int>(*shm_manager, "some_int", 123);
  TestInOutWithValue<char>(*shm_manager, "some_char", '1');
  TestInOutWithValue<double>(*shm_manager, "some_double", 1.23);
  TestInOutWithValue<bool>(*shm_manager, "some_bool", true);
  TestInOutWithValue<std::array<int, 3>>(*shm_manager, "some_array", {1, 2, 3});
  // Compiler error for std::vector<T> as it's not trivially copyable.
  // TestInOut<std::vector<int>>(shm_manager, "/some_vector", {1, 2, 3});
  // Compiler error for any pointer type, not trivially copyable either.
  // TestInOut<int*>(shm_manager, "/some_pointer", new int(123));
  auto memory_names = shm_manager->GetRegisteredMemoryNames();
  EXPECT_THAT(memory_names.size(), Eq(5));
  EXPECT_THAT(memory_names,
              UnorderedElementsAre("some_int", "some_char", "some_double",
                                   "some_bool", "some_array"));
  auto segment_info = shm_manager->GetSegmentInfo();
  EXPECT_THAT(segment_info.size(), 5);
  auto names = intrinsic::hal::GetNamesFromSegmentInfo(segment_info);
  ASSERT_TRUE(names.has_value()) << names.error();
  EXPECT_THAT(*names,
              UnorderedElementsAre(
                  HasSubstr("some_int"), HasSubstr("some_char"),
                  HasSubstr("some_double"), HasSubstr("some_bool"),
                  HasSubstr("some_array")));
}

TEST(SharedMemorymanagerTest, AddDefaultValues) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "module_name");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  TestInOut<int>(*shm_manager, "some_int");
  TestInOut<char>(*shm_manager, "some_char");
  TestInOut<double>(*shm_manager, "some_double");
  TestInOut<bool>(*shm_manager, "some_bool");
  TestInOut<std::array<int, 3>>(*shm_manager, "some_array");
  TestInOut<std::array<uint64_t, 1000>>(*shm_manager, "some_large_array");
  // Compiler error for std::vector<T> as it's not trivially copyable.
  // TestInOut<std::vector<int>>(shm_manager, "/some_vector");
  // Compiler error for any pointer type, not trivially copyable either.
  // TestInOut<int*>(shm_manager, "/some_pointer");
}

TEST(SharedMemorymanagerTest, AddDefaultValuesWithTypeInfo) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "module_name");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  TestInOut<int>(*shm_manager, "some_int", "int_id");
  TestInOut<char>(*shm_manager, "some_char", "char_id");
  TestInOut<double>(*shm_manager, "some_double", "double_id");
  TestInOut<bool>(*shm_manager, "some_bool", "bool_id");
  TestInOut<std::array<int, 3>>(*shm_manager, "some_array", "array_id");
  TestInOut<std::array<uint64_t, 1000>>(*shm_manager, "some_large_array",
                                        "some_large_array_id");
  //  Compiler error for std::vector<T> as it's not trivially copyable.
  //  TestInOut<std::vector<int>>(shm_manager, "/some_vector");
  //  Compiler error for any pointer type, not trivially copyable either.
  //  TestInOut<int*>(shm_manager, "/some_pointer");
}

TEST(SharedMemorymanagerTest, AddMoveOnlyTypes) {
  struct MoveOnly
  {
    explicit MoveOnly(int val)
    : value(val) {}
    MoveOnly(const MoveOnly & other) = delete;
    MoveOnly & operator=(const MoveOnly & other) = delete;
    MoveOnly(MoveOnly && other) = default;
    MoveOnly & operator=(MoveOnly && other) = default;
    int value;
  };


  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  const std::string move_only_shm_name = "some_move_only";
  EXPECT_EQ(shm_manager->AddSegment(move_only_shm_name, /*must_be_used=*/false,
                                    MoveOnly(13)).code,
            StatusCode::kOk);
  const MoveOnly * move_only_instance =
    shm_manager->GetSegmentValue<MoveOnly>(move_only_shm_name);
  EXPECT_THAT(move_only_instance->value, Eq(13));

  MoveOnly move_only_instance2(26);
  const std::string move_only_shm_name2 = "another_move_only";
  EXPECT_EQ(shm_manager->AddSegment(move_only_shm_name2, /*must_be_used=*/false,
                                    std::move(move_only_instance2)).code,
            StatusCode::kOk);
  const MoveOnly * move_only_instance2_out =
    shm_manager->GetSegmentValue<MoveOnly>(move_only_shm_name2);
  EXPECT_THAT(move_only_instance2_out->value, Eq(26));
}

TEST(SharedMemorymanagerTest, GetHeader) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  EXPECT_EQ(shm_manager->AddSegmentWithDefaultValue<int>(
      kDefaultMemoryName, /*must_be_used=*/false).code, StatusCode::kOk);

  const SegmentHeader * header =
    shm_manager->GetSegmentHeader(kDefaultMemoryName);
  ASSERT_THAT(header, NotNull());
  EXPECT_THAT(header->ReaderRefCount(), Eq(0));
  EXPECT_THAT(header->WriterRefCount(), Eq(0));
}

TEST(SharedMemorymanagerTest, ModifyRawValue) {
  constexpr int kBufferSize = 5;
  std::string_view kBufferName = "some_buffer_id";
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  EXPECT_EQ(shm_manager->AddSegment(kBufferName, /*must_be_used=*/false,
                                    kBufferSize).code,
            StatusCode::kOk);

  std::vector<uint8_t> source_data = {'a', 'b', 'c', 'd', 'e'};
  uint8_t * shm_data = shm_manager->GetRawValue(kBufferName);
  std::memcpy(shm_data, source_data.data(), kBufferSize);

  // We can't unfortunately use `ElementsAreArray` because the returned value
  // from the shm_manager is a pointer to a c-style array.
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[0], Eq('a'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[1], Eq('b'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[2], Eq('c'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[3], Eq('d'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[4], Eq('e'));
}

TEST(SharedMemorymanagerTest, HeaderCleanupOnManagerExit) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  SegmentNameToFileDescriptorMap segment_name_to_file_descriptor_map;
  {
    auto expected_shm_manager =
      SharedMemoryManager::Create(UniqueMemoryNamespace(), module_name);
    ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
    auto & shm_manager = *expected_shm_manager;
    EXPECT_EQ(shm_manager->AddSegmentWithDefaultValue<int>(
          kDefaultMemoryName, /*must_be_used=*/false).code,
                StatusCode::kOk);

    const SegmentHeader * header =
      shm_manager->GetSegmentHeader(kDefaultMemoryName);
    ASSERT_THAT(header, NotNull());
    EXPECT_THAT(header->ReaderRefCount(), Eq(0));
    EXPECT_THAT(header->WriterRefCount(), Eq(0));
    segment_name_to_file_descriptor_map =
      shm_manager->SegmentNameToFileDescriptorMap();
  }
  // Confirms that the destructor closed the file descriptors.
  for (const auto & [name, fd] : segment_name_to_file_descriptor_map) {
    EXPECT_EQ(close(fd), -1)
        << "File descriptor for " << name << "was not already closed.";
  }
}

TEST(SharedMemoryManager, InsertMultipleT) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;

  EXPECT_EQ(shm_manager->AddSegmentWithDefaultValue<int>(
      "int1", /*must_be_used=*/false).code,
            StatusCode::kOk);
  EXPECT_EQ(shm_manager->AddSegmentWithDefaultValue<int>(
      "int2", /*must_be_used=*/false).code,
            StatusCode::kOk);
}

TEST(SharedMemoryManager, FailOnDoubleInsertion) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;

  EXPECT_EQ(shm_manager->AddSegmentWithDefaultValue<int>(
      kDefaultMemoryName, /*must_be_used=*/false).code,
            StatusCode::kOk);
  EXPECT_EQ(shm_manager->AddSegmentWithDefaultValue<int>(
                  kDefaultMemoryName, /*must_be_used=*/false).code,
              StatusCode::kAlreadyExists);
}

TEST(SharedMemoryManager, FailOnWrongSegmentName) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;

  std::string_view empty_name = "";
  {
    auto add_segment_status = shm_manager->AddSegmentWithDefaultValue<int>(
        empty_name, /*must_be_used=*/false);
    EXPECT_EQ(add_segment_status.code,
              StatusCode::kInvalidArgument);
    EXPECT_THAT(add_segment_status.message, HasSubstr("empty"));
  }
  std::string name_over_max_length(std::string(
      FlatbufferArrayNumElements(&intrinsic_fbs::SegmentName::value), 'i'));
  {
    auto add_segment_status = shm_manager->AddSegmentWithDefaultValue<int>(
      name_over_max_length, /*must_be_used=*/false);
    EXPECT_EQ(add_segment_status.code, StatusCode::kInvalidArgument);
    EXPECT_THAT(add_segment_status.message,
              HasSubstr(name_over_max_length));
  }
  std::string_view name_with_multiple_slashes = "/in/trin/sic";
  {
    auto add_segment_status = shm_manager->AddSegmentWithDefaultValue<int>(
        name_with_multiple_slashes, /*must_be_used=*/false);
    EXPECT_EQ(add_segment_status.code,
              StatusCode::kInvalidArgument);
    EXPECT_THAT(add_segment_status.message,
                HasSubstr(name_with_multiple_slashes));
  }
}

TEST(SharedMemoryManager, FailOnSegmentNotFound) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;

  // We haven't inserted anything in the shm_manager yet, so a call to
  // `GetSegmentValue` returns `nullptr`.
  EXPECT_THAT(shm_manager->GetSegmentValue<int>(kDefaultMemoryName), IsNull());
}

TEST(SharedMemoryManager, TestMultipleInstances) {
  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), "some_module");
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  EXPECT_EQ(shm_manager->AddSegmentWithDefaultValue<int>(
      kDefaultMemoryName, /*must_be_used=*/false).code,
            StatusCode::kOk);

  const int * val1 = shm_manager->GetSegmentValue<int>(kDefaultMemoryName);
  ASSERT_THAT(val1, NotNull());

  const int * val2 = shm_manager->GetSegmentValue<int>(kDefaultMemoryName);
  ASSERT_THAT(val2, NotNull());

  EXPECT_THAT(*val1, Eq(*val2));
  EXPECT_EQ(shm_manager->SetSegmentValue(kDefaultMemoryName, 42).code,
            StatusCode::kOk);
  EXPECT_THAT(*val1, Eq(42));
  EXPECT_THAT(*val1, Eq(*val2));
}

TEST(SharedMemoryManager, SegmentNameToFileDescriptorMapWorks) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), module_name);
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  EXPECT_EQ(shm_manager->AddSegment<int>(kDefaultMemoryName,
                                         /*must_be_used=*/false, 42).code,
            StatusCode::kOk);

  EXPECT_THAT(shm_manager->SegmentNameToFileDescriptorMap(),
              Contains(Key(kDefaultMemoryName)));
}

TEST(SharedMemoryManager, TestExternalAccess) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), module_name);
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  EXPECT_EQ(shm_manager->AddSegment<int>(kDefaultMemoryName,
                                         /*must_be_used=*/false, 42).code,
            StatusCode::kOk);

  const int * val1 = shm_manager->GetSegmentValue<int>(kDefaultMemoryName);
  ASSERT_THAT(val1, NotNull());
  EXPECT_THAT(*val1, Eq(42));

  auto expected_domain_socket_server =
    DomainSocketServer::Create(
          /*socket_directory=*/SocketDirectoryFromNamespace(memory_namespace),
          /*module_name=*/module_name,
          DomainSocketServer::kDefaultLockAcquireTimeout);
  ASSERT_TRUE(expected_domain_socket_server.has_value())
      << expected_domain_socket_server.error();
  auto & domain_socket_server = *expected_domain_socket_server;

  ASSERT_EQ(
      domain_socket_server->AddSegmentInfoServeShmDescriptors(*shm_manager).code,
      StatusCode::kOk);

  auto expected_segment_name_to_file_descriptor_map =
    ::intrinsic::hal::GetSegmentNameToFileDescriptorMap(
           SocketDirectoryFromNamespace(memory_namespace),
           module_name, std::chrono::seconds(0));
  ASSERT_TRUE(expected_segment_name_to_file_descriptor_map.has_value())
      << expected_segment_name_to_file_descriptor_map.error();
  const auto & segment_name_to_file_descriptor_map = *expected_segment_name_to_file_descriptor_map;

  int * val2 = SegmentFromFdMap<int>(segment_name_to_file_descriptor_map,
                                    kDefaultMemoryName);
  ASSERT_THAT(val2, NotNull());
  EXPECT_THAT(*val1, Eq(*val2));

  EXPECT_EQ(shm_manager->SetSegmentValue(kDefaultMemoryName, 1337).code,
            StatusCode::kOk);
  EXPECT_THAT(*val1, Eq(1337));
  EXPECT_THAT(*val1, Eq(*val2));

  *val2 = 1338;
  EXPECT_THAT(*val1, Eq(*val2));
  EXPECT_THAT(*val1, Eq(1338));
}

TEST(SharedMemoryManager, GetWorksAndChecksName) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), module_name);
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;

  {
    auto result = shm_manager->Get<ReadWriteMemorySegment<int>>("invalid_name");
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code,
              StatusCode::kNotFound);
  }
  EXPECT_EQ(shm_manager->AddSegment<int>(kDefaultMemoryName,
                                         /*must_be_used=*/false, 42).code,
            StatusCode::kOk);
  {
    auto result =
      shm_manager->Get<ReadWriteMemorySegment<int>>(kDefaultMemoryName);
    ASSERT_TRUE(result.has_value()) << result.error();
  }
}

TEST(SharedMemoryManager, ModuleNameWorks) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  auto expected_shm_manager =
    SharedMemoryManager::Create(UniqueMemoryNamespace(), module_name);
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  EXPECT_EQ(shm_manager->ModuleName(), module_name);
}

TEST(SharedMemoryManager, SharedMemoryNamespaceWorks) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  auto expected_shm_manager =
    SharedMemoryManager::Create(memory_namespace, module_name);
  ASSERT_TRUE(expected_shm_manager.has_value()) << expected_shm_manager.error();
  auto & shm_manager = *expected_shm_manager;
  EXPECT_EQ(shm_manager->SharedMemoryNamespace(), memory_namespace);
}

TEST(SharedMemoryManager, CreateErrorsOnEmpyModuleName) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  auto expected_shm_manager = SharedMemoryManager::Create(memory_namespace, /*module_name=*/"");
  ASSERT_FALSE(expected_shm_manager.has_value());
  EXPECT_EQ(expected_shm_manager.error().code,
            StatusCode::kInvalidArgument);
  EXPECT_THAT(expected_shm_manager.error().message, HasSubstr("name"));
}

}  // namespace
}  // namespace intrinsic::hal

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
