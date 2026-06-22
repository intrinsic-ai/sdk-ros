#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"

#include <fcntl.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sys/mman.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "flatbuffer_definitions/icon/interprocess/shared_memory_manager/segment_info.fbs.h"
#include "icon/flatbuffers/flatbuffer_utils.h"
#include "icon/interprocess/shared_memory_manager/domain_socket_server.h"
#include "icon/interprocess/shared_memory_manager/domain_socket_utils.h"
#include "icon/interprocess/shared_memory_manager/memory_segment.h"
#include "icon/interprocess/shared_memory_manager/segment_header.h"
#include "icon/interprocess/shared_memory_manager/segment_info_utils.h"
#include "icon/interprocess/shared_memory_manager/testing/unique_segment_name.h"
#include "icon/utils/log.h"
#include "icon/utils/mock_log_sink.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_test_macros.h"
#include "icon/utils/time.h"

namespace intrinsic::icon {

namespace {

using ::intrinsic_fbs::FlatbufferArrayNumElements;
using ::testing::Contains;
using ::testing::Eq;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::Key;
using ::testing::NotNull;
using ::testing::UnorderedElementsAre;

constexpr char kDefaultMemoryName[] = "some_int";

template <class T>
void TestInOutWithValue(SharedMemoryManager& shm_manager,
                        std::string_view shm_name, const T& value) {
  INTR_ASSERT_OK(
      shm_manager.AddSegment<T>(shm_name, /*must_be_used=*/false, value));

  const T* val_out = shm_manager.GetSegmentValue<T>(shm_name);
  ASSERT_THAT(val_out, NotNull());
  EXPECT_THAT(*val_out, Eq(value));
}

template <class T>
void TestInOut(SharedMemoryManager& shm_manager, std::string_view shm_name) {
  INTR_ASSERT_OK(
      shm_manager.AddSegmentWithDefaultValue<T>(shm_name,
                                                /*must_be_used=*/false));

  const T* val_out = shm_manager.GetSegmentValue<T>(shm_name);
  ASSERT_THAT(val_out, NotNull());
  EXPECT_THAT(*val_out, Eq(T()));
}

template <class T>
void TestInOut(SharedMemoryManager& shm_manager, std::string_view shm_name,
               const std::string& type_id) {
  INTR_ASSERT_OK(shm_manager.AddSegmentWithDefaultValue<T>(
      shm_name, /*must_be_used=*/false, type_id));

  const T* val_out = shm_manager.GetSegmentValue<T>(shm_name);
  auto header = shm_manager.GetSegmentHeader(shm_name);
  ASSERT_THAT(val_out, NotNull());
  EXPECT_THAT(*val_out, Eq(T()));
  EXPECT_THAT(header->Type(),
              Eq(SegmentHeader(type_id.c_str(), nullptr).Type()))
      << header->Type().TypeID() << " vs "
      << SegmentHeader(type_id.c_str(), nullptr).Type().TypeID();
}

// Convenience function to get a pointer to the segment data.
template <class T>
T* SegmentFromFdMap(SegmentNameToFileDescriptorMap fd_map,
                    const std::string& shm_name) {
  auto fd_it = fd_map.find(shm_name);
  if (fd_it == fd_map.end()) {
    return nullptr;
  }
  int shm_fd = fd_it->second;

  if (shm_fd < 0) {
    return nullptr;
  }
  auto* data = static_cast<uint8_t*>(
      mmap(nullptr, sizeof(SegmentHeader) + sizeof(T), PROT_READ | PROT_WRITE,
           MAP_SHARED, shm_fd, 0));
  return reinterpret_cast<T*>(data + sizeof(SegmentHeader));
}

class SharedMemoryManagerTest : public ::testing::Test {
 public:
  SharedMemoryManagerTest()
      : logger_(log::Logger::Severity::kInfo, log::MockSink(log_entries_)) {}

  log::Logger* logger() { return &logger_; }

  std::span<const log::LogEntryWithStorage> log_entries() {
    return log_entries_;
  }

 private:
  std::vector<log::LogEntryWithStorage> log_entries_;
  log::Logger logger_;
};

TEST_F(SharedMemoryManagerTest, AddPrimitives) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "module_name", logger()));

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
  INTR_ASSERT_OK_AND_ASSIGN(
      auto names, intrinsic::icon::GetNamesFromSegmentInfo(segment_info));
  EXPECT_THAT(names, UnorderedElementsAre(
                         HasSubstr("some_int"), HasSubstr("some_char"),
                         HasSubstr("some_double"), HasSubstr("some_bool"),
                         HasSubstr("some_array")));
}

TEST_F(SharedMemoryManagerTest, AddDefaultValues) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "module_name", logger()));
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

TEST_F(SharedMemoryManagerTest, AddDefaultValuesWithTypeInfo) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "module_name", logger()));
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

TEST_F(SharedMemoryManagerTest, AddMoveOnlyTypes) {
  struct MoveOnly {
    explicit MoveOnly(int val) : value(val) {}
    MoveOnly(const MoveOnly& other) = delete;
    MoveOnly& operator=(const MoveOnly& other) = delete;
    MoveOnly(MoveOnly&& other) = default;
    MoveOnly& operator=(MoveOnly&& other) = default;
    int value;
  };

  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));
  const std::string move_only_shm_name = "some_move_only";
  INTR_ASSERT_OK(shm_manager->AddSegment(move_only_shm_name,
                                         /*must_be_used=*/false, MoveOnly(13)));
  const MoveOnly* move_only_instance =
      shm_manager->GetSegmentValue<MoveOnly>(move_only_shm_name);
  EXPECT_THAT(move_only_instance->value, Eq(13));

  MoveOnly move_only_instance2(26);
  const std::string move_only_shm_name2 = "another_move_only";
  INTR_ASSERT_OK(shm_manager->AddSegment(move_only_shm_name2,
                                         /*must_be_used=*/false,
                                         std::move(move_only_instance2)));
  const MoveOnly* move_only_instance2_out =
      shm_manager->GetSegmentValue<MoveOnly>(move_only_shm_name2);
  EXPECT_THAT(move_only_instance2_out->value, Eq(26));
}

TEST_F(SharedMemoryManagerTest, GetHeader) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));
  INTR_ASSERT_OK(
      shm_manager->AddSegmentWithDefaultValue<int>(kDefaultMemoryName,
                                                   /*must_be_used=*/false));

  const SegmentHeader* header =
      shm_manager->GetSegmentHeader(kDefaultMemoryName);
  ASSERT_THAT(header, NotNull());
  EXPECT_THAT(header->ReaderRefCount(), Eq(0));
  EXPECT_THAT(header->WriterRefCount(), Eq(0));
}

TEST_F(SharedMemoryManagerTest, ModifyRawValue) {
  constexpr int kBufferSize = 5;
  std::string_view kBufferName = "some_buffer_id";
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));
  INTR_ASSERT_OK(shm_manager->AddSegment(kBufferName, /*must_be_used=*/false,
                                         kBufferSize));

  std::vector<uint8_t> source_data = {'a', 'b', 'c', 'd', 'e'};
  uint8_t* shm_data = shm_manager->GetRawValue(kBufferName);
  std::memcpy(shm_data, source_data.data(), kBufferSize);

  // We can't unfortunately use `ElementsAreArray` because the returned value
  // from the shm_manager is a pointer to a c-style array.
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[0], Eq('a'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[1], Eq('b'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[2], Eq('c'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[3], Eq('d'));
  EXPECT_THAT(shm_manager->GetRawValue(kBufferName)[4], Eq('e'));
}

TEST_F(SharedMemoryManagerTest, HeaderCleanupOnManagerExit) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  SegmentNameToFileDescriptorMap segment_name_to_file_descriptor_map;
  {
    INTR_ASSERT_OK_AND_ASSIGN(
        auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                      module_name, logger()));
    INTR_EXPECT_OK(
        shm_manager->AddSegmentWithDefaultValue<int>(kDefaultMemoryName,
                                                     /*must_be_used=*/false));

    const SegmentHeader* header =
        shm_manager->GetSegmentHeader(kDefaultMemoryName);
    ASSERT_THAT(header, NotNull());
    EXPECT_THAT(header->ReaderRefCount(), Eq(0));
    EXPECT_THAT(header->WriterRefCount(), Eq(0));
    segment_name_to_file_descriptor_map =
        shm_manager->SegmentNameToFileDescriptorMap();
  }
  // Confirms that the destructor closed the file descriptors.
  for (const auto& [name, fd] : segment_name_to_file_descriptor_map) {
    EXPECT_EQ(close(fd), -1)
        << "File descriptor for " << name << "was not already closed.";
  }
}

TEST_F(SharedMemoryManagerTest, InsertMultipleT) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));

  INTR_EXPECT_OK(shm_manager->AddSegmentWithDefaultValue<int>(
      "int1", /*must_be_used=*/false));
  INTR_EXPECT_OK(shm_manager->AddSegmentWithDefaultValue<int>(
      "int2", /*must_be_used=*/false));
}

TEST_F(SharedMemoryManagerTest, FailOnDoubleInsertion) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));

  INTR_EXPECT_OK(
      shm_manager->AddSegmentWithDefaultValue<int>(kDefaultMemoryName,
                                                   /*must_be_used=*/false));
  EXPECT_EQ(shm_manager
                ->AddSegmentWithDefaultValue<int>(kDefaultMemoryName,
                                                  /*must_be_used=*/false)
                .code,
            StatusCode::kAlreadyExists);
}

TEST_F(SharedMemoryManagerTest, FailOnWrongSegmentName) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));

  std::string_view empty_name = "";
  {
    auto add_segment_status = shm_manager->AddSegmentWithDefaultValue<int>(
        empty_name, /*must_be_used=*/false);
    EXPECT_EQ(add_segment_status.code, StatusCode::kInvalidArgument);
    EXPECT_THAT(add_segment_status.message, HasSubstr("empty"));
  }
  std::string name_over_max_length(std::string(
      FlatbufferArrayNumElements(&intrinsic_fbs::SegmentName::value), 'i'));
  {
    auto add_segment_status = shm_manager->AddSegmentWithDefaultValue<int>(
        name_over_max_length, /*must_be_used=*/false);
    EXPECT_EQ(add_segment_status.code, StatusCode::kInvalidArgument);
    EXPECT_THAT(add_segment_status.message, HasSubstr(name_over_max_length));
  }
  std::string_view name_with_multiple_slashes = "/in/trin/sic";
  {
    auto add_segment_status = shm_manager->AddSegmentWithDefaultValue<int>(
        name_with_multiple_slashes, /*must_be_used=*/false);
    EXPECT_EQ(add_segment_status.code, StatusCode::kInvalidArgument);
    EXPECT_THAT(add_segment_status.message,
                HasSubstr(name_with_multiple_slashes));
  }
}

TEST_F(SharedMemoryManagerTest, FailOnSegmentNotFound) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));

  // We haven't inserted anything in the shm_manager yet, so a call to
  // `GetSegmentValue` returns `nullptr`.
  EXPECT_THAT(shm_manager->GetSegmentValue<int>(kDefaultMemoryName), IsNull());
}

TEST_F(SharedMemoryManagerTest, TestMultipleInstances) {
  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager, SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                    "some_module", logger()));
  INTR_ASSERT_OK(
      shm_manager->AddSegmentWithDefaultValue<int>(kDefaultMemoryName,
                                                   /*must_be_used=*/false));

  const int* val1 = shm_manager->GetSegmentValue<int>(kDefaultMemoryName);
  ASSERT_THAT(val1, NotNull());

  const int* val2 = shm_manager->GetSegmentValue<int>(kDefaultMemoryName);
  ASSERT_THAT(val2, NotNull());

  EXPECT_THAT(*val1, Eq(*val2));
  INTR_EXPECT_OK(shm_manager->SetSegmentValue(kDefaultMemoryName, 42));
  EXPECT_THAT(*val1, Eq(42));
  EXPECT_THAT(*val1, Eq(*val2));
}

TEST_F(SharedMemoryManagerTest, SegmentNameToFileDescriptorMapWorks) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  INTR_ASSERT_OK_AND_ASSIGN(auto shm_manager,
                            SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                        module_name, logger()));
  INTR_EXPECT_OK(shm_manager->AddSegment<int>(kDefaultMemoryName,
                                              /*must_be_used=*/false, 42));

  EXPECT_THAT(shm_manager->SegmentNameToFileDescriptorMap(),
              Contains(Key(kDefaultMemoryName)));
}

TEST_F(SharedMemoryManagerTest, TestExternalAccess) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  INTR_ASSERT_OK_AND_ASSIGN(auto shm_manager,
                            SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                        module_name, logger()));
  INTR_ASSERT_OK(shm_manager->AddSegment<int>(kDefaultMemoryName,
                                              /*must_be_used=*/false, 42));

  const int* val1 = shm_manager->GetSegmentValue<int>(kDefaultMemoryName);
  ASSERT_THAT(val1, NotNull());
  EXPECT_THAT(*val1, Eq(42));

  INTR_ASSERT_OK_AND_ASSIGN(
      auto domain_socket_server,
      DomainSocketServer::Create(
          /*socket_directory=*/SocketDirectoryFromNamespace(memory_namespace),
          /*module_name=*/module_name,
          DomainSocketServer::kDefaultLockAcquireTimeout, logger()));

  INTR_ASSERT_OK(
      domain_socket_server->AddSegmentInfoServeShmDescriptors(*shm_manager));

  INTR_ASSERT_OK_AND_ASSIGN(
      auto segment_name_to_file_descriptor_map,
      ::intrinsic::icon::GetSegmentNameToFileDescriptorMap(
          SocketDirectoryFromNamespace(memory_namespace), module_name,
          std::chrono::seconds(0), logger()));

  int* val2 = SegmentFromFdMap<int>(segment_name_to_file_descriptor_map,
                                    kDefaultMemoryName);
  ASSERT_THAT(val2, NotNull());
  EXPECT_THAT(*val1, Eq(*val2));

  INTR_EXPECT_OK(shm_manager->SetSegmentValue(kDefaultMemoryName, 1337));
  EXPECT_THAT(*val1, Eq(1337));
  EXPECT_THAT(*val1, Eq(*val2));

  *val2 = 1338;
  EXPECT_THAT(*val1, Eq(*val2));
  EXPECT_THAT(*val1, Eq(1338));
}

TEST_F(SharedMemoryManagerTest, GetWorksAndChecksName) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  INTR_ASSERT_OK_AND_ASSIGN(auto shm_manager,
                            SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                        module_name, logger()));

  {
    auto result =
        shm_manager->Get<ReadWriteMemorySegment<int>>("invalid_name", logger());
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, StatusCode::kNotFound);
  }
  INTR_EXPECT_OK(shm_manager->AddSegment<int>(kDefaultMemoryName,
                                              /*must_be_used=*/false, 42));
  INTR_EXPECT_OK(shm_manager->Get<ReadWriteMemorySegment<int>>(
      kDefaultMemoryName, logger()));
}

TEST_F(SharedMemoryManagerTest, ModuleNameWorks) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  INTR_ASSERT_OK_AND_ASSIGN(auto shm_manager,
                            SharedMemoryManager::Create(UniqueMemoryNamespace(),
                                                        module_name, logger()));
  EXPECT_EQ(shm_manager->ModuleName(), module_name);
}

TEST_F(SharedMemoryManagerTest, SharedMemoryNamespaceWorks) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  INTR_ASSERT_OK_AND_ASSIGN(
      auto shm_manager,
      SharedMemoryManager::Create(memory_namespace, module_name, logger()));
  EXPECT_EQ(shm_manager->SharedMemoryNamespace(), memory_namespace);
}

TEST_F(SharedMemoryManagerTest, CreateErrorsOnEmpyModuleName) {
  std::string memory_namespace = UniqueMemoryNamespace();
  std::string module_name = "some_module";

  auto expected_shm_manager = SharedMemoryManager::Create(
      memory_namespace, /*module_name=*/"", logger());
  ASSERT_FALSE(expected_shm_manager.has_value());
  EXPECT_EQ(expected_shm_manager.error().code, StatusCode::kInvalidArgument);
  EXPECT_THAT(expected_shm_manager.error().message, HasSubstr("name"));
}

}  // namespace
}  // namespace intrinsic::icon

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
