#include <eval/task.hpp>

#include <gtest/gtest.h>

namespace {

struct DummyInputData : public lsfm::GenericInputData {
  DummyInputData(const std::string& n) : GenericInputData{n} {}
};

struct DummyTask : public lsfm::Task {
  DummyTask(const std::string& name, bool verbose = false) : Task(name, verbose), prepared(false), run_count(0) {}

  void prepare(const lsfm::GenericInputData& data) override {
    prepared = true;
    input_name = data.name;
  }

  void run(std::size_t loops) override { run_count += loops; }

  void reset() override {
    prepared = false;
    run_count = 0;
    input_name.clear();
  }

  // Test state
  bool prepared;
  std::size_t run_count;
  std::string input_name;
};

}  // namespace

TEST(TaskTest, ConstructorSetsName) {
  DummyTask task("test_task", true);
  EXPECT_EQ(task.name, "test_task");
  EXPECT_TRUE(task.verbose);
}

TEST(TaskTest, PrepareAndRun) {
  DummyTask task("test_task");
  DummyInputData data("input1");

  // Initially not prepared
  EXPECT_FALSE(task.prepared);
  EXPECT_EQ(task.run_count, 0);

  // Prepare task
  task.prepare(data);
  EXPECT_TRUE(task.prepared);
  EXPECT_EQ(task.input_name, "input1");

  // Run task
  task.run(5);
  EXPECT_EQ(task.run_count, 5);

  // Run again to accumulate
  task.run(3);
  EXPECT_EQ(task.run_count, 8);
}

TEST(TaskTest, Reset) {
  DummyTask task("test_task");
  DummyInputData data("input1");

  task.prepare(data);
  task.run(10);

  EXPECT_TRUE(task.prepared);
  EXPECT_EQ(task.run_count, 10);
  EXPECT_EQ(task.input_name, "input1");

  // Reset should clear all state
  task.reset();
  EXPECT_FALSE(task.prepared);
  EXPECT_EQ(task.run_count, 0);
  EXPECT_TRUE(task.input_name.empty());
}

TEST(TaskTest, SharedPointer) {
  lsfm::Task::Ptr task = std::make_shared<DummyTask>("shared_task");
  EXPECT_EQ(task->name, "shared_task");

  lsfm::Task::PtrList tasks;
  tasks.push_back(task);
  tasks.push_back(std::make_shared<DummyTask>("task2"));

  EXPECT_EQ(tasks.size(), 2);
  EXPECT_EQ(tasks[0]->name, "shared_task");
  EXPECT_EQ(tasks[1]->name, "task2");
}
