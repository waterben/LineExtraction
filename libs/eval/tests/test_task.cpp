//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
/// @file test_task.cpp
/// @brief Unit tests for evaluation tasks.
#include <eval/input_task.hpp>
#include <eval/task.hpp>

#include <gtest/gtest.h>

namespace {

struct DummyInputData : public lsfm::GenericInputData {
  DummyInputData(const std::string& n) : GenericInputData{n} {}
};

/// @brief Dummy task for testing base Task class (no prepare)
struct DummyTask : public lsfm::Task {
  DummyTask(const std::string& task_name, bool task_verbose = false) : Task(task_name, task_verbose), run_count(0) {}

  void run(std::size_t loops) override { run_count += loops; }

  void reset() override { run_count = 0; }

  // Test state
  std::size_t run_count;
};

/// @brief Dummy input task for testing InputTask class (with prepare)
struct DummyInputTask : public lsfm::InputTask<DummyInputData> {
  DummyInputTask(const std::string& task_name, bool task_verbose = false)
      : InputTask(task_name, task_verbose), prepared(false), run_count(0), input_name() {}

  // Bring base class prepare overloads into scope
  using lsfm::InputTask<DummyInputData>::prepare;

  void prepare(const DummyInputData& data) override {
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

TEST(TaskTest, Run) {
  DummyTask task("test_task");

  // Initially no runs
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(0));

  // Run task
  task.run(5);
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(5));

  // Run again to accumulate
  task.run(3);
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(8));
}

TEST(InputTaskTest, PrepareAndRun) {
  DummyInputTask task("test_task");
  DummyInputData data("input1");

  // Initially not prepared
  EXPECT_FALSE(task.prepared);
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(0));

  // Prepare task
  task.prepare(data);
  EXPECT_TRUE(task.prepared);
  EXPECT_EQ(task.input_name, "input1");

  // Run task
  task.run(5);
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(5));

  // Run again to accumulate
  task.run(3);
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(8));
}

TEST(InputTaskTest, Reset) {
  DummyInputTask task("test_task");
  DummyInputData data("input1");

  task.prepare(data);
  task.run(10);

  EXPECT_TRUE(task.prepared);
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(10));
  EXPECT_EQ(task.input_name, "input1");

  // Reset should clear all state
  task.reset();
  EXPECT_FALSE(task.prepared);
  EXPECT_EQ(task.run_count, static_cast<std::size_t>(0));
  EXPECT_TRUE(task.input_name.empty());
}

TEST(TaskTest, SharedPointer) {
  lsfm::Task::Ptr task = std::make_shared<DummyTask>("shared_task");
  EXPECT_EQ(task->name, "shared_task");

  lsfm::Task::PtrList tasks;
  tasks.push_back(task);
  tasks.push_back(std::make_shared<DummyTask>("task2"));

  EXPECT_EQ(tasks.size(), static_cast<size_t>(2));
  EXPECT_EQ(tasks[0]->name, "shared_task");
  EXPECT_EQ(tasks[1]->name, "task2");
}
