#include <eval/eval_app.hpp>
#include <utility/console_app.hpp>

#include <gtest/gtest.h>

namespace {
struct DummyConsole : public lsfm::ConsoleApp {
  using ConsoleApp::ConsoleApp;
  int run() override { return verbose_ ? 2 : 1; }
};

struct DummyEval : public lsfm::EvalApp {
  using EvalApp::EvalApp;
  void runEval() override {}
};
}  // namespace

TEST(ConsoleAppTest, RunsAndParses) {
  DummyConsole app("dummy");
  const char* argv1[] = {"dummy"};
  int rc = static_cast<lsfm::ConsoleAppInterface&>(app).run(1, const_cast<char**>(argv1));
  EXPECT_EQ(rc, 1);

  const char* argv2[] = {"dummy", "--verbose"};
  rc = static_cast<lsfm::ConsoleAppInterface&>(app).run(2, const_cast<char**>(argv2));
  EXPECT_EQ(rc, 2);
}

#if GTEST_HAS_DEATH_TEST
TEST(ConsoleAppTest, HelpAndVersionExit) {
  DummyConsole app("dummy", "desc", "9.9.9");
  const char* argv1[] = {"dummy", "--help"};
  EXPECT_EXIT(static_cast<lsfm::ConsoleAppInterface&>(app).run(2, const_cast<char**>(argv1)),
              ::testing::ExitedWithCode(0), "");

  const char* argv2[] = {"dummy", "--version"};
  EXPECT_EXIT(static_cast<lsfm::ConsoleAppInterface&>(app).run(2, const_cast<char**>(argv2)),
              ::testing::ExitedWithCode(0), "");
}
#endif

TEST(EvalAppTest, RequiresInputAndRuns) {
  DummyEval app("eval");
  // missing required input should exit with code 1 via ConsoleApp::parseArgs
#if GTEST_HAS_DEATH_TEST
  const char* argv_bad[] = {"eval"};
  EXPECT_EXIT(static_cast<lsfm::ConsoleAppInterface&>(app).run(1, const_cast<char**>(argv_bad)),
              ::testing::ExitedWithCode(1), "");
#endif
  // valid input provided
  const char* argv_ok[] = {"eval", "--input", "file"};
  int rc = static_cast<lsfm::ConsoleAppInterface&>(app).run(3, const_cast<char**>(argv_ok));
  EXPECT_EQ(rc, 0);
}
