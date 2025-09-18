#include <gtest/gtest.h>
#include <utility/value.hpp>

using namespace lsfm;

class ValueTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(ValueTest, DefaultConstructor) {
    Value v;
    EXPECT_EQ(v.type(), Value::FLOAT);
    EXPECT_EQ(v.get<double>(), 0.0);
}

TEST_F(ValueTest, FloatConstructors) {
    Value v1(3.14);
    EXPECT_EQ(v1.type(), Value::FLOAT);
    EXPECT_DOUBLE_EQ(v1.get<double>(), 3.14);
    
    Value v2(2.5f);
    EXPECT_EQ(v2.type(), Value::FLOAT);
    EXPECT_FLOAT_EQ(v2.get<float>(), 2.5f);
}

TEST_F(ValueTest, IntegerConstructors) {
    Value v1(42);
    EXPECT_EQ(v1.type(), Value::INT);
    EXPECT_EQ(v1.get<int>(), 42);
    
    Value v2(static_cast<int64_t>(123456789));
    EXPECT_EQ(v2.type(), Value::INT);
    EXPECT_EQ(v2.get<int64_t>(), 123456789);
    
    // Test that integers maintain their values
    Value v3(100);
    EXPECT_EQ(v3.type(), Value::INT);
    EXPECT_EQ(v3.get<int>(), 100);
}

TEST_F(ValueTest, BoolConstructor) {
    Value v1(true);
    EXPECT_EQ(v1.type(), Value::BOOL);
    EXPECT_TRUE(v1.get<bool>());
    
    Value v2(false);
    EXPECT_EQ(v2.type(), Value::BOOL);
    EXPECT_FALSE(v2.get<bool>());
}

TEST_F(ValueTest, StringConstructors) {
    Value v1("hello");
    EXPECT_EQ(v1.type(), Value::STRING);
    EXPECT_STREQ(v1.get<const char*>(), "hello");
    
    std::string str = "world";
    Value v2(str);
    EXPECT_EQ(v2.type(), Value::STRING);
    EXPECT_STREQ(v2.get<const char*>(), "world");
    
    Value v3(static_cast<const char*>(nullptr));
    EXPECT_EQ(v3.type(), Value::STRING);
    EXPECT_STREQ(v3.get<const char*>(), "");
}

TEST_F(ValueTest, CopyConstructor) {
    Value original(42.5);
    Value copy(original);
    
    EXPECT_EQ(copy.type(), Value::FLOAT);
    EXPECT_DOUBLE_EQ(copy.get<double>(), 42.5);
    
    // Test string copy
    Value str_original("test");
    Value str_copy(str_original);
    EXPECT_EQ(str_copy.type(), Value::STRING);
    EXPECT_STREQ(str_copy.get<const char*>(), "test");
}

TEST_F(ValueTest, AssignmentOperators) {
    Value v;
    
    v = 3.14;
    EXPECT_EQ(v.type(), Value::FLOAT);
    EXPECT_DOUBLE_EQ(v.get<double>(), 3.14);
    
    v = 2.5f;
    EXPECT_EQ(v.type(), Value::FLOAT);
    EXPECT_FLOAT_EQ(v.get<float>(), 2.5f);
    
    v = 42;
    EXPECT_EQ(v.type(), Value::INT);
    EXPECT_EQ(v.get<int>(), 42);
    
    v = static_cast<int64_t>(999);
    EXPECT_EQ(v.type(), Value::INT);
    EXPECT_EQ(v.get<int64_t>(), 999);
}

TEST_F(ValueTest, NAVStaticValue) {
    const Value& nav = Value::NAV();
    EXPECT_EQ(nav.type(), Value::NOT_A_VALUE);
    
    // Should return the same instance
    const Value& nav2 = Value::NAV();
    EXPECT_EQ(&nav, &nav2);
}

TEST_F(ValueTest, TypeChecking) {
    Value float_val(3.14);
    Value int_val(42);
    Value bool_val(true);
    Value string_val("test");
    
    // Check types directly
    EXPECT_EQ(float_val.type(), Value::FLOAT);
    EXPECT_EQ(int_val.type(), Value::INT);
    EXPECT_EQ(bool_val.type(), Value::BOOL);
    EXPECT_EQ(string_val.type(), Value::STRING);
    
    // Check type names
    EXPECT_EQ(float_val.typeName(), "float");
    EXPECT_EQ(int_val.typeName(), "int");
    EXPECT_EQ(bool_val.typeName(), "bool");
    EXPECT_EQ(string_val.typeName(), "string");
}

TEST_F(ValueTest, Conversions) {
    // Test that values maintain their type constraints
    Value int_val(42);
    EXPECT_EQ(int_val.type(), Value::INT);
    EXPECT_EQ(int_val.get<int>(), 42);
    
    Value float_val(3.14);
    EXPECT_EQ(float_val.type(), Value::FLOAT);
    EXPECT_DOUBLE_EQ(float_val.get<double>(), 3.14);
    
    Value bool_val(true);
    EXPECT_EQ(bool_val.type(), Value::BOOL);
    EXPECT_TRUE(bool_val.get<bool>());
}

TEST_F(ValueTest, EmptyStringHandling) {
    Value empty_str("");
    EXPECT_EQ(empty_str.type(), Value::STRING);
    EXPECT_STREQ(empty_str.get<const char*>(), "");
}