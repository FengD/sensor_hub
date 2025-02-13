#include "common/factory.h"
#include <gtest/gtest.h>

namespace sensor {
namespace hub {

class ComponentA {
 public:
  std::string GetName() const { return "ComponentA"; }
};

class ComponentB {
 public:
  std::string GetName() const { return "ComponentB"; }
};

REGISTER_COMPONENT(ComponentA);
REGISTER_COMPONENT(ComponentB);
REGISTER_CLASS(ComponentA, ComponentA);
REGISTER_CLASS(ComponentB, ComponentB);

TEST(ComponentFactoryTest, TestComponentCreation) {
  auto component_a = ComponentAFactory::get("ComponentA");
  auto component_b = ComponentBFactory::get("ComponentB");

  ASSERT_NE(component_a, nullptr);
  ASSERT_NE(component_b, nullptr);
  ASSERT_EQ(component_a->GetName(), "ComponentA");
  ASSERT_EQ(component_b->GetName(), "ComponentB");
}

}  // namespace hub
}  // namespace sensor

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
