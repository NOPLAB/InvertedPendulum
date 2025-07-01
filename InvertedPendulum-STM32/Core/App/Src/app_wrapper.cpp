#include "app_wrapper.hpp"
#include "app.hpp"

static App *appInstance = nullptr;

void AppWrapperInit(void) {
  if (appInstance == nullptr) {
    appInstance = new App();
  }
}

void AppWrapperRun(void) {
  if (appInstance != nullptr) {
    appInstance->run();
  }
}