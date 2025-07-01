/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nopla
 */

#include "app.hpp"

int App::run() {
  this->initialize();

  while (true) {
    this->loop();
  }

  return 0;
}

void App::initialize() {
  for (auto device : this->state->devices) {
    {
      device->initialize();
    }
  }
}

void App::loop() {}

void App::interval() {}
