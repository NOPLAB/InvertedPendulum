/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nopla
 */

#include "app.hpp"
#include "mux.hpp"

int App::run() {
  this->initialize();

  while (true) {
    this->loop();
  }

  return 0;
}

void App::initialize() {
  this->interruptHandler->registerAdc(this->state->adcInterruptHandlers, 1);
  this->interruptHandler->registerTimer(nullptr, 0);

  for (auto device : this->state->devices) {
    {
      device->initialize();
    }
  }
}

void App::loop() {}

void App::interval() {
  MuxCorrectedValues values;
  this->state->mux->getCorrectedValues(&values);
}
