#pragma once

#include <thread>

void delay(uint32_t milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}