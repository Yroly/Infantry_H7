#pragma once

#include "stdint.h"
#include "FreeRTOS.h"

namespace at
{
class WatchDog
{
public:

  WatchDog(uint8_t id = 0, uint32_t life = 0);

  void init(uint32_t life);

  static void polling();

  void feed();

  bool is_dead() const { return life_ >= max_life_; }

  uint8_t id() const { return id_; }
	static constexpr size_t MAX_WATCHDOG_NUM = 9;
private:
  void onTimeout();
  void onFeed();
  uint32_t life_;
  uint32_t max_life_;
  uint8_t id_;
  static WatchDog* watchdog_list_[MAX_WATCHDOG_NUM];
};

} // namespace at
