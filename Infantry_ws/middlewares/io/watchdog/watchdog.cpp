#include "WatchDog.h"

namespace at
{
WatchDog* WatchDog::watchdog_list_[WatchDog::MAX_WATCHDOG_NUM] = { nullptr };

WatchDog::WatchDog(uint8_t id, uint32_t life)
    : life_(0)
    , max_life_(life)
    , id_(id)
{
    if(id < MAX_WATCHDOG_NUM)
    {
        watchdog_list_[id] = this;
    }
}

void WatchDog::init(uint32_t life)
{
    max_life_ = life;
    life_ = 0;
}

void WatchDog::feed()
{
    life_ = 0;
    onFeed();
}

void WatchDog::onTimeout()
{
    // printf("WatchDog %d offline!\n", id_);
}

void WatchDog::onFeed()
{
    // printf("WatchDog %d fed.\n", id_);
}

void WatchDog::polling()
{
    for(size_t i = 0; i < MAX_WATCHDOG_NUM; ++i)
    {
        WatchDog* dog = watchdog_list_[i];
        if(dog == nullptr)
            continue;

        dog->life_++;

        if(dog->life_ >= dog->max_life_)
        {
            dog->onTimeout();
        }
    }
}

} // namespace at
