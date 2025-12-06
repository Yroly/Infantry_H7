#pragma once

#include "usart.h"
namespace at{
constexpr size_t DBUS_BUFF_SIZE  = 18;
enum class DBusSwitchMode{
	DOWN,
	MID,
	UP
};
struct DBusMouseData
{
  float vx;  // 取值范围: [-1, 1]
  float vy;  // 取值范围: [-1, 1]
  float vs;  // 取值范围: [-1, 1], 鼠标滚轮, s代表scroll
  bool left;
  bool right;
	bool last_left;
	bool last_right;
};

struct DBusKeysData
{
    uint16_t W: 1;
    uint16_t S: 1;
    uint16_t A: 1;
    uint16_t D: 1;
    uint16_t Shift: 1;
    uint16_t Ctrl: 1;
    uint16_t Q: 1;
    uint16_t E: 1;
    uint16_t R: 1;
    uint16_t F: 1;
    uint16_t G: 1;
    uint16_t Z: 1;
    uint16_t X: 1;
    uint16_t C: 1;
    uint16_t V: 1;
    uint16_t B: 1;
};

class DBus
{
public:
  DBus(UART_HandleTypeDef * huart, bool use_dma = true);

  float ch_rx;  // 只读! 右水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_ry;  // 只读! 右垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lx;  // 只读! 左水平摇杆, 取值范围: [-1, 1], 右正左负
  float ch_ly;  // 只读! 左垂直摇杆, 取值范围: [-1, 1], 上正下负
  float ch_lu;  // 只读! 左上方拨轮, 取值范围: [-1, 1], 左正右负

  DBusSwitchMode sr;  // 只读! 右三位开关
  DBusSwitchMode sl;  // 只读! 左三位开关

  DBusMouseData mouse;  // 只读!
  DBusKeysData keys;    // 只读!

  void request();
  void update(uint16_t size, uint32_t stamp_ms);

  bool is_open() const;
  bool is_alive(uint32_t now_ms) const;

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;

  bool has_read_;
  uint32_t last_read_ms_;

  uint8_t buff_[DBUS_BUFF_SIZE];
};
}