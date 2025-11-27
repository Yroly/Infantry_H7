#include "dbus.h"

#include "math.h"

namespace at
{
static float get_stick(uint16_t raw) { return (raw - 1024) / 660.0f; }

static DBusSwitchMode get_switch(uint8_t raw)
{
  if (raw == 1)
    return DBusSwitchMode::UP;
  else if (raw == 3)
    return DBusSwitchMode::MID;
  else
    return DBusSwitchMode::DOWN;
}

DBus::DBus(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), has_read_(false)
{
}

void DBus::request()
{
  if (use_dma_) {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_DMA(huart_, buff_, DBUS_BUFF_SIZE);

    // ref: https://github.com/HNUYueLuRM/basic_framework/blob/master/bsp/usart/bsp_usart.c
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
  }
  else {
    // dismiss return
    HAL_UARTEx_ReceiveToIdle_IT(huart_, buff_, DBUS_BUFF_SIZE);
  }
}

void DBus::update(uint16_t size, uint32_t stamp_ms)
{
  if (size != DBUS_BUFF_SIZE) return;

  has_read_ = true;
  last_read_ms_ = stamp_ms;

  // 遥控器解析
  float ch_rx = get_stick(( buff_[0] | (buff_[1] << 8)) & 0x07ff);
  float ch_ry = get_stick(((buff_[1] >> 3) | (buff_[2] << 5)) & 0x07ff);
  float ch_lx = get_stick(((buff_[2] >> 6) | (buff_[3] << 2) | (buff_[4] << 10)) & 0x07ff);
  float ch_ly = get_stick(((buff_[4] >> 1) | (buff_[5] << 7)) & 0x07ff);
  float ch_lu = get_stick(((buff_[16] | (buff_[17] << 8)) & 0x07ff));

  DBusSwitchMode sr = get_switch(( buff_[5] >> 4) & 0x0003);
  DBusSwitchMode sl = get_switch(((buff_[5] >> 4) & 0x000C) >> 2);

  // 遥控器数据异常
  if (fabs(ch_rx) > 1 || fabs(ch_ry) > 1 || fabs(ch_lx) > 1 || fabs(ch_ly) > 1)
    return;

  // 键鼠解析
  int16_t mouse_vx_int = (buff_[7] << 8) | buff_[6];
  int16_t mouse_vy_int = (buff_[9] << 8) | buff_[8];
  int16_t mouse_vs_int = (buff_[11] << 8) | buff_[10];
  float mouse_vx = mouse_vx_int / 32768.0f;
  float mouse_vy = mouse_vy_int / 32768.0f;
  float mouse_vs = mouse_vs_int / 32768.0f;

  uint16_t keyboard_value = (buff_[15] << 8) | buff_[14];

  // 鼠标数据异常
  if (fabs(mouse_vx) > 1 || fabs(mouse_vy) > 1 || fabs(mouse_vs) > 1) return;

  /// 更新公有属性

  // 遥控器
  this->ch_rx = ch_rx;
  this->ch_ry = ch_ry;
  this->ch_lx = ch_lx;
  this->ch_ly = ch_ly;
  this->ch_lu = ch_lu;

  this->sr = sr;
  this->sl = sl;

  // 键鼠
  this->mouse.vx = mouse_vx;
  this->mouse.vy = mouse_vy;
  this->mouse.vs = mouse_vs;

  this->mouse.left = (buff_[12] == 1);
  this->mouse.right = (buff_[13] == 1);

  this->keys.W = ((keyboard_value & 0x0001) != 0);
  this->keys.S = ((keyboard_value & 0x0002) != 0);
  this->keys.A = ((keyboard_value & 0x0004) != 0);
  this->keys.D = ((keyboard_value & 0x0008) != 0);
  this->keys.Shift = ((keyboard_value & 0x0010) != 0);
  this->keys.Ctrl = ((keyboard_value & 0x0020) != 0);
  this->keys.Q = ((keyboard_value & 0x0040) != 0);
  this->keys.E = ((keyboard_value & 0x0080) != 0);
  this->keys.R = ((keyboard_value & 0x0100) != 0);
  this->keys.F = ((keyboard_value & 0x0200) != 0);
  this->keys.G = ((keyboard_value & 0x0400) != 0);
  this->keys.Z = ((keyboard_value & 0x0800) != 0);
  this->keys.X = ((keyboard_value & 0x1000) != 0);
  this->keys.C = ((keyboard_value & 0x2000) != 0);
  this->keys.V = ((keyboard_value & 0x4000) != 0);
  this->keys.B = ((keyboard_value & 0x8000) != 0);
}

bool DBus::is_open() const { return has_read_; }

bool DBus::is_alive(uint32_t now_ms) const { return is_open() && (now_ms - last_read_ms_ < 100); }

}  // namespace sp