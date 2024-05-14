#include <cstdint>
#include <stm32f4xx_hal_tim.h>
#define PI 3.14159265358979f

class encoder
{
 private:
  TIM_HandleTypeDef *henc; // pointer to encoder timer
  int16_t cpr = 0; // count per revolution
  uint16_t sample_time; // time to sample for omega
  float omega = 0.0f;
  int64_t count_aggregate = 0;
  uint32_t last_reset_time = 0;

 public:
  encoder(TIM_HandleTypeDef *_henc, int16_t _cpr, uint16_t _sample_time = 1)
  {
   henc = _henc;
   cpr = _cpr;
   sample_time = _sample_time;
   init();
  }

  //  ~encoder() = default;
  void init();
  float get_omega();
  int32_t get_count();
  int64_t get_count_aggregate();
  void reset_encoder_count();
};
