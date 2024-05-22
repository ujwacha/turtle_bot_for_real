#include "encoder.hpp"
#include <cstdint>

void Encoder::init()
{
 HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);
 last_reset_time = HAL_GetTick();
}

float Encoder::get_encoder_omega()
{
 uint32_t sampling_time = HAL_GetTick() - last_reset_time;
 if(sampling_time >= sample_time)
 {
  omega = (2.0f * PI)*((float)get_count()/((float)cpr*float(sampling_time))) * 1000.0f;
  reset_encoder_count();
 }
 return omega;
}

int32_t Encoder::get_count()
{
  int32_t count = henc->Instance->CNT;
  if(count > int32_t(32768))
    {
      count = count - int32_t(65536);
    }
  return count;
}

// int64_t Encoder::get_count_aggregate()
// {
//  return count_aggregate + get_count();
// }

void Encoder::reset_encoder_count()
{
  // count_aggregate += get_count();
 henc->Instance->CNT = 0;
 last_reset_time = HAL_GetTick();
}
