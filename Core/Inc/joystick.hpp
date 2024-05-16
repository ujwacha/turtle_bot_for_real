#ifdef _JOY_MSG_HPP
#define _JOY_MSG_HPP

#define BUTTONS(button) (1 << button)
namespace JoyMsg
{
    namespace PS4Msg
    {
        enum PS4Buttons
        {
             Cross,
             Circle,
             Square,
             Triangle,
             Share,
             Power,
             Option,
             L3,
             R3,
             L1, 
             R1,
             Up,
             Down,
             Left,
             Right,
             Touch
        };
    }  
}     
 struct JoyData
{
  int8_t lx;
  int8_t ly;
  int8_t rx;
  int8_t ry;
  uint8_t lt;
  uint8_t rt;
  uint16_t buttons;
};
