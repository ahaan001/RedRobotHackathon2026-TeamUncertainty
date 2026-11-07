/**
 * @brief Controller mapping only. No logic lives here; everything calls Drive::
 *        or Mech::. These callbacks run on the BLE task and must not block.
 *
 * Redpad (RR_Controller)                      Bluefruit Connect (fallback)
 *   Left joystick   arcade drive                D-pad      drive (stock behaviour)
 *   Right joystick Y arm jog                    1          exit auton (stock)
 *   D-pad UP/DOWN   arm preset up/down           2          jaw: open / next closed
 *   D-pad LEFT      jaw open                     3          arm preset up
 *   D-pad RIGHT     jaw next closed (RIM, CLAMP) 4          arm preset down
 *   D-pad CENTER    precision drive toggle
 *   A               start autonomous (only before it has run, while idle)
 *   B               place macro (lower, open, lift)
 *   X               exit autonomous (stock behaviour, keep)
 *   Y               stow arm and stop drive
 *
 * Every handler except A and X ignores input unless the robot is in remote
 * control, because the stock library delivers these callbacks during auton too.
 */
#include "RedRobot.h"
#include "Config.h"
#include "Robot.h"

namespace RR_Controller
{
  void Joystick1(int8_t x, int8_t y)
  {
    if (!RedRobot::IsRemoteControlled()) return;
    Mech::noteCommand();
    Drive::arcade(x, y);
  }

  void Joystick2(int8_t x, int8_t y)
  {
    if (!RedRobot::IsRemoteControlled()) return;
    Mech::noteCommand();
    Mech::setArmJog(y);   // 0 when released stops the jog
  }

  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled()) return;
    Mech::noteCommand();
    if (!buttonPressedDown) return;        // act on press only

    switch (Button) {
      case UP:     Mech::armPresetUp();     break;
      case DOWN:   Mech::armPresetDown();   break;
      case LEFT:   Mech::jawOpen();         break;
      case RIGHT:  Mech::jawNextClosed();   break;
      case CENTER: Drive::togglePrecision(); break;
    }
  }

  void ButtonX_Pressed(bool buttonPressedDown)
  {
    // Exit autonomous (stock behaviour). Safe mid-move: this runs on the BLE task.
    if (!RedRobot::IsRemoteControlled() && buttonPressedDown)
    {
      Auton::abort();
      RedRobot::StartRemoteControl();
      RedRobot::WriteDCMotor(2, 0);
      RedRobot::WriteDCMotor(3, 0);
    }
  }

  void ButtonY_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled()) return;
    Mech::noteCommand();
    if (buttonPressedDown) Mech::stow();
  }

  void ButtonB_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled()) return;
    Mech::noteCommand();
    if (buttonPressedDown) Mech::startPlaceMacro();
  }

  void ButtonA_Pressed(bool buttonPressedDown)
  {
    if (!buttonPressedDown) return;
    // Start autonomous: only from idle teleop, only once per boot.
    if (RedRobot::IsRemoteControlled() && !Auton::hasRun()) {
      Drive::stop();
      Auton::arm();
      RedRobot::InitAuton();     // in_remote_control = false; loop() starts calling AutonomousLoop
    }
  }
}

namespace Bluefruit
{
  void DPad_Pressed(button_t Button, bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled()) return;
    Mech::noteCommand();

    static bool button_pressed[4] = {false, false, false, false};
    button_pressed[Button] = buttonPressedDown;

    int x = 0, y = 0;
    if (button_pressed[UP])    y += 100;
    if (button_pressed[DOWN])  y -= 100;
    if (button_pressed[LEFT])  x -= 50;
    if (button_pressed[RIGHT]) x += 50;
    Drive::arcade(x, y);
  }

  void Button1_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() && buttonPressedDown)
    {
      Auton::abort();
      RedRobot::StartRemoteControl();
      RedRobot::WriteDCMotor(2, 0);
      RedRobot::WriteDCMotor(3, 0);
    }
  }

  void Button2_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() || !buttonPressedDown) return;
    Mech::noteCommand();
    if (Mech::jawPreset() == JAW_OPEN) Mech::jawNextClosed(); else Mech::jawOpen();
  }

  void Button3_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() || !buttonPressedDown) return;
    Mech::noteCommand();
    Mech::armPresetUp();
  }

  void Button4_Pressed(bool buttonPressedDown)
  {
    if (!RedRobot::IsRemoteControlled() || !buttonPressedDown) return;
    Mech::noteCommand();
    Mech::armPresetDown();
  }
}
