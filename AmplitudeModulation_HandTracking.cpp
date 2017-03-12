// This example uses the Amplitude Modulation Emitter and a Leap Motion to
// project a basic point onto a moving palm

#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>

#include <Leap.h>
#include <UltrahapticsAmplitudeModulation.hpp>

int main(int argc, char *argv[])
{
  // Create an emitter, alignment and leap controller.
  Ultrahaptics::AmplitudeModulation::Emitter emitter;
  Ultrahaptics::Alignment                    alignment;
  Leap::Controller                           controller;

  // Set frequency to 200 Hertz and maximum intensity
  const float frequency = 200.f * Ultrahaptics::Units::hertz;
  const float intensity = 1.5f;
  bool left_pressed = false, right_pressed = false;
  int left_count = 0, right_count = 0;
  const int pressed_released_count = 40;

  while (true) {
    // Get all the hand positions from the leap and position a focal point on each.
    const Leap::Frame    frame = controller.frame();
    const Leap::HandList hands = frame.hands();

    if (hands.isEmpty()) {
      emitter.stop();
    } else {
      for (int i = 0; i < hands.count(); i++) {
        if (hands[i].palmVelocity().z < -90 || (left_pressed && hands[i].isLeft()) || (right_pressed && hands[i].isRight())) {
          const Leap::Hand & hand = hands[i];

          if (!left_pressed && hand.isLeft()) {
            left_pressed = true;
            left_count = 0;
          }
          else if (left_pressed && hand.isLeft())
            left_count++;
          else if (!right_pressed && hand.isRight()) {
            right_pressed = true;
            right_count = 0;
          }
          else if (right_pressed && hand.isRight())
            right_count++;

          // Translate the hand position from leap objects to Ultrahaptics objects.
          const Leap::Vector & leap_palm_position       = hand.palmPosition();
          const Leap::Vector & leap_palm_normal         = hand.palmNormal();
          const Leap::Vector & leap_palm_direction      = hand.direction();

          // Convert to Ultrahaptics vectors, normal is negated as leap normal points down.
          const Ultrahaptics::Vector3 uh_palm_position  = +Ultrahaptics::Vector3(leap_palm_position.x,  leap_palm_position.y,  leap_palm_position.z);
          const Ultrahaptics::Vector3 uh_palm_normal    = -Ultrahaptics::Vector3(leap_palm_normal.x,    leap_palm_normal.y,    leap_palm_normal.z);
          const Ultrahaptics::Vector3 uh_palm_direction = +Ultrahaptics::Vector3(leap_palm_direction.x, leap_palm_direction.y, leap_palm_direction.z);

          // Convert to device space from leap space.
          const Ultrahaptics::Vector3 device_palm_position  = alignment.fromTrackingPositionToDevicePosition(uh_palm_position);
          const Ultrahaptics::Vector3 device_palm_normal    = alignment.fromTrackingDirectionToDeviceDirection(uh_palm_normal).normalize();
          const Ultrahaptics::Vector3 device_palm_direction = alignment.fromTrackingDirectionToDeviceDirection(uh_palm_direction).normalize();

          // These can then be converted to be a unit axis on the palm of the hand.
          const Ultrahaptics::Vector3 device_palm_z = device_palm_normal;                             // Unit Z direction.
          const Ultrahaptics::Vector3 device_palm_y = device_palm_direction;                          // Unit Y direction.
          const Ultrahaptics::Vector3 device_palm_x = device_palm_y.cross(device_palm_z).normalize(); // Unit X direction.

          // Use these to create a point at 2cm x 2cm from the centre of the palm
          const Ultrahaptics::Vector3 position = device_palm_position + (2 * Ultrahaptics::Units::cm * device_palm_x) + (2 * Ultrahaptics::Units::cm * device_palm_y);

          // Emit the point
          const Ultrahaptics::ControlPoint point(position, intensity, frequency * Ultrahaptics::Units::hertz);


          emitter.update(point);
        }
        if (left_pressed && left_count == pressed_released_count)
          left_pressed = false;
        else if (right_pressed && right_count == pressed_released_count)
          right_pressed = false;
        if (!left_pressed && !right_pressed)
          emitter.stop();
      }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(10000));
  }

  return 0;
}
