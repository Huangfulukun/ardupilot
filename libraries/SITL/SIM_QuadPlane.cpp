/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple quadplane simulator class
*/

#include "SIM_QuadPlane.h"

#include <stdio.h>

using namespace SITL;

QuadPlane::QuadPlane(const char *frame_str) :
    Plane(frame_str)
{
    // default to X frame
    const char *frame_type = "x";
    uint8_t motor_offset = 4;

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    if (strstr(frame_str, "tilthexa30")) {
        // Paper-3 SITL-only 30 kg independently tilting HexaX model.
        // Reuse the standard HexaX propulsion order while exposing six
        // additional pitch-tilt servo channels below.
        frame_type = "hexax";
        thrust_scale = 0;
    } else if (strstr(frame_str, "jobys4")) {
        // Six tilting propulsion stations.  The model JSON provides the
        // full-scale mass, inertia, wing aerodynamics and motor positions.
        frame_type = "jobys4";
        // Joby uses the tilt-propellers for both VTOL and cruise; there is
        // no separate conventional forward-throttle motor in this model.
        thrust_scale = 0;
    } else if (strstr(frame_str, "-octa-quad-cor")) {
        frame_type = "octa-quad-cor";
    } else if (strstr(frame_str, "-octa-quad-cw-cor")) {
        frame_type = "octa-quad-cw-cor";
    } else if (strstr(frame_str, "-octa-quad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octaquad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octa")) {
        frame_type = "octa";
    } else if (strstr(frame_str, "-hexax")) {
        frame_type = "hexax";
    } else if (strstr(frame_str, "-hexa")) {
        frame_type = "hexa";
    } else if (strstr(frame_str, "-plus")) {
        frame_type = "+";
    } else if (strstr(frame_str, "-y6")) {
        frame_type = "y6";
    } else if (strstr(frame_str, "-tri")) {
        frame_type = "tri";
    } else if (strstr(frame_str, "-tilttrivec")) {
        frame_type = "tilttrivec";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-tilthvec")) {
        frame_type = "tilthvec";
    } else if (strstr(frame_str, "-tilttri")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "firefly")) {
        frame_type = "firefly";
        // elevon style surfaces
        elevons = true;
        // fwd motor gives zero thrust
        thrust_scale = 0;
        // vtol motors start at 2
        motor_offset = 2;
    } else if (strstr(frame_str, "-tilt")) {
        frame_type = "tilt";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "cl84")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-copter_tailsitter")) {
        frame_type = "+";
        copter_tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    frame = Frame::create_frame(frame_type);
    if (frame == nullptr) {
        printf("Failed to find frame '%s' or insufficient memory\n", frame_type);
        exit(1);
    }

    if (strstr(frame_str, "cl84")) {
        // setup retract servos at front
        frame->motors[0].servo_type = Motor::SERVO_RETRACT;
        frame->motors[0].servo_rate = 7*60.0/90; // 7 seconds to change
        frame->motors[1].servo_type = Motor::SERVO_RETRACT;
        frame->motors[1].servo_rate = 7*60.0/90; // 7 seconds to change
    }

    if (strstr(frame_str, "tilthexa30")) {
        // SITL-only independent nacelle tilts on SERVO11..SERVO16.
        // Motor::pitch is opposite the paper beta convention, hence
        // beta=-10..+90 deg maps to motor pitch=+10..-90 deg.
        for (uint8_t i = 0; i < frame->num_motors; i++) {
            frame->motors[i].pitch_servo = 6 + i;
            frame->motors[i].pitch_min = 10.0f;
            frame->motors[i].pitch_max = -90.0f;
            // Motor::servo_rate is seconds per 60 degrees.
            frame->motors[i].servo_rate = 1.0f;
        }
    }

    // leave first 4 servos free for plane
    frame->motor_offset = motor_offset;

    // SITL's POSIX filesystem deliberately maps absolute-looking paths under
    // the process working directory.  The paper-3 harness writes its model
    // JSON into that working directory, so pass only the basename to Frame.
    const char *frame_init_arg = frame_str;
    char tilthexa30_frame_str[64];
    if (strstr(frame_str, "tilthexa30")) {
        const char *model_name = strrchr(frame_str, '/');
        if (model_name != nullptr) {
            snprintf(tilthexa30_frame_str, sizeof(tilthexa30_frame_str),
                     "quadplane-tilthexa30:%s", model_name + 1);
            frame_init_arg = tilthexa30_frame_str;
        }
    }

    // we use zero terminal velocity to let the plane model handle the drag
    frame->init(frame_init_arg);
    battery.setup(frame->get_model_batt_capacity_ah(),
                  frame->get_model_batt_resistance_ohm(),
                  frame->get_model_batt_max_voltage());

    // Most legacy quadplane frame JSON files describe only the multicopter
    // portion and add 50% for the fixed-wing structure.  JobyS4.json stores
    // the complete aircraft gross mass, so do not apply that legacy factor.
    if (strstr(frame_str, "jobys4") || strstr(frame_str, "tilthexa30")) {
        // These JSON files store complete aircraft mass, not just the VTOL
        // propulsion-frame portion used by legacy QuadPlane models.
        mass = frame->get_mass();
    } else {
        mass = frame->get_mass() * 1.5f;
    }
    frame->set_mass(mass);

    lock_step_scheduled = true;
}

/*
  update the quadplane simulation by one time step
 */
void QuadPlane::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    // first plane forces
    Vector3f rot_accel;
    calculate_forces(input, rot_accel);

    // now quad forces
    Vector3f quad_rot_accel;
    Vector3f quad_accel_body;

    motor_mask |= ((1U<<frame->num_motors)-1U) << frame->motor_offset;
    frame->calculate_forces(*this, input, quad_rot_accel, quad_accel_body, rpm, false);

    // rotate frames for copter tailsitters
    if (copter_tailsitter) {
        quad_rot_accel.rotate(ROTATION_PITCH_270);
        quad_accel_body.rotate(ROTATION_PITCH_270);
    }

    battery.maybe_reset(sitl->batt_voltage, sitl->batt_capacity_ah);
    battery_voltage = battery.get_voltage();
    battery_current = frame->get_current_amp();

    const uint64_t now_us = AP_HAL::micros64();
    battery.consume_energy(battery_current, now_us);

    float throttle;
    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    // assume 20A at full fwd throttle
    throttle = fabsf(throttle);
    battery_current += 20 * throttle;
    
    rot_accel += quad_rot_accel;
    accel_body += quad_accel_body;

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
