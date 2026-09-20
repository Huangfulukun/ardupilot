# Joby S4 concept SITL model

This directory contains a **research/concept model**, not a Joby Aviation
flight-dynamics model and not certification data.

## What is represented

- six electric tilt-propeller propulsion stations;
- four propulsion stations on the main wing and two on the V-tail;
- a full-aircraft gross-mass target of 2400 kg;
- fixed-wing lift/drag plus propulsive VTOL forces;
- approximately 90 degrees of propulsion-axis travel between hover and cruise;
- ArduPlane Hexa/X motor mixing with vectored-yaw tiltrotor control.

The production aircraft can control propulsion-station tilt, rotor speed and
blade pitch independently.  Current ArduPlane tiltrotor support exposes grouped
left/right vectored tilt outputs, so this first SITL implementation groups the
six physical stations by side.  That limitation is deliberate and documented;
it provides a stable baseline for later independent-tilt control-allocation
work.

## Public data and modelling assumptions

Public Joby material (data published December 2025) lists six five-bladed
electric tilt-propeller units, a maximum gross weight of 2400 kg, a top speed
of 200 mph, four battery packs, and a transition range of 45-90 knots.

NASA documentation describes four propulsion stations on the wing and two at
the V-tail, with each station containing an electric propulsion unit, tilt
mechanism and five-bladed variable-pitch propeller.

Values that are not public production data -- including inertia, exact
propulsor coordinates, aerodynamic derivatives, battery voltage/current,
pack capacity and controller gains -- are engineering starting assumptions.
The 2.9 m propeller-diameter value used to form the 39.63 m^2 total disc area
comes from an earlier public S4 technical description and should be replaced
when a validated geometry is available.

Primary public references used when creating this model:

- https://www.jobyaviation.com/technology
- NASA AAM-NC-115-001, *UAM Instrument Flight Procedure Design and Evaluation
  in the Joby Flight Simulator*
- AOPA, *Joby S4: Coming to your airport in 2025?*

## Run

From the ArduPilot repository root:

    Tools/autotest/sim_vehicle.py -v ArduPlane -f jobys4 --console --map

For a clean first run:

    Tools/autotest/sim_vehicle.py -v ArduPlane -f jobys4 -w --console --map

Recommended initial validation sequence:

1. arm in QSTABILIZE/QHOVER and verify hover trim;
2. verify motor order and left/right tilt directions;
3. check TILT log values and transition actuator motion;
4. perform a low-altitude QHOVER -> FBWA transition;
5. retune the Q attitude/rate loops before using aggressive manoeuvres.

Do not treat this model as a source of real-aircraft limits or certification
evidence.
