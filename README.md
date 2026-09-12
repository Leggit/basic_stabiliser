This is an untested version, use at your own risk!

## Optional elevon mode

For a flying-wing airframe, set `ENABLE_ELEVONS` to `true` in
`src/config.h` and rebuild/upload the firmware. The two aileron servos then
receive the pitch and roll mix in both manual and stabilised modes. The
separate elevator servo is held at neutral (`1500` microseconds).

The four `ELEVON_*_SIGN` values in `src/config.h` allow the servo directions to
be corrected without changing the mixer. Bench-test with linkages disconnected
before flight and reverse a sign if a surface moves in the wrong direction.

When `ENABLE_ELEVONS` is `false`, the existing elevator, aileron, and flaperon
behavior is retained.

## Airframe layouts

Servo attachment and output mixing live behind the `Airframe` interface in
`src/airframe/airframe.h`. `StandardAirframe` implements the elevator,
ailerons, and flaperons arrangement; `ElevonAirframe` implements the flying
wing arrangement. To add a layout such as a V-tail, create another
`Airframe` implementation and select it in `setup()` without adding layout
logic to the flight loop.

## Status indicators

Startup, ready, and error signaling live behind the `StatusIndicator`
interface in `src/status_indicator/status_indicator.h`. `SingleLED` is the
current implementation. A buzzer, RGB LED, or display can be added as another
implementation without putting hardware-specific signaling code in `main()`.

## Control input mapping

Receiver values are translated into pilot commands by `ControlInputMapper` in
`src/control_input/control_input.h`. This keeps transmitter-specific details
such as center, deadband, scale, and direction out of the flight loop.
