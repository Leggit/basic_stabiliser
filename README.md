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
