# Changelog

## v0.1 | Lane Following

### v0.1.0 - Unknown
- [x] Plans hard coded path (no signs)
- [x] Constant speed around the path

### v0.1.1 - Feb 18, 2026
"Open navigation"
- [x] Remove unnecessary "turn angle" and "vehicle speed" subscriptions
- [x] Subscribe to /mapping/lanes instead of hard coding
- [x] Automatically navigate curves while staying on target
- [x] Removed support for "location" waypoint navigation

## v0.2 | Stop Sign Navigation
### v0.2.0 - Feb 20, 2026
- [x] Stop at stop signs and continue successfully
- [x] Refactor into state machine and events
- [x] Add config loading for state transitions

### v0.2.1 - WIP
- [ ] Re-add target_speed control
- [x] Move `math_utils` unti ~~`ap1::planning::math`~~ `ap1::math` namespace.
- [x] Add age check on incoming packets
- [x] Move intial state publish to init instead of updating every planning frame

## v0.3 | Controlled Navigation
### v0.3.0 - WIP
- [ ] Add emergency stop
- [ ] Support intersections and navigate through them
- [ ] Navigate to a determined position safely
- [ ] Cleanly handle end-of-path situations (see loop path @ end)
