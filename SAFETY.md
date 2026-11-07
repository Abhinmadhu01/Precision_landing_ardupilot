# Safety Guide

This guide applies to any drone project that uses ArduPilot with a companion computer. Copy it into each repository and add project-specific notes in the last section.

Autonomous code can fail in ways you did not test. Treat every flight as if the software will misbehave, and keep a person ready to take manual control.

## Golden rules

1. **Propellers are off** for every bench test, firmware flash, parameter change and first run of new code.
2. **Test in simulation first**, then on hardware with propellers off, then tethered or netted, then in open flight.
3. **A pilot with an RC transmitter is always ready** to take over. The transmitter must have a mode switch you can reach without looking.
4. **Know your kill switch** and test it on the ground before the first flight of the day.
5. **Fly only where it is legal** and safe, away from people, roads and buildings, and follow your local drone regulations.
6. **Never fly alone**: have a second person watching the sky and the area.

## Switching between Loiter and Guided (throttle rule)

In Loiter, the throttle stick controls climb rate: centre holds altitude, and below centre the drone descends. In Guided, the stick is ignored, so it is easy to leave it low without noticing.

- After switching **from Loiter to Guided**, return the throttle stick to the **centre** position.
- If the stick is left low and you later switch **from Guided to Loiter**, the drone will descend and can fall.
- Before every switch back to Loiter, check the throttle stick is centred first.

## Before arming

- [ ] Propellers are in good condition, fitted on the correct motors, and tight (skip only for bench tests, where they must be **off**)
- [ ] Frame, arms, landing gear and the companion computer mount are secure
- [ ] Battery is fully charged, undamaged, strapped down, and its voltage matches the failsafe settings
- [ ] Companion computer has a stable power supply (no under-voltage warnings)
- [ ] GPS has a 3D fix with a good satellite count, and home position is set
- [ ] Compass and accelerometer are calibrated; no pre-arm errors in the ground station
- [ ] RC link works; mode switch positions are correct (Loiter, Guided, RTL or Land)
- [ ] Failsafes are enabled (RC loss, battery, GCS loss) and set to Return or Land
- [ ] Throttle stick is at the minimum for arming and you know where the kill switch is
- [ ] Flight area is clear, a safe landing area is identified, and bystanders are behind you

## Batteries and power

- Use a LiPo-safe charger and a fireproof charging bag. Never charge unattended.
- Do not plug in a battery until the wiring has been checked twice.
- Power the companion computer from a supply that meets its rating. Under-voltage causes throttling, USB dropouts and reboots in flight.

## Failsafes and emergencies

| Situation | Action |
| --- | --- |
| Drone behaves unexpectedly in Guided | Switch to Loiter (centre the throttle first), or to Stabilize or RTL if Loiter is unavailable |
| Loss of position or GPS | Switch to Stabilize or Alt Hold and land manually |
| Software crash or companion computer freeze | Take manual control with the mode switch |
| Imminent crash or danger to people | Use the kill switch |
| Fire or battery swelling | Keep away, use a fire extinguisher or sand, never handle a hot or swollen LiPo |

## Bench testing

- Remove all propellers.
- Power from a current-limited bench supply or a battery you are watching.
- Check motor directions only with propellers off.
- Confirm the companion computer can arm the vehicle **only when you intend it to**.

## After flight

- Disarm, unplug the battery, and let it cool before charging.
- Inspect propellers, motors and connections for damage.
- Save logs and note anything unusual.

## Project-specific notes

Add notes for each repository here. Example for Vision-Based Precision Landing for ArduPilot:

- With the stack running, **switching to GUIDED arms the drone and takes off automatically.** Only switch to Guided when the area is clear and you are ready.
- The stack uses RTL as its failsafe. Make sure the RTL altitude and home position are suitable for your site.
- The landing phase hands control to ArduPilot LAND mode close to the ground. Stay clear of the landing area.

## Disclaimer

You are responsible for your own flights. This software is provided without warranty, and the authors are not liable for damage or injury.
