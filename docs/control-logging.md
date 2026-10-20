# Control Data Logging (Plan)

## Concept

Record the autosteer control data to CSV so steering problems (snaking, sitting off the line,
overshoot when acquiring, wheel chatter, dropouts) can be analysed after a field session,
the same way RateController's PID logs are used for rate control.

Autosteer has two control loops, and each one only knows its own half:

| | Inner loop: wheel follows the angle | Outer loop: tractor follows the line |
|---|---|---|
| Runs in | Steer module firmware | AOG (`CABLine`, `CABCurve`, `CGuidance`, ...) |
| Rate | 40 Hz (Teensy `LOOP_TIME = 25`) | GPS fix rate, ~10 Hz |
| Knows | WAS, PWM, disable reasons | GPS, heading, cross-track error, guidance terms |

Phase 1 logs everything AOG knows. Phase 2 adds a firmware diagnostic packet for the inner loop.

Status: **PLAN ONLY**. Branch `control-logging`, based on tag `6.8.6`. Line numbers below refer to 6.8.6.

## Phase 1 - AOG guidance log

### Hook point
`UpdateFixPosition()` (`Forms/Position.designer.cs:131`) runs fix, heading, guidance and then sends the
autosteer packet with `SendPgnToLoop(p_254.pgn)`. One row is written right after that send, so the row
holds exactly what went to the module plus everything that produced it.

- Rate: one row per fix (~10 Hz, follows the receiver).
- Size: ~300 bytes per row, ~10 MB per hour.
- Location: `Documents\AgOpenGPS\Logs\` (`RegistrySettings.logsDirectory`), next to the existing events log.
- One file per session: `SteerLog_YYYYMMDD_HHMMSS.csv`.
- Buffered writer, flushed about once a second and on close.

### Columns

**1. Timing**

| Column | Source | Shows |
|---|---|---|
| PCTime | clock, ms | lines rows up with RateController logs and notes |
| FixDt_ms | time since previous fix | missed or late fixes |
| GpsHz | `gpsHz` | receiver rate drift |
| Sim | `timerSim.Enabled` | simulator rows |

**2. GPS** (PANDA / PGN 0xD6, `Forms/UDPComm.Designer.cs:58`)

| Column | Source |
|---|---|
| Lat, Lon | `AppModel.CurrentLatLon` |
| FixE, FixN, PivotE, PivotN (m, local plane) | `pn.fix`, `pivotAxlePos` |
| FixQuality, Sats, HDOP, Age | `pn.fixQuality`, `pn.satellitesTracked`, `pn.hdop`, `pn.age` |
| Altitude | `pn.altitude` |
| SpeedRaw, SpeedAvg | `pn.speed`, `avgSpeed` (used by guidance) |
| Reverse, ChangingDir | `isReverse`, `isChangingDirection` |

**3. Heading and roll**

| Column | Source |
|---|---|
| HeadingSource | `headingFromSource` (Fix / Dual / fused IMU) |
| GpsHeading, DualHeading, ImuHeading, ImuCorrected, FixHeading (used) | `Position.designer.cs:243-285`, `pn.headingTrueDual`, `ahrs.imuHeading` |
| Roll, YawRate | `ahrs.imuRoll`, `ahrs.imuYawRate` |

All heading candidates side by side show when IMU fusion or the heading source misbehaves.

**4. Guidance (outer loop)**

Each line type keeps its own values (`inty`, distance, steer angle) in its own object: `ABLine`,
`curve`, `ct` (contour), `recPath`, `yt` (U-turn), `gyd` (Stanley). The logger reads the active one.

| Column | Source |
|---|---|
| Mode | AB / Curve / Contour / RecPath / UTurn / FreeDrive / None |
| Algorithm | Stanley or Pure Pursuit (`isStanleyUsed`) |
| Track, PathsAway | `trk.idx` and name, `howManyPathsAway` |
| XTE_mm | `guidanceLineDistanceOff`; blank for sentinels 32000 (no line) and 32020 (steer off) |
| HeadingError_deg | `vehicle.modeActualHeadingError` (Pure Pursuit), `gyd.steerHeadingError` (Stanley, after gain) |
| LookAhead_m | `vehicle.goalDistance` (hold vs acquire) |
| Integral | `inty` of the active object |
| PPRadius / StanleyXteTerm | `ppRadiusAB/Cu/CT/RP/YT` / `gyd.xTrackSteerCorrection` |
| SideHill_deg | `ahrs.imuRoll * gyd.sideHillCompFactor` (roll compensation share of the command) |
| SteerCmd_deg | `guidanceLineSteerAngle / 100` |
| SteerSent_deg | steer angle bytes actually in PGN 254 |
| DeadZone, AtMaxAngle | `vehicle.isInDeadZone`; command clamped at `vehicle.maxSteerAngle` |

SteerCmd and SteerSent differ inside the dead zone, where the previous angle is held.

**5. Engagement and machine state**

| Column | Source |
|---|---|
| AutoSteerBtn, StatusSent | `isBtnAutoSteerOn`; status byte in PGN 254 |
| SteerSwitch, WorkSwitch | `mc.steerSwitchHigh`, `mc.workSwitchHigh` |
| UTurn, UTurnPhase | `yt.isYouTurnTriggered`, `yt.youTurnPhase` |
| ContourLocked | `ct.isLocked` |
| Sections | section bytes in PGN 254, hex |

**6. Module response** (PGN 253)

| Column | Source |
|---|---|
| ActualAngle_deg | `mc.actualSteerAngleDegrees` |
| AngleError_deg | sent setpoint minus actual |
| PWM | `mc.pwmDisplay` (unsigned - firmware sends `abs(pwmDrive)`) |
| SensorData | `mc.sensorData` (pressure / current kick-out) |
| ModuleAge | `steerModuleConnectedCounter` |

The module answers each PGN 254 with a PGN 253, so ActualAngle is about one fix old.

**7. Event** (text, usually blank)

State changes when they happen:
- autosteer engaged / disengaged, with reason (button, switch, min speed, max speed, reverse, no heading)
- fix quality change
- heading source switch
- line change, nudge, U-turn start
- steer settings sent

AOG's existing events log (`Log.EventWriter`) keeps only the last 100 lines.

### Settings sidecar
At start, write `SteerLog_..._settings.txt`. Append a new copy each time steer settings are sent
(and note it in the Event column):
- Vehicle: wheelbase, antenna pivot / height / offset, max steer angle, min / max steer speed
- Stanley: distance gain, heading gain, integral
- Pure Pursuit: hold lookahead, acquire factor, lookahead multiplier, integral
- Side-hill compensation, dead zone heading and delay
- Module (PGN 252 / 251): Kp, high / low / min PWM, counts per degree, Ackermann, WAS offset, invert flags
- IMU: fusion weight, roll zero, roll filter, dual heading offset

### On/off switch
Real Designer control, setting persisted. Location to be decided
(steer settings form, steer chart form, or Tools menu).

## Phase 2 - Steer module diagnostic packet

New PGN from the steer module firmware (`F:\Documents\GitHub\AOG_AutoSteer\AutoSteerTeensy`), sent every
25 ms loop while steering is enabled. AgIO already forwards every `0x80 0x81` module packet to AOG
(`AgIO/Source/Forms/UDP.designer.cs:350`). AOG writes it to `SteerModuleLog_YYYYMMDD_HHMMSS.csv`
with the same PCTime stamps.

Fields:
- module millis
- raw WAS counts, ZeroOffset, actual angle
- setpoint, ms since last PGN 254 (dropped packets)
- angle error, low/high PWM limit in effect, signed PWM
- disable-reason bits (AOG timeout, not ready, steer switch, min speed), relay state
- speed, IMU heading / roll / yaw rate
- max loop time

PGN number to be chosen when built. AOG already reads 0x84, 0xD3, 0xD4, 0xD6, 0xF0 and 221-253;
also check against module, AgIO and tool-steer PGNs (230, 233).

Why it is needed: PGN 253 is only sent in reply to PGN 254, so AOG sees the wheel at GPS rate,
not 40 Hz; PWM is sent unsigned; and the reason steering dropped out never reaches AOG.

## Open decisions
1. Record every fix while logging is on (including manual driving), or only while autosteer is engaged
   plus ~5 s either side. Recommendation: every fix.
2. Location of the on/off switch.
3. Phase 2 straight after phase 1, or field-test phase 1 first.
