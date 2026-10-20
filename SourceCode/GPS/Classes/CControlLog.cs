using AgLibrary.Logging;
using System;
using System.Globalization;
using System.IO;
using System.Text;

namespace AgOpenGPS
{
    /// <summary>
    /// Control data log - writes autosteer control data to CSV for troubleshooting.
    /// SteerLog: one row per GPS fix (guidance, heading, module response).
    /// SteerModuleLog: steer module diagnostic rows (PGN 249, ~40 Hz) while logging is on.
    /// Settings sidecar: steer settings at start and whenever they change.
    /// See docs/control-logging.md
    /// </summary>
    public class CControlLog
    {
        private readonly FormGPS mf;

        // PGN 249 - steer module diagnostics. AOG sends a request each second while logging,
        // the module streams diagnostic rows for 3 seconds after the last request.
        public const byte DiagPGN = 249;
        public const int DiagPacketLength = 35;
        private readonly byte[] diagRequest = new byte[] { 0x80, 0x81, 0x7F, DiagPGN, 1, 1, 0xCC };

        private StreamWriter fixWriter;
        private StreamWriter moduleWriter;
        private string fileStamp;
        private string settingsPath;
        private string lastSettings = "";

        private DateTime lastFixTime = DateTime.MinValue;
        private DateTime last253Time = DateTime.MinValue;
        private readonly StringBuilder pendingEvents = new StringBuilder();
        private readonly CultureInfo inv = CultureInfo.InvariantCulture;

        // previous row state, for event detection
        private bool firstRow = true;
        private bool prevBtnOn, prevSteerSwitch, prevWorkSwitch, prevReverse, prevUTurn;
        private byte prevStatus;
        private int prevFixQuality, prevTrackIdx;
        private string prevHeadingSource = "", prevMode = "";
        private double prevNudge;

        public bool IsOn { get; private set; }

        private static readonly string FixHeader =
            "PCTime,FixDt_ms,GpsHz,Sim," +
            "Lat,Lon,FixE,FixN,PivotE,PivotN,FixQuality,Sats,HDOP,Age,Altitude,SpeedRaw,SpeedAvg,Reverse,ChangingDir," +
            "HeadingSource,GpsHeading,DualHeading,ImuHeading,ImuCorrected,FixHeading,Roll,YawRate," +
            "Mode,Algorithm,TrackIdx,Track,PathsAway,Nudge_m,XTE_mm,HeadingError_deg,LookAhead_m,Integral,PPRadius,StanleyXteTerm_deg," +
            "SideHill_deg,SteerCmd_deg,SteerSent_deg,DeadZone,AtMaxAngle," +
            "AutoSteerBtn,StatusSent,SteerSwitch,WorkSwitch,UTurn,UTurnPhase,ContourLocked,Sections," +
            "ActualAngle_deg,AngleError_deg,PWM,SensorData,ModuleAge_ms," +
            "Event";

        private static readonly string ModuleHeader =
            "PCTime,ModuleMillis,WAS,ZeroOffset,ActualAngle_deg,SetPoint_deg,AngleError_deg,CmdPWM,OutPWM,MaxPWM," +
            "Enabled,AOGTimeout,NotReady,SteerSwitchOff,BelowMinSpeed,IMUConnected,SwitchByte," +
            "SetPointAge_ms,Speed_kmh,ImuHeading,ImuRoll,MaxLoop_us";

        public CControlLog(FormGPS callingForm)
        {
            mf = callingForm;
        }

        public void SetEnabled(bool on)
        {
            if (on && !IsOn) Start();
            else if (!on && IsOn) Stop();
        }

        private void Start()
        {
            try
            {
                Directory.CreateDirectory(RegistrySettings.logsDirectory);
                fileStamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", inv);

                fixWriter = new StreamWriter(Path.Combine(RegistrySettings.logsDirectory, "SteerLog_" + fileStamp + ".csv"), false);
                fixWriter.WriteLine(FixHeader);

                settingsPath = Path.Combine(RegistrySettings.logsDirectory, "SteerLog_" + fileStamp + "_settings.txt");
                lastSettings = SettingsSnapshot();
                File.WriteAllText(settingsPath, "---- " + DateTime.Now.ToString("HH:mm:ss", inv) + " logging started ----\r\n" + lastSettings);

                firstRow = true;
                lastFixTime = DateTime.MinValue;
                pendingEvents.Clear();
                IsOn = true;
                AddEvent("Logging started");
                Log.EventWriter("Control log started: SteerLog_" + fileStamp);
            }
            catch (Exception ex)
            {
                Log.ErrorWriter("CControlLog", "Start", "could not open log files", ex);
                Stop();
            }
        }

        public void Stop()
        {
            IsOn = false;
            try
            {
                fixWriter?.Close();
                moduleWriter?.Close();
            }
            catch
            {
            }
            fixWriter = null;
            moduleWriter = null;
        }

        public void AddEvent(string text)
        {
            if (IsOn)
            {
                if (pendingEvents.Length > 0) pendingEvents.Append("; ");
                pendingEvents.Append(text);
            }
        }

        /// <summary>
        /// Called when a PGN 253 arrives from the steer module.
        /// </summary>
        public void Steer253Received()
        {
            last253Time = DateTime.Now;
        }

        /// <summary>
        /// Called once per second from the watchdog timer.
        /// </summary>
        public void OneSecond()
        {
            if (IsOn)
            {
                mf.SendPgnToLoop(diagRequest);

                string settings = SettingsSnapshot();
                if (settings != lastSettings)
                {
                    lastSettings = settings;
                    AddEvent("Settings changed");
                    try
                    {
                        File.AppendAllText(settingsPath, "\r\n---- " + DateTime.Now.ToString("HH:mm:ss", inv) + " settings changed ----\r\n" + settings);
                    }
                    catch
                    {
                    }
                }

                try
                {
                    fixWriter?.Flush();
                    moduleWriter?.Flush();
                }
                catch (Exception ex)
                {
                    Log.ErrorWriter("CControlLog", "Flush", "write failed, logging stopped", ex);
                    Stop();
                }
            }
        }

        /// <summary>
        /// Called once per fix, right after PGN 254 is sent.
        /// </summary>
        public void WriteFixRow()
        {
            if (IsOn && fixWriter != null) WriteFix();
        }

        private void WriteFix()
        {
            DateTime now = DateTime.Now;
            StringBuilder sb = new StringBuilder(600);

            // ----- guidance mode and active guidance object
            string mode = GuidanceMode(out bool isTrack);
            bool isAB = mode == "AB";
            bool isCurve = isTrack && !isAB;
            bool isStanley = mf.isStanleyUsed && mode != "RecPath" && mode != "FreeDrive" && mode != "None";

            int trackIdx = isTrack ? mf.trk.idx : -1;
            string trackName = isTrack ? (mf.trk.gArr[trackIdx].name ?? "").Replace(",", " ").Replace("\"", "") : "";
            double nudge = isTrack ? mf.trk.gArr[trackIdx].nudgeDistance : 0;

            // ----- timing
            sb.Append(now.ToString("HH:mm:ss.fff", inv)).Append(',');
            sb.Append(lastFixTime == DateTime.MinValue ? "" : ((int)(now - lastFixTime).TotalMilliseconds).ToString(inv)).Append(',');
            lastFixTime = now;
            sb.Append(F(mf.gpsHz, 1)).Append(',');
            sb.Append(B(mf.timerSim.Enabled)).Append(',');

            // ----- GPS
            sb.Append(F(mf.AppModel.CurrentLatLon.Latitude, 8)).Append(',');
            sb.Append(F(mf.AppModel.CurrentLatLon.Longitude, 8)).Append(',');
            sb.Append(F(mf.pn.fix.easting, 3)).Append(',');
            sb.Append(F(mf.pn.fix.northing, 3)).Append(',');
            sb.Append(F(mf.pivotAxlePos.easting, 3)).Append(',');
            sb.Append(F(mf.pivotAxlePos.northing, 3)).Append(',');
            sb.Append(mf.pn.fixQuality.ToString(inv)).Append(',');
            sb.Append(mf.pn.satellitesTracked.ToString(inv)).Append(',');
            sb.Append(F(mf.pn.hdop, 2)).Append(',');
            sb.Append(F(mf.pn.age, 1)).Append(',');
            sb.Append(F(mf.pn.altitude, 2)).Append(',');
            sb.Append(F(mf.pn.speed, 2)).Append(',');
            sb.Append(F(mf.avgSpeed, 2)).Append(',');
            sb.Append(B(mf.isReverse)).Append(',');
            sb.Append(B(mf.isChangingDirection)).Append(',');

            // ----- heading and roll
            sb.Append(mf.headingFromSource).Append(',');
            sb.Append(F(glm.toDegrees(mf.gpsHeading), 2)).Append(',');
            sb.Append(F(mf.pn.headingTrueDual, 2)).Append(',');
            sb.Append(mf.ahrs.imuHeading == 99999 ? "" : F(mf.ahrs.imuHeading, 2)).Append(',');
            sb.Append(F(glm.toDegrees(mf.imuCorrected), 2)).Append(',');
            sb.Append(F(glm.toDegrees(mf.fixHeading), 2)).Append(',');
            sb.Append(mf.ahrs.imuRoll == 88888 ? "" : F(mf.ahrs.imuRoll, 2)).Append(',');
            sb.Append(F(mf.ahrs.imuYawRate, 2)).Append(',');

            // ----- guidance
            sb.Append(mode).Append(',');
            sb.Append(mode == "None" || mode == "FreeDrive" ? "" : (isStanley ? "Stanley" : "PurePursuit")).Append(',');
            sb.Append(isTrack ? trackIdx.ToString(inv) : "").Append(',');
            sb.Append(trackName).Append(',');
            sb.Append(isAB ? mf.ABLine.howManyPathsAway.ToString(inv) : (isCurve ? mf.curve.howManyPathsAway.ToString(inv) : "")).Append(',');
            sb.Append(isTrack ? F(nudge, 3) : "").Append(',');

            // lightbarDistance holds the guidance distance before the "steer off" sentinel overwrites it
            bool hasXte = !mf.vehicle.isInFreeDriveMode && mf.lightbarDistance < 29000;
            sb.Append(hasXte ? F(mf.lightbarDistance, 0) : "").Append(',');
            sb.Append(isTrack && hasXte ? F(mf.vehicle.modeActualHeadingError, 2) : "").Append(',');
            sb.Append(hasXte && !isStanley ? F(mf.vehicle.goalDistance, 2) : "").Append(',');
            sb.Append(hasXte ? F(Integral(mode, isAB, isStanley), 4) : "").Append(',');
            sb.Append(hasXte && !isStanley ? F(PPRadius(mode, isAB), 2) : "").Append(',');
            sb.Append(hasXte && isStanley ? F(glm.toDegrees(mf.gyd.xTrackSteerCorrection), 2) : "").Append(',');

            double steerCmd = mf.guidanceLineSteerAngle * 0.01;
            double steerSent = (short)((mf.p_254.pgn[mf.p_254.steerAngleHi] << 8) | mf.p_254.pgn[mf.p_254.steerAngleLo]) * 0.01;
            sb.Append(mf.ahrs.imuRoll == 88888 ? "" : F(mf.ahrs.imuRoll * -mf.gyd.sideHillCompFactor, 2)).Append(',');
            sb.Append(F(steerCmd, 2)).Append(',');
            sb.Append(F(steerSent, 2)).Append(',');
            sb.Append(B(mf.vehicle.isInDeadZone)).Append(',');
            sb.Append(B(Math.Abs(steerCmd) >= mf.vehicle.maxSteerAngle - 0.01)).Append(',');

            // ----- engagement and machine state
            byte status = mf.p_254.pgn[mf.p_254.status];
            sb.Append(B(mf.isBtnAutoSteerOn)).Append(',');
            sb.Append(status.ToString(inv)).Append(',');
            sb.Append(B(mf.mc.steerSwitchHigh)).Append(',');
            sb.Append(B(mf.mc.workSwitchHigh)).Append(',');
            sb.Append(B(mf.yt.isYouTurnTriggered)).Append(',');
            sb.Append(mf.yt.youTurnPhase.ToString(inv)).Append(',');
            sb.Append(B(mf.ct.isLocked)).Append(',');
            sb.Append(mf.p_254.pgn[mf.p_254.sc9to16].ToString("X2", inv)).Append(mf.p_254.pgn[mf.p_254.sc1to8].ToString("X2", inv)).Append(',');

            // ----- module response (PGN 253)
            sb.Append(F(mf.mc.actualSteerAngleDegrees, 2)).Append(',');
            sb.Append(F(steerSent - mf.mc.actualSteerAngleDegrees, 2)).Append(',');
            sb.Append(mf.mc.pwmDisplay.ToString(inv)).Append(',');
            sb.Append(mf.mc.sensorData.ToString(inv)).Append(',');
            sb.Append(last253Time == DateTime.MinValue ? "" : ((int)(now - last253Time).TotalMilliseconds).ToString(inv)).Append(',');

            // ----- events
            DetectEvents(mode, trackIdx, nudge, status);
            if (pendingEvents.Length > 0)
            {
                sb.Append('"').Append(pendingEvents.ToString().Replace("\"", "'")).Append('"');
                pendingEvents.Clear();
            }

            try
            {
                fixWriter.WriteLine(sb.ToString());
            }
            catch (Exception ex)
            {
                Log.ErrorWriter("CControlLog", "WriteFixRow", "write failed, logging stopped", ex);
                Stop();
            }
        }

        /// <summary>
        /// Called when a PGN 249 diagnostic packet arrives from the steer module.
        /// </summary>
        public void WriteModuleRow(byte[] data)
        {
            if (IsOn && data.Length == DiagPacketLength) WriteModule(data);
        }

        private void WriteModule(byte[] data)
        {
            try
            {
                if (moduleWriter == null)
                {
                    moduleWriter = new StreamWriter(Path.Combine(RegistrySettings.logsDirectory, "SteerModuleLog_" + fileStamp + ".csv"), false);
                    moduleWriter.WriteLine(ModuleHeader);
                }

                uint millis = BitConverter.ToUInt32(data, 5);
                ushort was = BitConverter.ToUInt16(data, 9);
                short zeroOffset = BitConverter.ToInt16(data, 11);
                double actual = BitConverter.ToInt16(data, 13) * 0.01;
                double setPoint = BitConverter.ToInt16(data, 15) * 0.01;
                short cmdPWM = BitConverter.ToInt16(data, 17);
                short outPWM = BitConverter.ToInt16(data, 19);
                byte maxPWM = data[21];
                byte flags = data[22];
                byte switchByte = data[23];
                ushort setPointAge = BitConverter.ToUInt16(data, 24);
                double speed = BitConverter.ToUInt16(data, 26) * 0.1;
                short imuHeading = BitConverter.ToInt16(data, 28);
                short imuRoll = BitConverter.ToInt16(data, 30);
                ushort maxLoop = BitConverter.ToUInt16(data, 32);

                StringBuilder sb = new StringBuilder(200);
                sb.Append(DateTime.Now.ToString("HH:mm:ss.fff", inv)).Append(',');
                sb.Append(millis.ToString(inv)).Append(',');
                sb.Append(was.ToString(inv)).Append(',');
                sb.Append(zeroOffset.ToString(inv)).Append(',');
                sb.Append(F(actual, 2)).Append(',');
                sb.Append(F(setPoint, 2)).Append(',');
                sb.Append(F(setPoint - actual, 2)).Append(',');
                sb.Append(cmdPWM.ToString(inv)).Append(',');
                sb.Append(outPWM.ToString(inv)).Append(',');
                sb.Append(maxPWM.ToString(inv)).Append(',');
                for (int i = 0; i < 6; i++) sb.Append(B((flags & (1 << i)) != 0)).Append(',');
                sb.Append(switchByte.ToString(inv)).Append(',');
                sb.Append(setPointAge.ToString(inv)).Append(',');
                sb.Append(F(speed, 1)).Append(',');
                sb.Append(imuHeading == 9999 ? "" : F(imuHeading * 0.1, 1)).Append(',');
                sb.Append(imuRoll == 8888 ? "" : F(imuRoll * 0.1, 1)).Append(',');
                sb.Append(maxLoop.ToString(inv));

                moduleWriter.WriteLine(sb.ToString());
            }
            catch (Exception ex)
            {
                Log.ErrorWriter("CControlLog", "WriteModuleRow", "write failed, logging stopped", ex);
                Stop();
            }
        }

        private string GuidanceMode(out bool isTrack)
        {
            isTrack = false;
            string Result;
            if (mf.vehicle.isInFreeDriveMode) Result = "FreeDrive";
            else if (mf.recPath.isDrivingRecordedPath || mf.recPath.isFollowingDubinsToPath) Result = "RecPath";
            else if (mf.ct.isContourBtnOn) Result = "Contour";
            else if (mf.trk.gArr != null && mf.trk.idx >= 0 && mf.trk.idx < mf.trk.gArr.Count)
            {
                isTrack = true;
                Result = mf.trk.gArr[mf.trk.idx].mode.ToString();
            }
            else Result = "None";
            return Result;
        }

        private double Integral(string mode, bool isAB, bool isStanley)
        {
            double Result;
            if (isStanley) Result = mf.gyd.inty;
            else if (isAB) Result = mf.ABLine.inty;
            else if (mode == "Contour") Result = mf.ct.inty;
            else if (mode == "RecPath") Result = mf.recPath.inty;
            else Result = mf.curve.inty;
            return Result;
        }

        private double PPRadius(string mode, bool isAB)
        {
            double Result;
            if (isAB) Result = mf.ABLine.ppRadiusAB;
            else if (mode == "Contour") Result = mf.ct.ppRadiusCT;
            else if (mode == "RecPath") Result = mf.recPath.ppRadiusRP;
            else Result = mf.curve.ppRadiusCu;
            return Result;
        }

        private void DetectEvents(string mode, int trackIdx, double nudge, byte status)
        {
            bool btnOn = mf.isBtnAutoSteerOn;
            bool steerSwitch = mf.mc.steerSwitchHigh;

            if (!firstRow)
            {
                if (btnOn && !prevBtnOn) AddEvent("AutoSteer on");
                if (!btnOn && prevBtnOn)
                {
                    string reason;
                    if (mf.avgSpeed > mf.vehicle.maxSteerSpeed) reason = "above max speed";
                    else if (mf.vehicle.minSteerSpeed >= 0.1 && mf.avgSpeed < mf.vehicle.minSteerSpeed) reason = "below min speed";
                    else if (steerSwitch != prevSteerSwitch) reason = "steer switch";
                    else reason = "button or other";
                    AddEvent("AutoSteer off (" + reason + ")");
                }
                if (btnOn && prevBtnOn && status != prevStatus)
                {
                    if (status == 0) AddEvent("Steer status off (" + (mf.isReverse ? "reverse" : "no heading or direction change") + ")");
                    else AddEvent("Steer status on");
                }
                if (steerSwitch != prevSteerSwitch) AddEvent("Steer switch " + (steerSwitch ? "high" : "low"));
                if (mf.mc.workSwitchHigh != prevWorkSwitch) AddEvent("Work switch " + (mf.mc.workSwitchHigh ? "high" : "low"));
                if (mf.pn.fixQuality != prevFixQuality) AddEvent("Fix quality " + prevFixQuality.ToString(inv) + " -> " + mf.pn.fixQuality.ToString(inv));
                if (mf.headingFromSource != prevHeadingSource) AddEvent("Heading source " + mf.headingFromSource);
                if (mode != prevMode) AddEvent("Guidance mode " + mode);
                else if (trackIdx != prevTrackIdx) AddEvent("Track changed");
                if (trackIdx == prevTrackIdx && nudge != prevNudge) AddEvent("Nudge " + F(nudge, 3) + " m");
                if (mf.yt.isYouTurnTriggered != prevUTurn) AddEvent(mf.yt.isYouTurnTriggered ? "U-turn started" : "U-turn ended");
                if (mf.isReverse != prevReverse) AddEvent(mf.isReverse ? "Reverse" : "Forward");
            }

            firstRow = false;
            prevBtnOn = btnOn;
            prevSteerSwitch = steerSwitch;
            prevWorkSwitch = mf.mc.workSwitchHigh;
            prevStatus = status;
            prevFixQuality = mf.pn.fixQuality;
            prevHeadingSource = mf.headingFromSource;
            prevMode = mode;
            prevTrackIdx = trackIdx;
            prevNudge = nudge;
            prevUTurn = mf.yt.isYouTurnTriggered;
            prevReverse = mf.isReverse;
        }

        private string SettingsSnapshot()
        {
            StringBuilder sb = new StringBuilder(2000);
            CVehicle v = mf.vehicle;
            byte[] p252 = mf.p_252.pgn;
            byte[] p251 = mf.p_251.pgn;

            sb.Append("[Vehicle]\r\n");
            Line(sb, "Wheelbase", F(v.VehicleConfig.Wheelbase, 3));
            Line(sb, "AntennaHeight", F(v.VehicleConfig.AntennaHeight, 3));
            Line(sb, "AntennaPivot", F(v.VehicleConfig.AntennaPivot, 3));
            Line(sb, "AntennaOffset", F(v.VehicleConfig.AntennaOffset, 3));
            Line(sb, "MaxSteerAngle", F(v.maxSteerAngle, 1));
            Line(sb, "MaxAngularVelocity", F(v.maxAngularVelocity, 3));
            Line(sb, "MaxSteerSpeed", F(v.maxSteerSpeed, 1));
            Line(sb, "MinSteerSpeed", F(v.minSteerSpeed, 1));
            Line(sb, "SteerInReverse", B(mf.isSteerInReverse));

            sb.Append("[Guidance]\r\n");
            Line(sb, "Algorithm", mf.isStanleyUsed ? "Stanley" : "PurePursuit");
            Line(sb, "StanleyDistanceGain", F(v.stanleyDistanceErrorGain, 3));
            Line(sb, "StanleyHeadingGain", F(v.stanleyHeadingErrorGain, 3));
            Line(sb, "StanleyIntegral", F(v.stanleyIntegralGainAB, 3));
            Line(sb, "PPLookAheadHold", F(v.goalPointLookAheadHold, 3));
            Line(sb, "PPLookAheadMult", F(v.goalPointLookAheadMult, 3));
            Line(sb, "PPAcquireFactor", F(v.goalPointAcquireFactor, 3));
            Line(sb, "PPIntegral", F(v.purePursuitIntegralGain, 3));
            Line(sb, "UTurnCompensation", F(v.uturnCompensation, 3));
            Line(sb, "SideHillComp", F(mf.gyd.sideHillCompFactor, 3));
            Line(sb, "DeadZoneHeading", v.deadZoneHeading.ToString(inv));
            Line(sb, "DeadZoneDelay", v.deadZoneDelay.ToString(inv));
            Line(sb, "ModeXTE", F(v.modeXTE, 3));
            Line(sb, "ModeTime", v.modeTime.ToString(inv));

            sb.Append("[IMU and Heading]\r\n");
            Line(sb, "FusionWeight", F(mf.ahrs.fusionWeight, 3));
            Line(sb, "ForwardComp", F(mf.ahrs.forwardComp, 3));
            Line(sb, "ReverseComp", F(mf.ahrs.reverseComp, 3));
            Line(sb, "RollZero", F(mf.ahrs.rollZero, 2));
            Line(sb, "RollFilter", F(mf.ahrs.rollFilter, 3));
            Line(sb, "RollInvert", B(mf.ahrs.isRollInvert));
            Line(sb, "DualAsIMU", B(mf.ahrs.isDualAsIMU));
            Line(sb, "DualHeadingOffset", F(mf.pn.headingTrueDualOffset, 2));

            sb.Append("[Steer Module PGN 252]\r\n");
            Line(sb, "Kp", p252[mf.p_252.gainProportional].ToString(inv));
            Line(sb, "HighPWM", p252[mf.p_252.highPWM].ToString(inv));
            Line(sb, "LowPWM", p252[mf.p_252.lowPWM].ToString(inv));
            Line(sb, "MinPWM", p252[mf.p_252.minPWM].ToString(inv));
            Line(sb, "CountsPerDegree", p252[mf.p_252.countsPerDegree].ToString(inv));
            Line(sb, "WASOffset", ((short)((p252[mf.p_252.wasOffsetHi] << 8) | p252[mf.p_252.wasOffsetLo])).ToString(inv));
            Line(sb, "Ackerman", p252[mf.p_252.ackerman].ToString(inv));

            sb.Append("[Steer Module PGN 251]\r\n");
            byte set0 = p251[mf.p_251.set0];
            byte set1 = p251[mf.p_251.set1];
            Line(sb, "InvertWAS", B((set0 & 1) != 0));
            Line(sb, "InvertRelays", B((set0 & 2) != 0));
            Line(sb, "InvertSteer", B((set0 & 4) != 0));
            Line(sb, "SingleInputWAS", B((set0 & 8) != 0));
            Line(sb, "CytronDriver", B((set0 & 16) != 0));
            Line(sb, "SteerSwitch", B((set0 & 32) != 0));
            Line(sb, "SteerButton", B((set0 & 64) != 0));
            Line(sb, "ShaftEncoder", B((set0 & 128) != 0));
            Line(sb, "Danfoss", B((set1 & 1) != 0));
            Line(sb, "PressureSensor", B((set1 & 2) != 0));
            Line(sb, "CurrentSensor", B((set1 & 4) != 0));
            Line(sb, "IMU_Y_Axis", B((set1 & 8) != 0));
            Line(sb, "MaxPulse", p251[mf.p_251.maxPulse].ToString(inv));
            Line(sb, "MinSpeed", F(p251[mf.p_251.minSpeed] * 0.1, 1));

            return sb.ToString();
        }

        private static void Line(StringBuilder sb, string name, string value)
        {
            sb.Append(name).Append('=').Append(value).Append("\r\n");
        }

        private string F(double value, int decimals)
        {
            return value.ToString("F" + decimals.ToString(inv), inv);
        }

        private static string B(bool value)
        {
            return value ? "1" : "0";
        }
    }
}
