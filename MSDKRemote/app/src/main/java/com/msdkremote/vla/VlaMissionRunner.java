package com.msdkremote.vla;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import dji.sdk.keyvalue.key.DJIActionKeyInfo;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.common.Attitude;
import dji.sdk.keyvalue.value.common.EmptyMsg;
import dji.sdk.keyvalue.value.common.LocationCoordinate2D;
import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem;
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel;
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam;
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode;
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation;
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode;
import dji.sdk.keyvalue.value.gimbal.GimbalMode;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager;
import dji.v5.manager.interfaces.IVirtualStickManager;

/**
 * Runs one VLA mission as a closed loop on a background thread:
 *
 *   camera frame -> Gemini decision -> virtual stick execution -> repeat
 *
 * until the model reports "done", the model lands the drone, the step
 * limit is reached, or the mission is stopped from outside.
 *
 * Movement uses the advanced virtual stick mode with body-frame
 * velocities. Note the DJI parameter naming: 'roll' carries the
 * forward velocity and 'pitch' the rightward velocity (this mirrors
 * exactly how the SDK maps its own stick input in velocity mode).
 */
public class VlaMissionRunner
{
    private static final String TAG = VlaMissionRunner.class.getSimpleName();

    private static final int MAX_STEPS = 150;
    private static final int HISTORY_IN_PROMPT = 25;

    private static final int FRAME_MAX_WIDTH = 640;
    private static final int FRAME_JPEG_QUALITY = 70;
    private static final long FRAME_TIMEOUT_MS = 5000;

    private static final long STICK_PERIOD_MS = 100;
    private static final long TAKEOFF_TIMEOUT_MS = 20_000;
    private static final long LANDING_TIMEOUT_MS = 60_000;
    private static final long ACTION_TIMEOUT_S = 10;

    // Gimbal rotation pacing: rotation time plus settling before the
    // next frame is captured.
    private static final double GIMBAL_ROTATE_DURATION_S = 1.0;
    private static final long GIMBAL_SETTLE_MS = 500;
    private static final double GIMBAL_PITCH_TOLERANCE_DEG = 3.0;
    private static final long GIMBAL_VERIFY_TIMEOUT_MS = 3000;

    // goto_gps navigation controller.
    private static final double GOTO_ARRIVE_RADIUS_M = 2.0;    // stop within this
    private static final double GOTO_SLOWDOWN_RADIUS_M = 12.0; // start braking here
    private static final double GOTO_ALIGN_TOLERANCE_DEG = 25.0; // fly fwd when facing target
    private static final double GOTO_YAW_GAIN = 0.8;           // deg/s per deg of error
    private static final long GOTO_TIMEOUT_MS = 240_000;
    private static final int GOTO_MIN_SATELLITES = 8;

    /** Mission callbacks; invoked on the mission thread. */
    public interface Listener {
        void onMissionStarted(String instruction);
        void onDecision(int step, VlaDecision decision);
        void onInfo(String message);
        /** Mission is over. error == null means a normal end. */
        void onMissionFinished(String summary, @Nullable String error);
    }

    private final GeminiVlaClient client;
    private final FrameSnapshotter snapshotter;
    private final Listener listener;

    private Thread thread = null;

    private volatile boolean running = false;
    private volatile boolean emergency = false;

    // Telemetry mirrored by key listeners (listener object = this).
    // connectionKnown guards against aborting on the initial unknown
    // state - the cached value is unreliable, only listener pushes
    // (or an affirmative cached 'true') count as known.
    private volatile boolean droneConnected = false;
    private volatile boolean connectionKnown = false;
    private volatile boolean isFlying = false;
    private volatile double altitude = 0.0;
    private volatile double heading = 0.0;

    // GPS and gimbal state. Location objects are immutable value
    // holders from the SDK; null means "no fix / unknown yet".
    @Nullable private volatile LocationCoordinate3D location = null;
    @Nullable private volatile LocationCoordinate2D homeLocation = null;
    private volatile int satelliteCount = 0;
    private volatile GPSSignalLevel gpsSignalLevel = GPSSignalLevel.UNKNOWN;
    private volatile double gimbalPitch = 0.0;

    private boolean virtualStickEnabled = false;

    public VlaMissionRunner(@NonNull GeminiVlaClient client,
                            @NonNull FrameSnapshotter snapshotter,
                            @NonNull Listener listener)
    {
        this.client = client;
        this.snapshotter = snapshotter;
        this.listener = listener;
    }

    public synchronized boolean isRunning() {
        return running;
    }

    /**
     * Start a mission with the given instruction.
     * @return false if a mission is already running.
     */
    public synchronized boolean start(@NonNull String instruction)
    {
        if (running)
            return false;

        running = true;
        emergency = false;

        thread = new Thread(() -> runMission(instruction), "VlaMissionRunner");
        thread.start();
        return true;
    }

    /**
     * Ask the mission to stop. The drone stops moving and hovers,
     * control returns to the remote controller.
     */
    public synchronized void requestStop()
    {
        running = false;
        if (thread != null)
            thread.interrupt();
    }

    /**
     * Notify the runner that an emergency landing was triggered from
     * outside. The mission stops without touching the sticks again, so
     * it cannot interfere with the landing.
     */
    public synchronized void notifyEmergency()
    {
        emergency = true;
        requestStop();
    }

    // ==================== Mission loop ====================

    private void runMission(String instruction)
    {
        listener.onMissionStarted(instruction);

        registerTelemetryListeners();

        // The listeners above deliver their first values asynchronously on
        // the main thread. Seed the telemetry from the key cache right away,
        // so the first loop iteration does not see stale defaults (which
        // aborted the mission with a false "drone disconnected").
        seedTelemetry();

        List<String> history = new ArrayList<>();
        String plan = null;
        String endError = null;
        String endSummary = "Mission ended.";

        try {
            for (int step = 1; step <= MAX_STEPS && running; step++) {

                if (connectionKnown && !droneConnected)
                    throw new MissionAbort("Drone disconnected.");

                byte[] jpeg = snapshotter.captureJpeg(
                        FRAME_MAX_WIDTH, FRAME_JPEG_QUALITY, FRAME_TIMEOUT_MS);

                if (jpeg == null)
                    throw new MissionAbort("No video frame from the drone "
                            + "(is the camera stream up?).");

                listener.onInfo("Step " + step + ": asking Gemini...");

                VlaDecision decision;
                try {
                    decision = client.decide(
                            instruction, jpeg, telemetryLine(), plan, tail(history));
                }
                catch (Exception e) {
                    throw new MissionAbort("Gemini request failed: " + e.getMessage());
                }

                if (!running)
                    break;

                if (decision.plan != null)
                    plan = decision.plan;

                listener.onDecision(step, decision);
                history.add("step " + step + " @ [" + positionStamp() + "]: "
                        + decision.actionsLine() + " (" + decision.reasoning + ")");

                // Execute the whole sequence before consulting the model again.
                for (VlaAction action : decision.actions) {
                    if (!running)
                        break;

                    if (action.type == VlaAction.Type.DONE) {
                        endSummary = "Model reports mission done: " + decision.reasoning;
                        return;
                    }

                    executeAction(action);

                    if (action.type == VlaAction.Type.LAND) {
                        endSummary = "Mission ended with landing.";
                        return;
                    }
                }
            }

            if (running)
                endSummary = "Mission stopped: step limit (" + MAX_STEPS + ") reached.";
            else
                endSummary = "Mission stopped by pilot.";
        }
        catch (MissionAbort e) {
            endError = e.getMessage();
        }
        catch (InterruptedException e) {
            endSummary = "Mission stopped by pilot.";
        }
        catch (Exception e) {
            Log.e(TAG, "Unexpected mission error", e);
            endError = "Unexpected error: " + e;
        }
        finally {
            cleanup();

            synchronized (this) {
                running = false;
                thread = null;
            }

            listener.onMissionFinished(
                    endError != null ? "Mission aborted." : endSummary, endError);
        }
    }

    private String telemetryLine()
    {
        StringBuilder line = new StringBuilder(String.format(Locale.ENGLISH,
                "flying=%b, relative altitude=%.1f m, compass heading=%.0f deg, "
                        + "gimbal pitch=%.0f deg",
                isFlying, altitude, heading, gimbalPitch));

        LocationCoordinate3D loc = location;
        if (hasGpsFix(loc)) {
            line.append(String.format(Locale.ENGLISH,
                    ", gps=(%.6f, %.6f), satellites=%d (signal %s)",
                    loc.getLatitude(), loc.getLongitude(),
                    satelliteCount, gpsSignalLevel));
        }
        else {
            line.append(", gps=no fix");
        }

        LocationCoordinate2D home = homeLocation;
        if (hasGpsFix(loc) && isValidLatLon(home)) {
            double distance = distanceMeters(
                    loc.getLatitude(), loc.getLongitude(),
                    home.getLatitude(), home.getLongitude());
            line.append(String.format(Locale.ENGLISH,
                    ", home point=(%.6f, %.6f) %.0f m away at bearing %.0f deg",
                    home.getLatitude(), home.getLongitude(), distance,
                    bearingDegrees(loc.getLatitude(), loc.getLongitude(),
                            home.getLatitude(), home.getLongitude())));
        }

        return line.toString();
    }

    /** Compact position used to stamp history entries. */
    private String positionStamp()
    {
        LocationCoordinate3D loc = location;
        if (!hasGpsFix(loc))
            return String.format(Locale.ENGLISH,
                    "no gps, alt %.1f m, hdg %.0f", altitude, heading);

        return String.format(Locale.ENGLISH,
                "%.6f, %.6f, alt %.1f m, hdg %.0f",
                loc.getLatitude(), loc.getLongitude(), altitude, heading);
    }

    // ==================== Geo helpers ====================

    private static boolean hasGpsFix(@Nullable LocationCoordinate3D loc)
    {
        return loc != null && isValid(loc.getLatitude()) && isValid(loc.getLongitude())
                && !(loc.getLatitude() == 0.0 && loc.getLongitude() == 0.0);
    }

    private static boolean isValidLatLon(@Nullable LocationCoordinate2D loc)
    {
        return loc != null && isValid(loc.getLatitude()) && isValid(loc.getLongitude())
                && !(loc.getLatitude() == 0.0 && loc.getLongitude() == 0.0);
    }

    private static boolean isValid(@Nullable Double value)
    {
        return value != null && Double.isFinite(value);
    }

    /** Great-circle distance between two coordinates (haversine). */
    private static double distanceMeters(double lat1, double lon1,
                                         double lat2, double lon2)
    {
        final double earthRadius = 6_371_000.0;
        double dLat = Math.toRadians(lat2 - lat1);
        double dLon = Math.toRadians(lon2 - lon1);
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
                + Math.cos(Math.toRadians(lat1)) * Math.cos(Math.toRadians(lat2))
                * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        return earthRadius * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
    }

    /** Initial bearing from point 1 to point 2, degrees [0, 360), 0 = north. */
    private static double bearingDegrees(double lat1, double lon1,
                                         double lat2, double lon2)
    {
        double phi1 = Math.toRadians(lat1);
        double phi2 = Math.toRadians(lat2);
        double dLon = Math.toRadians(lon2 - lon1);
        double y = Math.sin(dLon) * Math.cos(phi2);
        double x = Math.cos(phi1) * Math.sin(phi2)
                - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dLon);
        double bearing = Math.toDegrees(Math.atan2(y, x));
        return (bearing + 360.0) % 360.0;
    }

    /** Normalize an angle difference to [-180, 180). */
    private static double normalizeAngle(double degrees)
    {
        double a = degrees % 360.0;
        if (a >= 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }

    private List<String> tail(List<String> history)
    {
        if (history.size() <= HISTORY_IN_PROMPT)
            return history;
        return history.subList(history.size() - HISTORY_IN_PROMPT, history.size());
    }

    // ==================== Action execution ====================

    private void executeAction(VlaAction action) throws InterruptedException, MissionAbort
    {
        switch (action.type) {
            case TAKEOFF:
                executeTakeoff();
                break;

            case LAND:
                executeLand();
                break;

            case MOVE:
                executeMove(action);
                break;

            case GIMBAL:
                executeGimbal(action);
                break;

            case GOTO_GPS:
                executeGotoGps(action);
                break;

            case DONE:
                break;
        }
    }

    private void executeTakeoff() throws InterruptedException, MissionAbort
    {
        if (isFlying) {
            listener.onInfo("Already flying - takeoff skipped.");
            return;
        }

        listener.onInfo("Taking off...");

        String error = performActionBlocking(FlightControllerKey.KeyStartTakeoff);
        if (error != null)
            throw new MissionAbort("Takeoff failed: " + error);

        // Wait until airborne at hover altitude.
        long deadline = System.currentTimeMillis() + TAKEOFF_TIMEOUT_MS;
        while (System.currentTimeMillis() < deadline) {
            if (isFlying && altitude >= 1.0)
                break;
            Thread.sleep(250);
        }

        if (!isFlying)
            throw new MissionAbort("Drone did not take off in time.");

        listener.onInfo("Airborne.");
    }

    private void executeLand() throws InterruptedException, MissionAbort
    {
        listener.onInfo("Landing...");

        // Stop any movement, then hand the landing to the flight controller.
        if (virtualStickEnabled) {
            sendVelocity(0, 0, 0, 0);
            disableVirtualStick();
        }

        String error = performActionBlocking(FlightControllerKey.KeyStartAutoLanding);
        if (error != null)
            throw new MissionAbort("Landing failed to start: " + error);

        long deadline = System.currentTimeMillis() + LANDING_TIMEOUT_MS;
        while (System.currentTimeMillis() < deadline && running) {
            if (!isFlying) {
                listener.onInfo("Landed.");
                return;
            }
            Thread.sleep(500);
        }

        if (isFlying)
            listener.onInfo("Still landing (timeout waiting for touchdown).");
    }

    private void executeMove(VlaAction action) throws InterruptedException, MissionAbort
    {
        if (!isFlying) {
            listener.onInfo("Ignored MOVE: drone is not flying (needs takeoff first).");
            return;
        }

        enableVirtualStickIfNeeded();

        long deadline = System.currentTimeMillis() + action.durationMs;
        while (System.currentTimeMillis() < deadline && running) {
            sendVelocity(action.forward, action.right, action.up, action.yawRate);
            Thread.sleep(STICK_PERIOD_MS);
        }

        // Back to hover.
        for (int i = 0; i < 3; i++) {
            sendVelocity(0, 0, 0, 0);
            Thread.sleep(STICK_PERIOD_MS);
        }
    }

    /**
     * Rotate the gimbal to an absolute pitch angle and wait until the
     * gimbal attitude telemetry confirms it arrived, so the next
     * captured frame already shows the new view.
     *
     * Mini-series quirks are worked around here: the gimbal must not be
     * in FPV mode (pitch commands are silently ignored there), and on
     * some firmwares an ABSOLUTE_ANGLE rotation is a silent no-op, in
     * which case a RELATIVE_ANGLE rotation by the remaining delta is
     * tried. A gimbal that still does not move only logs a warning -
     * the mission continues and the model sees the real pitch in the
     * telemetry.
     */
    private void executeGimbal(VlaAction action) throws InterruptedException
    {
        listener.onInfo(String.format(Locale.ENGLISH,
                "Rotating gimbal to %.0f deg pitch...", action.gimbalPitch));

        // FPV mode ignores pitch commands; make sure we are in a
        // pitch-controllable mode first (best effort).
        setGimbalModeQuietly(GimbalMode.YAW_FOLLOW);

        // Attempt 1: absolute angle.
        GimbalAngleRotation rotation = new GimbalAngleRotation();
        rotation.setMode(GimbalAngleRotationMode.ABSOLUTE_ANGLE);
        rotation.setPitch(action.gimbalPitch);
        rotation.setRollIgnored(true);
        rotation.setYawIgnored(true);
        rotation.setDuration(GIMBAL_ROTATE_DURATION_S);

        String error = performActionBlocking(GimbalKey.KeyRotateByAngle, rotation);
        if (error != null)
            listener.onInfo("Gimbal absolute rotation rejected: " + error);

        if (waitForGimbalPitch(action.gimbalPitch)) {
            Thread.sleep(GIMBAL_SETTLE_MS);
            return;
        }

        // Attempt 2: relative rotation by the remaining delta (works on
        // aircraft where absolute mode is a no-op).
        double delta = action.gimbalPitch - gimbalPitch;
        listener.onInfo(String.format(Locale.ENGLISH,
                "Gimbal did not move (pitch %.0f deg) - retrying with a %+.0f deg "
                        + "relative rotation.", gimbalPitch, delta));

        rotation = new GimbalAngleRotation();
        rotation.setMode(GimbalAngleRotationMode.RELATIVE_ANGLE);
        rotation.setPitch(delta);
        rotation.setRollIgnored(true);
        rotation.setYawIgnored(true);
        rotation.setDuration(GIMBAL_ROTATE_DURATION_S);

        error = performActionBlocking(GimbalKey.KeyRotateByAngle, rotation);
        if (error != null)
            listener.onInfo("Gimbal relative rotation rejected: " + error);

        if (waitForGimbalPitch(action.gimbalPitch))
            Thread.sleep(GIMBAL_SETTLE_MS);
        else
            listener.onInfo(String.format(Locale.ENGLISH,
                    "WARNING: gimbal stuck at %.0f deg (wanted %.0f deg) - continuing.",
                    gimbalPitch, action.gimbalPitch));
    }

    /**
     * Wait until the gimbal attitude telemetry reports the target pitch
     * (within tolerance). @return true if the gimbal got there.
     */
    private boolean waitForGimbalPitch(double targetPitch) throws InterruptedException
    {
        long deadline = System.currentTimeMillis() + GIMBAL_VERIFY_TIMEOUT_MS;
        while (System.currentTimeMillis() < deadline && running) {
            if (Math.abs(gimbalPitch - targetPitch) <= GIMBAL_PITCH_TOLERANCE_DEG)
                return true;
            Thread.sleep(100);
        }
        return Math.abs(gimbalPitch - targetPitch) <= GIMBAL_PITCH_TOLERANCE_DEG;
    }

    /** Set the gimbal mode, logging (not failing) on error. */
    private void setGimbalModeQuietly(GimbalMode mode) throws InterruptedException
    {
        final CountDownLatch latch = new CountDownLatch(1);
        final AtomicReference<String> error = new AtomicReference<>(null);

        KeyManager.getInstance().setValue(
                KeyTools.createKey(GimbalKey.KeyGimbalMode),
                mode,
                new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onSuccess() {
                        latch.countDown();
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError idjiError) {
                        error.set(String.valueOf(idjiError));
                        latch.countDown();
                    }
                });

        if (!latch.await(ACTION_TIMEOUT_S, TimeUnit.SECONDS))
            Log.w(TAG, "Setting gimbal mode timed out");
        else if (error.get() != null)
            Log.w(TAG, "Setting gimbal mode failed: " + error.get());
    }

    /**
     * Fly one navigation leg towards a GPS target: yaw until the nose
     * (and camera) faces the target, then fly forward, braking near the
     * goal. Altitude is held. Runs on the same body-frame virtual stick
     * commands as MOVE, so no new coordinate conventions are involved.
     */
    private void executeGotoGps(VlaAction action) throws InterruptedException, MissionAbort
    {
        if (!isFlying) {
            listener.onInfo("Ignored GOTO_GPS: drone is not flying (needs takeoff first).");
            return;
        }

        LocationCoordinate3D start = location;
        if (!hasGpsFix(start) || satelliteCount < GOTO_MIN_SATELLITES) {
            listener.onInfo(String.format(Locale.ENGLISH,
                    "Ignored GOTO_GPS: needs a good GPS fix (satellites=%d, need >=%d).",
                    satelliteCount, GOTO_MIN_SATELLITES));
            return;
        }

        double startDistance = distanceMeters(
                start.getLatitude(), start.getLongitude(),
                action.latitude, action.longitude);

        if (startDistance > VlaAction.MAX_GOTO_DISTANCE_M) {
            listener.onInfo(String.format(Locale.ENGLISH,
                    "Ignored GOTO_GPS: target is %.0f m away (limit %.0f m per leg).",
                    startDistance, VlaAction.MAX_GOTO_DISTANCE_M));
            return;
        }

        listener.onInfo(String.format(Locale.ENGLISH,
                "Flying to (%.6f, %.6f), %.0f m away...",
                action.latitude, action.longitude, startDistance));

        enableVirtualStickIfNeeded();

        long deadline = System.currentTimeMillis() + GOTO_TIMEOUT_MS;

        while (running && System.currentTimeMillis() < deadline) {
            LocationCoordinate3D loc = location;
            if (!hasGpsFix(loc))
                throw new MissionAbort("GPS fix lost during goto_gps.");

            double distance = distanceMeters(
                    loc.getLatitude(), loc.getLongitude(),
                    action.latitude, action.longitude);

            if (distance <= GOTO_ARRIVE_RADIUS_M) {
                listener.onInfo(String.format(Locale.ENGLISH,
                        "Arrived: %.1f m from the target.", distance));
                break;
            }

            double bearing = bearingDegrees(
                    loc.getLatitude(), loc.getLongitude(),
                    action.latitude, action.longitude);
            double headingError = normalizeAngle(bearing - heading);

            // Turn towards the target; fly forward once roughly aligned,
            // slowing down within the braking radius.
            double yawRate = Math.max(-VlaAction.MAX_YAW_RATE,
                    Math.min(VlaAction.MAX_YAW_RATE, headingError * GOTO_YAW_GAIN));

            double forward = 0.0;
            if (Math.abs(headingError) < GOTO_ALIGN_TOLERANCE_DEG) {
                forward = VlaAction.MAX_HORIZONTAL_SPEED
                        * Math.min(1.0, distance / GOTO_SLOWDOWN_RADIUS_M);
                forward = Math.max(0.2, forward);
            }

            sendVelocity(forward, 0, 0, yawRate);
            Thread.sleep(STICK_PERIOD_MS);
        }

        if (running && System.currentTimeMillis() >= deadline)
            listener.onInfo("goto_gps timed out - stopping here.");

        // Back to hover.
        for (int i = 0; i < 3; i++) {
            sendVelocity(0, 0, 0, 0);
            Thread.sleep(STICK_PERIOD_MS);
        }
    }

    /**
     * Send one body-frame velocity setpoint.
     * DJI naming: roll = forward velocity, pitch = rightward velocity,
     * yaw = clockwise angular velocity, verticalThrottle = upward velocity.
     */
    private void sendVelocity(double forward, double right, double up, double yawRate)
    {
        VirtualStickFlightControlParam param = new VirtualStickFlightControlParam();
        param.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
        param.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        param.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        param.setVerticalControlMode(VerticalControlMode.VELOCITY);
        param.setRoll(forward);
        param.setPitch(right);
        param.setYaw(yawRate);
        param.setVerticalThrottle(up);

        VirtualStickManager.getInstance().sendVirtualStickAdvancedParam(param);
    }

    // ==================== Virtual stick handling ====================

    private void enableVirtualStickIfNeeded() throws InterruptedException, MissionAbort
    {
        if (virtualStickEnabled)
            return;

        IVirtualStickManager manager = VirtualStickManager.getInstance();

        final CountDownLatch latch = new CountDownLatch(1);
        final AtomicReference<String> error = new AtomicReference<>(null);

        manager.enableVirtualStick(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                latch.countDown();
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                error.set(String.valueOf(idjiError));
                latch.countDown();
            }
        });

        if (!latch.await(ACTION_TIMEOUT_S, TimeUnit.SECONDS))
            throw new MissionAbort("Enabling virtual stick timed out.");

        if (error.get() != null)
            throw new MissionAbort("Could not enable virtual stick: " + error.get());

        manager.setVirtualStickAdvancedModeEnabled(true);
        virtualStickEnabled = true;
    }

    private void disableVirtualStick()
    {
        if (!virtualStickEnabled)
            return;

        virtualStickEnabled = false;

        IVirtualStickManager manager = VirtualStickManager.getInstance();
        manager.setVirtualStickAdvancedModeEnabled(false);
        manager.disableVirtualStick(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                Log.i(TAG, "Virtual stick disabled, control returned to RC.");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                Log.w(TAG, "Failed disabling virtual stick: " + idjiError);
            }
        });
    }

    /** Perform a parameterless DJI action and wait for its callback. */
    @Nullable
    private String performActionBlocking(DJIActionKeyInfo<EmptyMsg, EmptyMsg> actionKey)
            throws InterruptedException
    {
        return performActionBlocking(actionKey, new EmptyMsg());
    }

    /** Perform a DJI action with a parameter and wait for its callback. */
    @Nullable
    private <P> String performActionBlocking(DJIActionKeyInfo<P, EmptyMsg> actionKey, P param)
            throws InterruptedException
    {
        final CountDownLatch latch = new CountDownLatch(1);
        final AtomicReference<String> error = new AtomicReference<>(null);

        KeyManager.getInstance().performAction(
                KeyTools.createKey(actionKey),
                param,
                new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                    @Override
                    public void onSuccess(EmptyMsg emptyMsg) {
                        latch.countDown();
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError idjiError) {
                        error.set(String.valueOf(idjiError));
                        latch.countDown();
                    }
                });

        if (!latch.await(ACTION_TIMEOUT_S, TimeUnit.SECONDS))
            return "action timed out";

        return error.get();
    }

    // ==================== Telemetry ====================

    private void seedTelemetry()
    {
        // A cached 'true' is trustworthy; a cached 'false' may just be an
        // empty cache, so it does not mark the connection state as known.
        Boolean connected = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyConnection), Boolean.FALSE);
        if (Boolean.TRUE.equals(connected)) {
            droneConnected = true;
            connectionKnown = true;
        }

        Boolean flying = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyIsFlying), Boolean.FALSE);
        isFlying = (flying != null && flying);

        Double cachedAltitude = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyAltitude), 0.0);
        altitude = (cachedAltitude != null) ? cachedAltitude : 0.0;

        Double cachedHeading = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyCompassHeading), 0.0);
        heading = (cachedHeading != null) ? cachedHeading : 0.0;

        location = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D),
                (LocationCoordinate3D) null);

        homeLocation = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyHomeLocation),
                (LocationCoordinate2D) null);

        Integer cachedSatellites = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount), 0);
        satelliteCount = (cachedSatellites != null) ? cachedSatellites : 0;

        GPSSignalLevel cachedGpsLevel = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel),
                GPSSignalLevel.UNKNOWN);
        gpsSignalLevel = (cachedGpsLevel != null) ? cachedGpsLevel : GPSSignalLevel.UNKNOWN;

        Attitude cachedGimbal = KeyManager.getInstance().getValue(
                KeyTools.createKey(GimbalKey.KeyGimbalAttitude), (Attitude) null);
        if (cachedGimbal != null && isValid(cachedGimbal.getPitch()))
            gimbalPitch = cachedGimbal.getPitch();
    }

    private void registerTelemetryListeners()
    {
        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyConnection),
                this,
                (oldValue, newValue) -> {
                    if (newValue != null) {
                        droneConnected = newValue;
                        connectionKnown = true;
                    }
                });

        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyIsFlying),
                this,
                (oldValue, newValue) ->
                        isFlying = (newValue != null && newValue));

        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyAltitude),
                this,
                (oldValue, newValue) ->
                        altitude = (newValue != null) ? newValue : 0.0);

        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyCompassHeading),
                this,
                (oldValue, newValue) ->
                        heading = (newValue != null) ? newValue : 0.0);

        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D),
                this,
                (oldValue, newValue) -> {
                    if (newValue != null)
                        location = newValue;
                });

        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyHomeLocation),
                this,
                (oldValue, newValue) -> {
                    if (newValue != null)
                        homeLocation = newValue;
                });

        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount),
                this,
                (oldValue, newValue) ->
                        satelliteCount = (newValue != null) ? newValue : 0);

        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel),
                this,
                (oldValue, newValue) ->
                        gpsSignalLevel = (newValue != null) ? newValue : GPSSignalLevel.UNKNOWN);

        KeyManager.getInstance().listen(
                KeyTools.createKey(GimbalKey.KeyGimbalAttitude),
                this,
                (oldValue, newValue) -> {
                    if (newValue != null && isValid(newValue.getPitch()))
                        gimbalPitch = newValue.getPitch();
                });

        // Confirm landing (needed below ~0.7 m on some aircraft) whenever asked.
        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyIsLandingConfirmationNeeded),
                this,
                (oldValue, newValue) -> {
                    if (newValue != null && newValue) {
                        KeyManager.getInstance().performAction(
                                KeyTools.createKey(FlightControllerKey.KeyConfirmLanding),
                                new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                                    @Override
                                    public void onSuccess(EmptyMsg emptyMsg) {
                                        Log.i(TAG, "Landing confirmed.");
                                    }

                                    @Override
                                    public void onFailure(@NonNull IDJIError idjiError) {
                                        Log.w(TAG, "Landing confirmation failed: "
                                                + idjiError);
                                    }
                                });
                    }
                });
    }

    private void cleanup()
    {
        // On emergency the activity owns the aircraft (landing);
        // do not touch the sticks from here.
        if (!emergency) {
            try {
                if (virtualStickEnabled)
                    sendVelocity(0, 0, 0, 0);
            }
            catch (Exception e) {
                Log.w(TAG, "Failed zeroing sticks on cleanup", e);
            }
            disableVirtualStick();
        }

        KeyManager.getInstance().cancelListen(this);
    }

    /** Internal: aborts the mission with a user-readable error. */
    private static class MissionAbort extends Exception {
        MissionAbort(String message) {
            super(message);
        }
    }
}
