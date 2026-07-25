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
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.common.EmptyMsg;
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem;
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam;
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode;
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

    private static final int MAX_STEPS = 60;
    private static final int HISTORY_IN_PROMPT = 10;

    private static final int FRAME_MAX_WIDTH = 640;
    private static final int FRAME_JPEG_QUALITY = 70;
    private static final long FRAME_TIMEOUT_MS = 5000;

    private static final long STICK_PERIOD_MS = 100;
    private static final long TAKEOFF_TIMEOUT_MS = 20_000;
    private static final long LANDING_TIMEOUT_MS = 60_000;
    private static final long ACTION_TIMEOUT_S = 10;

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
                            instruction, jpeg, telemetryLine(), tail(history));
                }
                catch (Exception e) {
                    throw new MissionAbort("Gemini request failed: " + e.getMessage());
                }

                if (!running)
                    break;

                listener.onDecision(step, decision);
                history.add("step " + step + ": " + decision.action
                        + " (" + decision.reasoning + ")");

                VlaAction action = decision.action;

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
        return String.format(Locale.ENGLISH,
                "flying=%b, relative altitude=%.1f m, compass heading=%.0f deg",
                isFlying, altitude, heading);
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
        final CountDownLatch latch = new CountDownLatch(1);
        final AtomicReference<String> error = new AtomicReference<>(null);

        KeyManager.getInstance().performAction(
                KeyTools.createKey(actionKey),
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
