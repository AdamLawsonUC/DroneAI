package com.msdkremote.vla;

import androidx.annotation.NonNull;

import org.json.JSONObject;

import java.util.Locale;

/**
 * A single flight action decided by the VLA model.
 * All numeric values are clamped to the safety limits below,
 * no matter what the model returns.
 */
public class VlaAction
{
    public enum Type { TAKEOFF, LAND, MOVE, DONE }

    // Hard safety limits - the model cannot exceed these.
    public static final double MAX_HORIZONTAL_SPEED = 1.0;  // m/s
    public static final double MAX_VERTICAL_SPEED   = 0.5;  // m/s
    public static final double MAX_YAW_RATE         = 45.0; // deg/s
    public static final long   MIN_DURATION_MS      = 100;
    public static final long   MAX_DURATION_MS      = 4000;

    public final Type type;

    // Body frame velocities, held for durationMs (MOVE only).
    public final double forward;  // m/s, camera direction positive
    public final double right;    // m/s, strafe right positive
    public final double up;       // m/s, climb positive
    public final double yawRate;  // deg/s, clockwise (from above) positive
    public final long durationMs;

    private VlaAction(Type type, double forward, double right,
                      double up, double yawRate, long durationMs)
    {
        this.type = type;
        this.forward = forward;
        this.right = right;
        this.up = up;
        this.yawRate = yawRate;
        this.durationMs = durationMs;
    }

    private static double clamp(double value, double limit)
    {
        if (!Double.isFinite(value)) return 0.0;
        return Math.max(-limit, Math.min(limit, value));
    }

    /**
     * Parse an action object returned by the model, e.g.
     * {"type":"move","forward_mps":0.5,"right_mps":0,"up_mps":0,"yaw_dps":0,"duration_s":2}
     *
     * @throws IllegalArgumentException if the type is missing or unknown.
     */
    @NonNull
    public static VlaAction fromJson(@NonNull JSONObject json)
    {
        String typeName = json.optString("type", "").trim().toUpperCase(Locale.ROOT);

        final Type type;
        switch (typeName) {
            case "TAKEOFF": type = Type.TAKEOFF; break;
            case "LAND":    type = Type.LAND;    break;
            case "MOVE":    type = Type.MOVE;    break;
            case "DONE":
            case "STOP":    type = Type.DONE;    break;
            default:
                throw new IllegalArgumentException("Unknown action type: '" + typeName + "'");
        }

        if (type != Type.MOVE)
            return new VlaAction(type, 0, 0, 0, 0, 0);

        double forward = clamp(json.optDouble("forward_mps", 0.0), MAX_HORIZONTAL_SPEED);
        double right   = clamp(json.optDouble("right_mps",   0.0), MAX_HORIZONTAL_SPEED);
        double up      = clamp(json.optDouble("up_mps",      0.0), MAX_VERTICAL_SPEED);
        double yawRate = clamp(json.optDouble("yaw_dps",     0.0), MAX_YAW_RATE);

        double durationS = json.optDouble("duration_s", 1.0);
        if (!Double.isFinite(durationS)) durationS = 1.0;
        long durationMs = Math.max(MIN_DURATION_MS,
                Math.min(MAX_DURATION_MS, (long) (durationS * 1000.0)));

        return new VlaAction(type, forward, right, up, yawRate, durationMs);
    }

    @NonNull
    @Override
    public String toString()
    {
        if (type != Type.MOVE)
            return type.name();

        return String.format(Locale.ENGLISH,
                "MOVE(fwd=%.2f m/s, right=%.2f m/s, up=%.2f m/s, yaw=%.1f deg/s, %.1f s)",
                forward, right, up, yawRate, durationMs / 1000.0);
    }
}
