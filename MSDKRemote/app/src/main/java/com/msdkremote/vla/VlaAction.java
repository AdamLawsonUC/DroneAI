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
    public enum Type { TAKEOFF, LAND, MOVE, GIMBAL, GOTO_GPS, DONE }

    // Hard safety limits - the model cannot exceed these.
    public static final double MAX_HORIZONTAL_SPEED = 3.0;  // m/s
    public static final double MAX_VERTICAL_SPEED   = 1.5;  // m/s
    public static final double MAX_YAW_RATE         = 90.0; // deg/s
    public static final long   MIN_DURATION_MS      = 100;
    public static final long   MAX_DURATION_MS      = 15_000;

    // Gimbal pitch limits (Mini 3 controllable range).
    public static final double MIN_GIMBAL_PITCH = -90.0; // deg, straight down
    public static final double MAX_GIMBAL_PITCH =  60.0; // deg, upward

    // A single goto_gps leg may not go farther than this from the
    // current position; longer routes need multiple legs with a
    // camera check in between.
    public static final double MAX_GOTO_DISTANCE_M = 200.0;

    public final Type type;

    // Body frame velocities, held for durationMs (MOVE only).
    public final double forward;  // m/s, camera direction positive
    public final double right;    // m/s, strafe right positive
    public final double up;       // m/s, climb positive
    public final double yawRate;  // deg/s, clockwise (from above) positive
    public final long durationMs;

    // Absolute gimbal pitch angle (GIMBAL only).
    public final double gimbalPitch; // deg, 0 = level, -90 = straight down

    // Target coordinates (GOTO_GPS only).
    public final double latitude;
    public final double longitude;

    private VlaAction(Type type, double forward, double right,
                      double up, double yawRate, long durationMs,
                      double gimbalPitch, double latitude, double longitude)
    {
        this.type = type;
        this.forward = forward;
        this.right = right;
        this.up = up;
        this.yawRate = yawRate;
        this.durationMs = durationMs;
        this.gimbalPitch = gimbalPitch;
        this.latitude = latitude;
        this.longitude = longitude;
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
            case "TAKEOFF":  type = Type.TAKEOFF;  break;
            case "LAND":     type = Type.LAND;     break;
            case "MOVE":     type = Type.MOVE;     break;
            case "GIMBAL":   type = Type.GIMBAL;   break;
            case "GOTO_GPS": type = Type.GOTO_GPS; break;
            case "DONE":
            case "STOP":     type = Type.DONE;     break;
            default:
                throw new IllegalArgumentException("Unknown action type: '" + typeName + "'");
        }

        switch (type) {
            case MOVE: {
                double forward = clamp(json.optDouble("forward_mps", 0.0), MAX_HORIZONTAL_SPEED);
                double right   = clamp(json.optDouble("right_mps",   0.0), MAX_HORIZONTAL_SPEED);
                double up      = clamp(json.optDouble("up_mps",      0.0), MAX_VERTICAL_SPEED);
                double yawRate = clamp(json.optDouble("yaw_dps",     0.0), MAX_YAW_RATE);

                double durationS = json.optDouble("duration_s", 1.0);
                if (!Double.isFinite(durationS)) durationS = 1.0;
                long durationMs = Math.max(MIN_DURATION_MS,
                        Math.min(MAX_DURATION_MS, (long) (durationS * 1000.0)));

                return new VlaAction(type, forward, right, up, yawRate, durationMs, 0, 0, 0);
            }

            case GIMBAL: {
                double pitch = json.optDouble("pitch_deg", 0.0);
                if (!Double.isFinite(pitch)) pitch = 0.0;
                pitch = Math.max(MIN_GIMBAL_PITCH, Math.min(MAX_GIMBAL_PITCH, pitch));

                return new VlaAction(type, 0, 0, 0, 0, 0, pitch, 0, 0);
            }

            case GOTO_GPS: {
                double latitude = json.optDouble("latitude", Double.NaN);
                double longitude = json.optDouble("longitude", Double.NaN);

                if (!Double.isFinite(latitude) || !Double.isFinite(longitude)
                        || Math.abs(latitude) > 90.0 || Math.abs(longitude) > 180.0
                        || (latitude == 0.0 && longitude == 0.0))
                    throw new IllegalArgumentException(
                            "goto_gps needs valid 'latitude' and 'longitude' fields");

                return new VlaAction(type, 0, 0, 0, 0, 0, 0, latitude, longitude);
            }

            default:
                return new VlaAction(type, 0, 0, 0, 0, 0, 0, 0, 0);
        }
    }

    @NonNull
    @Override
    public String toString()
    {
        switch (type) {
            case MOVE:
                return String.format(Locale.ENGLISH,
                        "MOVE(fwd=%.2f m/s, right=%.2f m/s, up=%.2f m/s, yaw=%.1f deg/s, %.1f s)",
                        forward, right, up, yawRate, durationMs / 1000.0);

            case GIMBAL:
                return String.format(Locale.ENGLISH,
                        "GIMBAL(pitch=%.0f deg)", gimbalPitch);

            case GOTO_GPS:
                return String.format(Locale.ENGLISH,
                        "GOTO_GPS(%.6f, %.6f)", latitude, longitude);

            default:
                return type.name();
        }
    }
}
