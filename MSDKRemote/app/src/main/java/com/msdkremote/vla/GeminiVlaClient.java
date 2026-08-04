package com.msdkremote.vla;

import android.util.Base64;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

/**
 * Minimal client for the Gemini API (generateContent), used as the
 * vision-language-action policy of the drone.
 *
 * Each call sends the pilot instruction, the latest camera frame and a
 * short action history, and expects a strict JSON decision back:
 * {"reasoning": "...", "action": {"type": "...", ...}}
 */
public class GeminiVlaClient
{
    private static final String ENDPOINT_FORMAT =
            "https://generativelanguage.googleapis.com/v1beta/models/%s:generateContent";

    private static final int CONNECT_TIMEOUT_MS = 15_000;
    private static final int READ_TIMEOUT_MS = 90_000;

    /** Longest action sequence accepted per decision; extras are dropped. */
    public static final int MAX_ACTIONS_PER_DECISION = 8;

    private static final String SYSTEM_PROMPT =
            "You are the flight policy of a small quadcopter camera drone (DJI Mini 3). " +
            "On every turn you receive: the pilot's instruction, your current mission plan, " +
            "the latest image from the drone's forward-facing camera, telemetry (including " +
            "GPS position, home point and gimbal pitch), and the history of your previous " +
            "actions with the GPS position where each was taken. Decide the next actions.\n" +
            "\n" +
            "Respond with STRICT JSON only, no markdown, exactly this shape:\n" +
            "{\"reasoning\": \"<one or two short sentences>\", " +
            "\"plan\": \"<your updated mission plan>\", \"actions\": [{...}, ...]}\n" +
            "\n" +
            "The plan is your only long-term memory and is echoed back to you on the next " +
            "turn. Keep it a short numbered list of steps with progress markers, e.g. " +
            "'1. [done] take off 2. [now] search yard for the red car 3. return to home " +
            "point and land'. Record important observations and GPS coordinates of places " +
            "you want to come back to (waypoints, the takeoff point, found objects). " +
            "Update it every turn; rewrite it if the situation changes.\n" +
            "\n" +
            "\"actions\" is a sequence of 1 to " + MAX_ACTIONS_PER_DECISION + " actions " +
            "executed back-to-back WITHOUT consulting you in between - you only see a new " +
            "camera image after the whole sequence. Commit to long sequences when you are " +
            "confident about the environment (open space, target far away, known route); " +
            "return a single short action when you need to re-observe soon (searching, " +
            "close to obstacles, target may move).\n" +
            "\n" +
            "Each action object must be one of:\n" +
            "{\"type\": \"takeoff\"} - start motors and climb to a ~1.2 m hover. " +
            "Required first action when the drone is not flying.\n" +
            "{\"type\": \"move\", \"forward_mps\": f, \"right_mps\": f, \"up_mps\": f, " +
            "\"yaw_dps\": f, \"duration_s\": f} - fly with these body-frame velocities for the " +
            "given duration, then hover. forward_mps > 0 flies the direction the camera faces, " +
            "right_mps > 0 strafes right, up_mps > 0 climbs, yaw_dps > 0 rotates clockwise " +
            "seen from above. Unused fields may be 0.\n" +
            "{\"type\": \"gimbal\", \"pitch_deg\": f} - point the camera: rotate the gimbal " +
            "to this absolute pitch. 0 = level with the horizon, -90 = straight down, " +
            "up to +" + (int) VlaAction.MAX_GIMBAL_PITCH + " = upward. The gimbal only " +
            "pitches; to look left or right, yaw the aircraft with a move action. Tilt " +
            "down to inspect the ground or find landing spots, level for navigation.\n" +
            "{\"type\": \"goto_gps\", \"latitude\": f, \"longitude\": f} - fly a whole " +
            "navigation leg: the drone turns to face the target, flies there at the current " +
            "altitude and stops within ~2 m of it. Use coordinates from telemetry, from your " +
            "plan, or the home point (e.g. to return to takeoff). Legs are limited to " +
            (int) VlaAction.MAX_GOTO_DISTANCE_M + " m - for longer routes fly several legs. " +
            "Needs a good GPS fix.\n" +
            "{\"type\": \"land\"} - descend and land at the current position.\n" +
            "{\"type\": \"done\"} - the instruction is fulfilled (or cannot be fulfilled " +
            "safely); the drone keeps hovering. Only valid as the last action.\n" +
            "\n" +
            "Limits (violations are clamped): |forward_mps| and |right_mps| <= " + VlaAction.MAX_HORIZONTAL_SPEED +
            ", |up_mps| <= " + VlaAction.MAX_VERTICAL_SPEED +
            ", |yaw_dps| <= " + VlaAction.MAX_YAW_RATE +
            ", 0.1 <= duration_s <= " + (VlaAction.MAX_DURATION_MS / 1000.0) +
            ", " + (int) VlaAction.MIN_GIMBAL_PITCH + " <= pitch_deg <= " + (int) VlaAction.MAX_GIMBAL_PITCH + ".\n" +
            "\n" +
            "Flying guidelines:\n" +
            "- The camera image is your only perception of the world, and you fly blind " +
            "inside a sequence. Match sequence length and speed to your confidence: sweep " +
            "wide open areas with long fast legs, creep near structures with short slow ones.\n" +
            "- A useful pattern: one sequence = one leg of the plan, e.g. " +
            "[gimbal level, yaw to the target bearing, long forward move] or " +
            "[climb, goto_gps waypoint, gimbal -90 to inspect].\n" +
            "- Prefer rotating in place (yaw) to search for a target before translating.\n" +
            "- Use the GPS positions in telemetry and history to localise yourself: they tell " +
            "you how far you have actually moved and where previous observations were made.\n" +
            "- The camera image is undistorted; things at the image centre are along the " +
            "current heading. Combine heading + GPS to estimate coordinates of seen objects.\n" +
            "- Keep generous distance from people, walls, trees and obstacles; if an obstacle " +
            "fills the view, stop or back away. goto_gps does NOT avoid obstacles - only use " +
            "it when the path ahead is clearly free, at a safe altitude.\n" +
            "- Never descend below ~1 m unless landing.\n" +
            "- If the instruction is complete, ambiguous, or cannot be done safely, return " +
            "\"done\" and say why in the reasoning.";

    private final String apiKey;
    private final String modelName;

    public GeminiVlaClient(@NonNull String apiKey, @NonNull String modelName)
    {
        this.apiKey = apiKey;
        this.modelName = modelName;
    }

    /**
     * Ask the model for the next action. Blocking call, run off the UI thread.
     *
     * @param instruction the pilot instruction (from voice recognition).
     * @param jpegImage   latest camera frame as JPEG.
     * @param telemetry   short human-readable telemetry line.
     * @param plan        the plan the model returned on the previous step, or null.
     * @param history     summaries of the previous steps, oldest first.
     */
    @NonNull
    public VlaDecision decide(@NonNull String instruction,
                              @NonNull byte[] jpegImage,
                              @NonNull String telemetry,
                              @Nullable String plan,
                              @NonNull List<String> history)
            throws IOException, JSONException
    {
        JSONObject request = buildRequest(instruction, jpegImage, telemetry, plan, history);
        String responseText = post(request.toString());
        return parseDecision(responseText);
    }

    private JSONObject buildRequest(String instruction, byte[] jpegImage,
                                    String telemetry, @Nullable String plan,
                                    List<String> history)
            throws JSONException
    {
        StringBuilder text = new StringBuilder();
        text.append("Pilot instruction: ").append(instruction).append('\n');
        text.append("Telemetry: ").append(telemetry).append('\n');

        if (plan == null || plan.trim().isEmpty())
            text.append("Current plan: none yet - draft one for this mission.\n");
        else
            text.append("Current plan (yours, from the previous step): ")
                    .append(plan.trim()).append('\n');

        if (history.isEmpty()) {
            text.append("History: none, this is the first step.");
        }
        else {
            text.append("History of previous steps:\n");
            for (String step : history)
                text.append("- ").append(step).append('\n');
        }
        text.append("\nThe attached image is the current camera view. Decide the next action.");

        JSONObject imagePart = new JSONObject().put(
                "inlineData", new JSONObject()
                        .put("mimeType", "image/jpeg")
                        .put("data", Base64.encodeToString(jpegImage, Base64.NO_WRAP)));

        JSONObject textPart = new JSONObject().put("text", text.toString());

        JSONObject content = new JSONObject()
                .put("role", "user")
                .put("parts", new JSONArray().put(imagePart).put(textPart));

        JSONObject systemInstruction = new JSONObject()
                .put("parts", new JSONArray().put(new JSONObject().put("text", SYSTEM_PROMPT)));

        JSONObject generationConfig = new JSONObject()
                .put("temperature", 0.2)
                .put("responseMimeType", "application/json");

        return new JSONObject()
                .put("systemInstruction", systemInstruction)
                .put("contents", new JSONArray().put(content))
                .put("generationConfig", generationConfig);
    }

    private String post(String body) throws IOException
    {
        URL url = new URL(String.format(ENDPOINT_FORMAT, modelName));
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();

        try {
            connection.setRequestMethod("POST");
            connection.setConnectTimeout(CONNECT_TIMEOUT_MS);
            connection.setReadTimeout(READ_TIMEOUT_MS);
            connection.setRequestProperty("Content-Type", "application/json; charset=utf-8");
            connection.setRequestProperty("x-goog-api-key", apiKey);
            connection.setDoOutput(true);

            byte[] payload = body.getBytes(StandardCharsets.UTF_8);
            connection.setFixedLengthStreamingMode(payload.length);

            try (OutputStream out = connection.getOutputStream()) {
                out.write(payload);
            }

            int status = connection.getResponseCode();

            if (status / 100 != 2) {
                String error = readStream(connection.getErrorStream());
                throw new IOException("Gemini API HTTP " + status + ": "
                        + extractApiError(error));
            }

            return readStream(connection.getInputStream());
        }
        finally {
            connection.disconnect();
        }
    }

    private static String readStream(InputStream stream) throws IOException
    {
        if (stream == null)
            return "";

        StringBuilder builder = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(stream, StandardCharsets.UTF_8))) {
            char[] buffer = new char[4096];
            int read;
            while ((read = reader.read(buffer)) != -1)
                builder.append(buffer, 0, read);
        }
        return builder.toString();
    }

    private static String extractApiError(String errorBody)
    {
        try {
            return new JSONObject(errorBody)
                    .getJSONObject("error")
                    .getString("message");
        }
        catch (JSONException e) {
            return errorBody.length() > 400 ? errorBody.substring(0, 400) : errorBody;
        }
    }

    @NonNull
    private static VlaDecision parseDecision(String responseBody)
            throws JSONException
    {
        JSONObject response = new JSONObject(responseBody);

        JSONArray candidates = response.optJSONArray("candidates");
        if (candidates == null || candidates.length() == 0) {
            String feedback = response.optString("promptFeedback", "no candidates");
            throw new JSONException("Model returned no candidates: " + feedback);
        }

        JSONObject content = candidates.getJSONObject(0).optJSONObject("content");
        JSONArray parts = (content != null) ? content.optJSONArray("parts") : null;

        StringBuilder textBuilder = new StringBuilder();
        if (parts != null) {
            for (int i = 0; i < parts.length(); i++) {
                String partText = parts.getJSONObject(i).optString("text", "");
                textBuilder.append(partText);
            }
        }

        String text = textBuilder.toString().trim();

        // Defensive: strip markdown fences if the model added them anyway.
        if (text.startsWith("```")) {
            int firstNewline = text.indexOf('\n');
            int lastFence = text.lastIndexOf("```");
            if (firstNewline >= 0 && lastFence > firstNewline)
                text = text.substring(firstNewline + 1, lastFence).trim();
        }

        if (text.isEmpty())
            throw new JSONException("Model returned empty text (finishReason: "
                    + candidates.getJSONObject(0).optString("finishReason", "unknown") + ")");

        JSONObject decision = new JSONObject(text);
        String reasoning = decision.optString("reasoning", "(no reasoning)");
        String plan = decision.optString("plan", "").trim();

        List<VlaAction> actions = new ArrayList<>();

        JSONArray actionsJson = decision.optJSONArray("actions");
        if (actionsJson != null) {
            int count = Math.min(actionsJson.length(), MAX_ACTIONS_PER_DECISION);
            for (int i = 0; i < count; i++)
                actions.add(VlaAction.fromJson(actionsJson.getJSONObject(i)));
        }
        else {
            // Tolerate the old single-action shape.
            JSONObject actionJson = decision.optJSONObject("action");
            if (actionJson != null)
                actions.add(VlaAction.fromJson(actionJson));
        }

        if (actions.isEmpty())
            throw new JSONException("Model response has no 'actions' array: " + text);

        return new VlaDecision(reasoning, plan.isEmpty() ? null : plan, actions);
    }
}
