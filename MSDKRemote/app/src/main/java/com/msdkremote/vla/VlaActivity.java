package com.msdkremote.vla;

import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.speech.RecognizerIntent;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import com.msdkremote.R;

import java.util.ArrayList;
import java.util.Locale;

import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.value.common.ComponentIndexType;
import dji.sdk.keyvalue.value.common.EmptyMsg;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager;
import dji.v5.manager.datacenter.camera.CameraStreamManager;
import dji.v5.manager.interfaces.ICameraStreamManager;

/**
 * Voice-commanded VLA (vision-language-action) missions.
 *
 * The pilot speaks (or types) an instruction, the drone camera image and
 * the instruction are sent to a Gemini Robotics model in a closed loop,
 * and the returned actions are executed on the drone.
 *
 * The red EMERGENCY STOP button aborts the mission, returns stick
 * control to the remote controller and starts an automatic landing.
 */
public class VlaActivity extends AppCompatActivity
        implements VlaMissionRunner.Listener
{
    private final String TAG = this.getClass().getSimpleName();

    private final FrameSnapshotter snapshotter = new FrameSnapshotter();

    @Nullable
    private VlaMissionRunner missionRunner = null;

    // Set when the emergency button was pressed, so landing
    // confirmation requests are answered automatically.
    private volatile boolean emergencyLanding = false;

    // Live connection state pushed by the SDK (the cached getValue is
    // not reliable for this key, listeners are).
    private volatile boolean droneConnected = false;

    private EditText editInstruction;
    private Button buttonSpeak;
    private Button buttonStart;
    private Button buttonStop;
    private TextView textLog;
    private ScrollView scrollLog;

    private final ICameraStreamManager.AvailableCameraUpdatedListener cameraListener =
            availableCameraList -> {
                if (!availableCameraList.contains(ComponentIndexType.LEFT_OR_MAIN))
                    return;

                SurfaceView surfaceView = findViewById(R.id.vlaVideo);
                Surface surface = surfaceView.getHolder().getSurface();

                ICameraStreamManager manager = CameraStreamManager.getInstance();
                manager.removeCameraStreamSurface(surface);
                manager.putCameraStreamSurface(
                        ComponentIndexType.LEFT_OR_MAIN,
                        surface,
                        surfaceView.getWidth(),
                        surfaceView.getHeight(),
                        ICameraStreamManager.ScaleType.CENTER_INSIDE
                );
            };

    private final ActivityResultLauncher<Intent> speechLauncher =
            registerForActivityResult(
                    new ActivityResultContracts.StartActivityForResult(),
                    result -> {
                        if (result.getResultCode() != RESULT_OK || result.getData() == null)
                            return;

                        ArrayList<String> matches = result.getData()
                                .getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);

                        if (matches != null && !matches.isEmpty()) {
                            editInstruction.setText(matches.get(0));
                            appendLog("Heard: \"" + matches.get(0) + "\"");
                        }
                    });

    @Override
    protected void onCreate(@Nullable android.os.Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_vla);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        editInstruction = findViewById(R.id.editInstruction);
        buttonSpeak = findViewById(R.id.buttonSpeak);
        buttonStart = findViewById(R.id.buttonStartMission);
        buttonStop = findViewById(R.id.buttonStopMission);
        textLog = findViewById(R.id.textMissionLog);
        scrollLog = findViewById(R.id.scrollMissionLog);

        buttonSpeak.setOnClickListener(v -> startVoiceRecognition());
        buttonStart.setOnClickListener(v -> confirmAndStartMission());
        buttonStop.setOnClickListener(v -> stopMission());
        findViewById(R.id.buttonVlaEmergency).setOnClickListener(v -> emergencyStop());

        // Track drone connection the same way the main screen icon does.
        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyConnection),
                this,
                (oldValue, newValue) ->
                        droneConnected = (newValue != null && newValue));

        setMissionRunningUi(false);
        appendLog("Ready. Speak or type an instruction, then start the mission.");

        if (getGeminiApiKey().isEmpty()) {
            appendLog("WARNING: no Gemini API key found. Add\n"
                    + "<string name=\"GEMINI_API_KEY\">your-key</string>\n"
                    + "to app/src/main/res/values/secrets.xml");
        }
    }

    @Override
    protected void onResume()
    {
        super.onResume();
        CameraStreamManager.getInstance()
                .addAvailableCameraUpdatedListener(cameraListener);
        snapshotter.start();
    }

    @Override
    protected void onPause()
    {
        super.onPause();

        ICameraStreamManager manager = CameraStreamManager.getInstance();
        manager.removeAvailableCameraUpdatedListener(cameraListener);

        SurfaceView surfaceView = findViewById(R.id.vlaVideo);
        manager.removeCameraStreamSurface(surfaceView.getHolder().getSurface());
    }

    @Override
    protected void onDestroy()
    {
        super.onDestroy();

        // Leaving the screen stops the mission - the drone hovers
        // and stick control returns to the remote controller.
        if (missionRunner != null)
            missionRunner.requestStop();

        snapshotter.stop();
        KeyManager.getInstance().cancelListen(this);
    }

    // ==================== Voice recognition ====================

    private void startVoiceRecognition()
    {
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(
                RecognizerIntent.EXTRA_LANGUAGE_MODEL,
                RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(
                RecognizerIntent.EXTRA_PROMPT,
                getString(R.string.vla_speak_prompt));

        try {
            speechLauncher.launch(intent);
        }
        catch (ActivityNotFoundException e) {
            Toast.makeText(this, R.string.vla_no_speech_service, Toast.LENGTH_LONG).show();
        }
    }

    // ==================== Mission control ====================

    private String getGeminiApiKey()
    {
        // secrets.xml is not committed, so the resource may not exist -
        // resolve it dynamically instead of referencing R.string directly.
        int id = getResources().getIdentifier(
                "GEMINI_API_KEY", "string", getPackageName());

        return (id == 0) ? "" : getString(id).trim();
    }

    private void confirmAndStartMission()
    {
        final String instruction = editInstruction.getText().toString().trim();

        if (instruction.isEmpty()) {
            Toast.makeText(this, R.string.vla_no_instruction, Toast.LENGTH_SHORT).show();
            return;
        }

        final String apiKey = getGeminiApiKey();
        if (apiKey.isEmpty()) {
            Toast.makeText(this, R.string.vla_no_api_key, Toast.LENGTH_LONG).show();
            return;
        }

        // Listener state first; cached value as fallback in case the
        // listener has not delivered yet.
        Boolean cachedConnected = KeyManager.getInstance().getValue(
                KeyTools.createKey(FlightControllerKey.KeyConnection), Boolean.FALSE);
        if (!droneConnected && !Boolean.TRUE.equals(cachedConnected)) {
            Toast.makeText(this, R.string.vla_drone_not_connected, Toast.LENGTH_LONG).show();
            return;
        }

        if (missionRunner != null && missionRunner.isRunning())
            return;

        new AlertDialog.Builder(this)
                .setTitle(R.string.vla_confirm_title)
                .setMessage(getString(R.string.vla_confirm_message, instruction))
                .setPositiveButton(R.string.vla_confirm_start,
                        (dialog, which) -> startMission(instruction, apiKey))
                .setNegativeButton(android.R.string.cancel, null)
                .show();
    }

    private void startMission(String instruction, String apiKey)
    {
        emergencyLanding = false;

        GeminiVlaClient client = new GeminiVlaClient(
                apiKey, getString(R.string.gemini_model_name));

        missionRunner = new VlaMissionRunner(client, snapshotter, this);

        if (missionRunner.start(instruction))
            setMissionRunningUi(true);
    }

    private void stopMission()
    {
        if (missionRunner != null && missionRunner.isRunning()) {
            missionRunner.requestStop();
            appendLog("Stop requested - drone will hover.");
        }
    }

    // ==================== Emergency stop ====================

    /**
     * Abort everything and land: stop the mission without letting it
     * touch the sticks again, give stick control back to the remote
     * controller, and start an automatic landing (auto-confirming the
     * landing if the aircraft asks for it).
     */
    private void emergencyStop()
    {
        appendLog(">>> EMERGENCY STOP - landing <<<");
        emergencyLanding = true;

        // 1. Stop the mission loop; it will not send stick commands anymore.
        if (missionRunner != null)
            missionRunner.notifyEmergency();

        // 2. Return stick control to the remote controller.
        VirtualStickManager.getInstance().setVirtualStickAdvancedModeEnabled(false);
        VirtualStickManager.getInstance().disableVirtualStick(
                new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onSuccess() {
                        Log.i(TAG, "Emergency: control returned to RC.");
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError error) {
                        Log.e(TAG, "Emergency: failed returning control: "
                                + error);
                    }
                });

        // 3. Land, and answer the landing confirmation if requested.
        KeyManager.getInstance().listen(
                KeyTools.createKey(FlightControllerKey.KeyIsLandingConfirmationNeeded),
                this,
                (oldValue, newValue) -> {
                    if (emergencyLanding && newValue != null && newValue)
                        confirmLanding();
                });

        KeyManager.getInstance().performAction(
                KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding),
                new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                    @Override
                    public void onSuccess(EmptyMsg emptyMsg) {
                        runOnUiThread(() -> {
                            appendLog("Auto landing started.");
                            Toast.makeText(VlaActivity.this,
                                    R.string.vla_landing_started, Toast.LENGTH_SHORT).show();
                        });
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError error) {
                        runOnUiThread(() -> appendLog(
                                "Landing command failed: " + error
                                        + " (drone may already be on the ground)"));
                    }
                });
    }

    private void confirmLanding()
    {
        KeyManager.getInstance().performAction(
                KeyTools.createKey(FlightControllerKey.KeyConfirmLanding),
                new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                    @Override
                    public void onSuccess(EmptyMsg emptyMsg) {
                        Log.i(TAG, "Emergency landing confirmed.");
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError error) {
                        Log.w(TAG, "Landing confirmation failed: " + error);
                    }
                });
    }

    // ==================== Mission runner callbacks ====================

    @Override
    public void onMissionStarted(String instruction)
    {
        runOnUiThread(() -> appendLog("Mission started: \"" + instruction + "\""));
    }

    @Override
    public void onDecision(int step, VlaDecision decision)
    {
        runOnUiThread(() -> {
            StringBuilder entry = new StringBuilder(String.format(Locale.ENGLISH,
                    "Step %d\nGemini: %s\nActions: %s",
                    step, decision.reasoning, decision.actionsLine()));

            if (decision.plan != null)
                entry.append("\nPlan: ").append(decision.plan);

            appendLog(entry.toString());
        });
    }

    @Override
    public void onInfo(String message)
    {
        runOnUiThread(() -> appendLog(message));
    }

    @Override
    public void onMissionFinished(String summary, @Nullable String error)
    {
        runOnUiThread(() -> {
            if (error != null)
                appendLog("MISSION ABORTED: " + error);
            appendLog(summary);
            setMissionRunningUi(false);
        });
    }

    // ==================== UI helpers ====================

    private void setMissionRunningUi(boolean running)
    {
        buttonStart.setEnabled(!running);
        buttonStop.setEnabled(running);
        buttonSpeak.setEnabled(!running);
        editInstruction.setEnabled(!running);
    }

    private void appendLog(String line)
    {
        textLog.append(line + "\n\n");
        scrollLog.post(() -> scrollLog.fullScroll(View.FOCUS_DOWN));
    }
}
