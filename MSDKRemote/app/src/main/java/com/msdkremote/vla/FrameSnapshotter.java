package com.msdkremote.vla;

import android.graphics.Bitmap;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.io.ByteArrayOutputStream;
import java.nio.ByteBuffer;

import dji.sdk.keyvalue.value.common.ComponentIndexType;
import dji.v5.manager.datacenter.camera.CameraStreamManager;
import dji.v5.manager.interfaces.ICameraStreamManager;

/**
 * Captures single frames from the drone main camera as JPEG.
 *
 * A frame listener is registered on the camera stream, but frames are
 * only copied when a snapshot was requested, so the idle overhead is
 * minimal. Use start() / stop() around the lifetime of the consumer.
 */
public class FrameSnapshotter implements ICameraStreamManager.CameraFrameListener
{
    private static final String TAG = FrameSnapshotter.class.getSimpleName();

    private final Object lock = new Object();

    private boolean started = false;

    // Snapshot request state, guarded by lock.
    private boolean requestPending = false;
    private byte[] rgba = null;
    private int width = 0;
    private int height = 0;

    public void start()
    {
        synchronized (lock) {
            if (started) return;
            started = true;
        }

        CameraStreamManager.getInstance().addFrameListener(
                ComponentIndexType.LEFT_OR_MAIN,
                ICameraStreamManager.FrameFormat.RGBA_8888,
                this
        );
    }

    public void stop()
    {
        synchronized (lock) {
            if (!started) return;
            started = false;
            requestPending = false;
            rgba = null;
            lock.notifyAll();
        }

        CameraStreamManager.getInstance().removeFrameListener(this);
    }

    @Override
    public void onFrame(@NonNull byte[] frameData, int offset, int length,
                        int width, int height, ICameraStreamManager.FrameFormat format)
    {
        synchronized (lock) {
            if (!requestPending)
                return;

            if (length < width * height * 4) {
                Log.w(TAG, "Unexpected RGBA frame size: " + length
                        + " for " + width + "x" + height);
                return;
            }

            this.rgba = new byte[width * height * 4];
            System.arraycopy(frameData, offset, this.rgba, 0, this.rgba.length);
            this.width = width;
            this.height = height;
            this.requestPending = false;
            lock.notifyAll();
        }
    }

    /**
     * Capture the next frame from the camera stream and return it as JPEG.
     *
     * @param maxWidth  downscale the image to at most this width (keeps ratio).
     * @param quality   JPEG quality, 1-100.
     * @param timeoutMs how long to wait for a frame.
     * @return JPEG bytes, or null if no frame arrived within the timeout.
     */
    @Nullable
    public byte[] captureJpeg(int maxWidth, int quality, long timeoutMs)
            throws InterruptedException
    {
        final byte[] frame;
        final int frameWidth;
        final int frameHeight;

        synchronized (lock) {
            if (!started)
                return null;

            rgba = null;
            requestPending = true;

            long deadline = System.currentTimeMillis() + timeoutMs;
            while (rgba == null) {
                long remaining = deadline - System.currentTimeMillis();
                if (remaining <= 0 || !started) {
                    requestPending = false;
                    return null;
                }
                lock.wait(remaining);
            }

            frame = rgba;
            frameWidth = width;
            frameHeight = height;
            rgba = null;
        }

        return toJpeg(frame, frameWidth, frameHeight, maxWidth, quality);
    }

    @Nullable
    private static byte[] toJpeg(byte[] rgba, int width, int height,
                                 int maxWidth, int quality)
    {
        try {
            Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            bitmap.copyPixelsFromBuffer(ByteBuffer.wrap(rgba));

            if (width > maxWidth) {
                int scaledHeight = (int) ((long) height * maxWidth / width);
                Bitmap scaled = Bitmap.createScaledBitmap(bitmap, maxWidth, scaledHeight, true);
                bitmap.recycle();
                bitmap = scaled;
            }

            ByteArrayOutputStream stream = new ByteArrayOutputStream();
            bitmap.compress(Bitmap.CompressFormat.JPEG, quality, stream);
            bitmap.recycle();

            return stream.toByteArray();
        }
        catch (Exception e) {
            Log.e(TAG, "Failed converting frame to JPEG", e);
            return null;
        }
    }
}
