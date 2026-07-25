package com.msdkremote.vla;

import androidx.annotation.NonNull;

/**
 * One decision step returned by the VLA model:
 * a short reasoning text and the action to execute.
 */
public class VlaDecision
{
    public final String reasoning;
    public final VlaAction action;

    public VlaDecision(@NonNull String reasoning, @NonNull VlaAction action)
    {
        this.reasoning = reasoning;
        this.action = action;
    }
}
