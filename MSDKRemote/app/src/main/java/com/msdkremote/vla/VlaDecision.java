package com.msdkremote.vla;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.List;

/**
 * One decision step returned by the VLA model:
 * a short reasoning text, the model's updated mission plan
 * (carried into the next step as its long-term memory), and
 * a sequence of one or more actions to execute back-to-back
 * before the model is consulted again.
 */
public class VlaDecision
{
    public final String reasoning;
    @Nullable
    public final String plan;
    public final List<VlaAction> actions;

    public VlaDecision(@NonNull String reasoning, @Nullable String plan,
                       @NonNull List<VlaAction> actions)
    {
        if (actions.isEmpty())
            throw new IllegalArgumentException("A decision needs at least one action");

        this.reasoning = reasoning;
        this.plan = plan;
        this.actions = actions;
    }

    /** All actions of this step on one line, e.g. for logs and history. */
    @NonNull
    public String actionsLine()
    {
        StringBuilder line = new StringBuilder();
        for (VlaAction action : actions) {
            if (line.length() > 0)
                line.append(" -> ");
            line.append(action);
        }
        return line.toString();
    }
}
