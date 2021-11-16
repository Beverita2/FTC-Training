package org.firstinspires.ftc.teamcode.core.thread.types.impl;

import org.firstinspires.ftc.teamcode.core.thread.types.api.RunListenerOnceEvent;

import java.util.function.Supplier;

public class RunWhenOutputChangedOnceEvent extends RunListenerOnceEvent {
    private final Supplier<Object> output;
    private final int hashCode;

    /**
     * @param listener The listener to be run.
     */
    public RunWhenOutputChangedOnceEvent(Runnable listener, Supplier<Object> output) {
        super(listener);
        this.output = output;
        hashCode = output.get().hashCode();
    }

    @Override
    public boolean shouldRun() {
        return output.get().hashCode() != hashCode;
    }
}
