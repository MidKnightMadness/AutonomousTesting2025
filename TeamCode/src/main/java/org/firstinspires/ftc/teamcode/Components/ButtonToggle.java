package org.firstinspires.ftc.teamcode.Components;

public class ButtonToggle {
    boolean lastState;
    boolean value;

    public boolean update(boolean currentState) {
        // if lastState is false
        if (currentState && !lastState) {
            lastState = true;
            value = true;
            return true;
        }

        lastState = currentState;
        value = false;
        return false;
    }

    public static ButtonToggle n() {
        return new ButtonToggle();
    }

    public boolean getValue() {
        return value;
    }

}
