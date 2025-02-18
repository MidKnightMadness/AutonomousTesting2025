package org.firstinspires.ftc.teamcode.Components;

public class ButtonToggle {
    boolean value;

    public boolean update(boolean stateValue){
        if(stateValue && !value){
            value = true;

            return true;
        }

        value = false;
        return false;
    }
}
