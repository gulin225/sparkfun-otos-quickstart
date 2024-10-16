package org.firstinspires.ftc.teamcode.Autonomous.SubSystemActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class ClawActions {
    private Claw claw;
    public ClawActions(HardwareMap hardwareMap){
        claw = new Claw(hardwareMap);
    }
    public class closeClaw implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                claw.moveClaw(Claw.clawStates.close);
                initialized = true;
            }

            return true;
        }
    }
    public class openClaw implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                claw.moveClaw(Claw.clawStates.open);
                initialized = true;
            }

            return true;
        }
    }

    public class spinOn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                claw.moveClaw(Claw.clawStates.spinOn);
                initialized = true;
            }

            return true;
        }
    }
    public class spinOff implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                claw.moveClaw(Claw.clawStates.spinOff);
                initialized = true;
            }

            return true;
        }
    }

    public Action closeClawAction() {
        return new closeClaw();
    }

    public Action openClawAction(){
        return new openClaw();
    }
    public Action spinOnAction(){
        return new spinOn();
    }
    public Action spinOffAction(){
        return new spinOff();
    }
}
