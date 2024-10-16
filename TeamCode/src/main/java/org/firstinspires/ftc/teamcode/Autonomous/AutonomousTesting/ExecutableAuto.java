package org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting;

import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting.AutoStates.*;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Autonomous.PinpointDrive;

@Config
@Autonomous(name = "Executable", group = "Autonomous")
public class ExecutableAuto extends LinearOpMode {
    AutoStates driveState = followingPath;
    PinpointDrive drive;
    boolean testing = false;
    final Pose2d startPose = new Pose2d(7.9,-8.5, Math.toRadians(0));
    TelemetryPacket tel = new TelemetryPacket();
    SequentialAction path;
    boolean running;
    AutoStates autoLocation = bucket;
    ActionCreator actionCreator;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new PinpointDrive(hardwareMap, startPose, telemetry);
        actionCreator = new ActionCreator();

        while(!isStopRequested() && !opModeIsActive()){
            if (gamepad1.left_bumper) autoLocation = bucket;
            if (gamepad1.right_bumper) autoLocation = specimen;

            telemetry.addData("Current path", autoLocation);
            telemetry.update();
        }

        if (testing) driveState = idle;
        else path = actionCreator.CreatePath(autoLocation, drive);

        while (!isStopRequested() && opModeIsActive()) {
            switch (driveState){
                case followingPath:
                    running = path.run(tel);
                    if (!running) driveState = idle;
                    break;
                case idle:
                    break;
            }
        }
    }

}

























