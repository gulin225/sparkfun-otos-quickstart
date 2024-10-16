package org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting;

import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting.AutoStates.idle;
import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting.AutoStates.preload;
import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting.AutoStates.samples;

import androidx.annotation.NonNull;

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
@Autonomous(name = "BlueBucket", group = "Autonomous")
public class BlueBucket extends LinearOpMode {

    AutoStates autoStates = preload;
    PinpointDrive drive;
    final Pose2d startPose = new Pose2d(7,-7, Math.toRadians(0));
    TelemetryPacket tel = new TelemetryPacket();
    SequentialAction path;
    boolean running;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new PinpointDrive(hardwareMap, startPose, telemetry);

        while(!isStopRequested() && !opModeIsActive())

        waitForStart();

        if (isStopRequested()) return;

        autoStates = preload;
        if (autoStates == idle) {}
        else {
            path = actionCreator();
            running = path.run(tel);
        }

        while (!isStopRequested() && opModeIsActive()) {
            switch (autoStates){
                case preload:
                    running = path.run(tel);
                    if (!running) autoStates = idle;
                    break;
                case idle:
                    break;
            }
        }
    }

    public SequentialAction actionCreator(){
        ParallelAction preload = new ParallelAction(drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(30,-10), Math.toRadians(0)).build()
        );

        ParallelAction intakeSample1 = new ParallelAction(drive.actionBuilder(new Pose2d(30,-10,Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,10, Math.toRadians(90)), Math.toRadians(40)).build()
        );

        ParallelAction outtakeSample1 = new ParallelAction(drive.actionBuilder(new Pose2d(48,10,Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(16,32, Math.toRadians(135)), Math.toRadians(160)).build()
        );

        ParallelAction intakeSample2 = new ParallelAction(drive.actionBuilder(new Pose2d(16,32, Math.toRadians(135)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(45,25, Math.toRadians(90)), Math.toRadians(60)).build()
        );

        ParallelAction outtakeSample2 = new ParallelAction(drive.actionBuilder(new Pose2d(45,25,Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(16,32, Math.toRadians(135)), Math.toRadians(160)).build()
        );

        return new SequentialAction(preload, intakeSample1, outtakeSample1, intakeSample2, outtakeSample2, intakeSample2, outtakeSample2);
    }
}

























