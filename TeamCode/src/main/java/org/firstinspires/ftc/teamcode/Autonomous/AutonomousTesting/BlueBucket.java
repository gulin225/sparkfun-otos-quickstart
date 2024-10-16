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
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Autonomous.MecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomous.PinpointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting.AprilTagDrive.*;

import java.util.ArrayList;

//-72 y = red side
//x is pos on red side
@Config
@Autonomous(name = "BlueBucket", group = "Autonomous")
public class BlueBucket extends LinearOpMode {

    AutoStates autoStates = preload;
    IMU imu;
    PinpointDrive drive;
    AprilTagDrive aprilTagDrive; //14.25,41
    final Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
    TelemetryPacket tel = new TelemetryPacket();
    SequentialAction sampleAction;
    SequentialAction preloadAction;
    Limelight limelight;
    boolean running;
    final Vector2d targetAprilTag = new Vector2d(47.5,71.5);
    final double cameraPlacementX = 7.5;
    final double cameraPlacementY = 0;
    final double cameraAngle = Math.atan(cameraPlacementY/cameraPlacementX);
    final double botCenterHypotenuse = Math.sqrt(Math.pow(cameraPlacementX,2) + Math.pow(cameraPlacementY,2));
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new PinpointDrive(hardwareMap, startPose);
        while(!isStopRequested() && !opModeIsActive()) {
        }

        waitForStart();

        if (isStopRequested()) return;

        autoStates = preload;
        if (autoStates == idle) {}
        else {
            preloadAction = createPreloadAction();
            running = preloadAction.run(tel);
        }

        while (!isStopRequested() && opModeIsActive()) {
            switch (autoStates){
                case preload:
                    running = preloadAction.run(tel);
                    if (!running) {
                        autoStates = idle;
                    }
                    break;
                case samples:
                    running = sampleAction.run(tel);
                    if (!running) {

                        autoStates = idle;
                    }
                    break;
                case idle:
                    break;
            }

            telemetry.addData("State", autoStates);
            telemetry.addData("RR Location", "x: " + drive.pose.position.x + " Y: " + drive.pose.position.y);
            telemetry.update();

        }
    }


    public SequentialAction createPreloadAction(){
        SequentialAction preloadAction = new SequentialAction(drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-17,3), Math.toRadians(200))
                .waitSeconds(2).build(),
                new SequentialAction(drive.actionBuilder(new Pose2d(-17,3,Math.toRadians(0)))
                .setTangent(Math.toRadians(200))
                .splineToConstantHeading(new Vector2d(-29,-34), Math.toRadians(260)).build()

        ));

        return  preloadAction;
    }

    public SequentialAction createSampleAction(){
        //Each action goes to the sample, then deposits it at the human player
        ParallelAction sample1Action = new ParallelAction(drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(-29,-34), Math.toRadians(260))
                .waitSeconds(1)
                //.splineToConstantHeading(new Vector2d(57,-19), Math.toRadians(340))
                //.waitSeconds(1) -29.4 -36.5
                .build()
                //There should be sleep actions with subsytem movements inside each parallel action
        );

        ParallelAction sample2Action = new ParallelAction(drive.actionBuilder(drive.pose)
                .lineToY(-33)
                .splineToLinearHeading(new Pose2d(57,62,Math.toRadians(-4.5)), Math.toRadians(-90))
                .waitSeconds(1).build()
        );

        ParallelAction sample3Action = new ParallelAction(drive.actionBuilder(drive.pose)
                .build()
        );

        SequentialAction specimenSequence = new SequentialAction(sample1Action);
        return specimenSequence;
    }


}

























