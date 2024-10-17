
        package org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;

import static org.firstinspires.ftc.teamcode.Autonomous.AutonomousTesting.AutoStates.*;

        import org.firstinspires.ftc.teamcode.Autonomous.PinpointDrive;

public class ActionCreator {
    ParallelAction preload, intakeSample1, outtakeSample1, intakeSample2, outtakeSample2;

    public ActionCreator(){

    }

    public SequentialAction CreatePath(AutoStates location, PinpointDrive drive){
        switch (location){
            case bucket:
                preload = new ParallelAction(drive.actionBuilder(drive.pose)
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(30,-10), Math.toRadians(0)).build()
                );

                intakeSample1 = new ParallelAction(drive.actionBuilder(new Pose2d(30,10,Math.toRadians(0)))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(48,10, Math.toRadians(90)), Math.toRadians(40)).build()
                );

                outtakeSample1 = new ParallelAction(drive.actionBuilder(new Pose2d(48,10,Math.toRadians(90)))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(16,32, Math.toRadians(135)), Math.toRadians(160)).build()
                );

                intakeSample2 = new ParallelAction(drive.actionBuilder(new Pose2d(16,32, Math.toRadians(135)))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(45,25, Math.toRadians(90)), Math.toRadians(60)).build()
                );

                outtakeSample2 = new ParallelAction(drive.actionBuilder(new Pose2d(45,25,Math.toRadians(90)))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(16,32, Math.toRadians(135)), Math.toRadians(160)).build()
                );
                break;
            case specimen:
                break;
        }
        return new SequentialAction(preload, intakeSample1, outtakeSample1, intakeSample2, outtakeSample2, intakeSample2, outtakeSample2);
    }
}