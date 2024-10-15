package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@TeleOp(name = "TestingPID")
public class TestingPID extends OpMode {
private PIDController controller;



    public static double f = 0;

    public static int target =0;
    public static double p = 0, i = 0, d = 0;
    public final double ticks_in_degrees = 145.1/180; //num of ticks per rotation we need to find this out
    DcMotorEx backLeftSlidePID, backRightSlide, frontLeftSlide, frontRightSlide;


    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backLeftSlidePID = hardwareMap.get(DcMotorEx.class,"leftBackS");
        backRightSlide = hardwareMap.get(DcMotorEx.class,"rightBackS");
        frontLeftSlide = hardwareMap.get(DcMotorEx.class,"leftFrontS");
        frontRightSlide = hardwareMap.get(DcMotorEx.class,"rightFrontS");

        frontLeftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos = backLeftSlidePID.getCurrentPosition();



        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

        double power = pid + ff;

        telemetry.addData("arm pos", armPos);
        telemetry.addData("target", target);

        backLeftSlidePID.setPower(power);
        backRightSlide.setPower(power);
        frontLeftSlide.setPower(power);
        frontRightSlide.setPower(power);
        telemetry.update();
    }
}
