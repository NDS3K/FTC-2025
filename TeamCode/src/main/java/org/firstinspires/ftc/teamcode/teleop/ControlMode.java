package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.algorithms.PID;
import org.firstinspires.ftc.teamcode.mechanism.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Scoring;

@TeleOp(name = "ControlMode", group = "Linear OpMode")
public class ControlMode extends LinearOpMode {
    public void runOpMode(){
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Scoring scoring = new Scoring(hardwareMap);
        PID pid = new PID();
        waitForStart();
        while(opModeIsActive()) {
            Actions.runBlocking(new ParallelAction(
                    drivetrain.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x*0.7),
                    intake.intakeControl(gamepad1.x,gamepad2.x,gamepad2.y, gamepad1.dpad_up, gamepad1.dpad_down, 0.8),
                    scoring.scoringControl(gamepad1.a, gamepad1.b, gamepad1.right_stick_y*0.7, 1))
            );
            telemetry.addData("elbow:", scoring.elbow.getCurrentPosition());
            telemetry.addData("linear:", scoring.linear.getCurrentPosition());
            telemetry.update();
        }
    }
}