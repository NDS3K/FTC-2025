package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ColorSen;
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
        ColorSen colorSen = new ColorSen(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {

            Actions.runBlocking(new ParallelAction(
                    drivetrain.drive(gamepad1.left_stick_x*.65, -gamepad1.left_stick_y*.65, gamepad1.right_stick_x*0.7),
                    intake.intakeControl(gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.left_bumper, gamepad1.right_bumper,gamepad1.triangle, gamepad1.cross),
                    scoring.scoringControl(gamepad1.circle, gamepad1.square, gamepad2.x, gamepad2.y, gamepad1.left_trigger>0, gamepad1.right_trigger>0)
            ));
            telemetry.addData("linear:", scoring.linear.getCurrentPosition());
            telemetry.update();
        }
    }
}
