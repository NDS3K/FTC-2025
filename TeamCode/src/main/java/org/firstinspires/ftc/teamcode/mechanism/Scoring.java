package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.algorithms.PID;

public class Scoring {
    public DcMotorEx linear;
    public Servo elbow;
    public Servo claw;
    public Servo wrist;
    PID pid = new PID();
    public Scoring(HardwareMap hardwareMap){
        linear = hardwareMap.get(DcMotorEx.class, "ln");
        linear.setDirection(DcMotorEx.Direction.REVERSE);
        linear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbow = hardwareMap.get(Servo.class, "el");
        elbow.setPosition(0.64);
        wrist = hardwareMap.get(Servo.class, "ws");
        wrist.setPosition(0);
        claw = hardwareMap.get(Servo.class, "cl");
        claw.setPosition(1);

    }
    public Action elbowAuto(double position){
        return telemetryPacket -> {
            elbow.setPosition(position);
            return false;
        };
    }
    public Action wsAuto(double position){
        return telemetryPacket -> {
            wrist.setPosition(position);
            return false;
        };
    }
    public Action clawAuto(double position){
        return telemetryPacket -> {
            claw.setPosition(position);
            return false;
        };
    }
    public Action linearAuto(double target, double power) {
        return telemetryPacket -> {
            double output = pid.PIDControlDistance(target, linear.getCurrentPosition());
            linear.setPower(output * power);
            return false;
        };
    }

    public Action linearControl(boolean x, boolean y,double power){
        return telemetryPacket ->  {
            if(x) linear.setPower(power);
            else if(y) linear.setPower(-power);
            else linear.setPower(0);
            return false;
        };
    }
    /*
    public Action elbowControl(double power) {
        return telemetryPacket -> {
            elbow.setPower(power);
            return false;
        };
    }
    */
    public Action scoringControl(boolean x, boolean y, boolean a, boolean b, boolean c, boolean d){
        return telemetryPacket ->{
            if(x) Actions.runBlocking(elbowAuto(.2));
            else if(y) Actions.runBlocking(elbowAuto(0.8));
            if(a) Actions.runBlocking(clawAuto(0.3));
            else if(b) Actions.runBlocking(clawAuto(1));
            if(c) Actions.runBlocking(wsAuto(1));
            else if(d) Actions.runBlocking(wsAuto(0));
            return false;

        };
    }

    /*
    public Action elbowAuto(double target, double power){
        return telemetryPacket -> {
            double output = pid.PIDControlDistance(target, elbow.getCurrentPosition());
            elbow.setPower(output*power);
            return false;
        };
    }
    */

    public Action scoringAutomation(double linear_target,double linear_power, int elbow_position, double wrist_position){
        return telemetryPacket -> {
            Actions.runBlocking(new SequentialAction(
                    linearAuto(linear_target,linear_power),
                    elbowAuto(elbow_position),
                    wsAuto(wrist_position),
                    clawAuto(0.3)
            ));
            return false;
        };
    }
    //TODO: Replace when using PID control Scoring mechanism
    /*  public Action PIDControlElbow(boolean x, boolean y,double target,double power){
            return telemetryPacket -> {
                double output1 = pid.PIDControlDistance(target, elbow.getCurrentPosition());
                double output2 = pid.PIDControlDistance(0, elbow.getCurrentPosition());
                if(x)elbow.setPower(output1);
                else if(y)elbow.setPower(output2);
                return Math.abs(elbow.getCurrentPosition() - target) < 2;
            };
        }
        public Action PIDControlLinear(boolean x, boolean y,double target,double power){
            return telemetryPacket -> {
                double output1 = pid.PIDControlDistance(target, linear.getCurrentPosition());
                double output2 = pid.PIDControlDistance(0, linear.getCurrentPosition());
                if(x)linear.setPower(output1*power);
                else if(y)linear.setPower(output2*power);
                return Math.abs(linear.getCurrentPosition() - target) < 2;
            };
        }
        public Action PIDControlScoring(boolean x_el, boolean y_el,double elbow_target,double elbow_power,boolean a_ln, boolean b_ln,double linear_target, double linear_power){
            return telemetryPacket -> {
                Actions.runBlocking(new ParallelAction(
                    PIDControlElbow(x_el,y_el,elbow_target,elbow_power),
                    PIDControlLinear(a_ln,b_ln,linear_target,linear_power)
                ));
                return false;
            };
        }
    */
}
