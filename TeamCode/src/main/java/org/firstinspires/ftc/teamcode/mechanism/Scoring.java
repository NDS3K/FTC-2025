package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.algorithms.PID;


public class Scoring {
    public DcMotorEx linear, elbow;
    PID pid = new PID();
    public Scoring(HardwareMap hardwareMap){
        linear = hardwareMap.get(DcMotorEx.class, "ln");
        elbow = hardwareMap.get(DcMotorEx.class, "elb");
        linear.setDirection(DcMotorEx.Direction.REVERSE);
        elbow.setDirection(DcMotorEx.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public Action linearControl(boolean x, boolean y,double power){
        return telemetryPacket ->  {
            if(x) elbow.setPower(power);
            else if(y) elbow.setPower(-power);
            else elbow.setPower(0);
            return false;
        };
    }
    public Action elbowControl(double power) {
        return telemetryPacket -> {
            linear.setPower(power);
            return false;
        };
    }
    public Action scoringControl(boolean x, boolean y,double elbow_power, double linear_power){
        return telemetryPacket ->{
            Actions.runBlocking(new ParallelAction(
                    elbowControl(elbow_power),
                    linearControl(x,y,linear_power)
            ));
            return false;
        };
    }

    public Action elbowAuto(double target, double power){
        return telemetryPacket -> {
            double output = pid.PIDControlDistance(target, elbow.getCurrentPosition());
            elbow.setPower(output*power);
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
