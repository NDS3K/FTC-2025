package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake{
    ElapsedTime runtime = new ElapsedTime();
    public Servo claw;
    public CRServo wrist;
    public CRServo roller_left;
    public CRServo roller_right;
    public Intake(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "cl");
        wrist = hardwareMap.get(CRServo.class, "wr");
        roller_left = hardwareMap.get(CRServo.class, "rl");
        roller_right = hardwareMap.get(CRServo.class, "rr");

        //TODO: Replace when wrist using Servo Position
        // wrist = hardwareMap.get(Servo.class, "wr");
        // Don't forget to change Servo Firmwares
    }
    // Close Func for both auto and control claw
    public Action close() {
        return telemetryPacket -> {
            claw.setPosition(0.95);
            return false;
        };
    }
    // Open func for both auto and control claw
    public Action open(){
        return telemetrypacket -> {
            claw.setPosition(0.4);
            return false;
        };
    }
    // Auto func for cr wrist
    public Action wristAuto(int time,double power){
        return telemetryPacket -> {
            wrist.setPower(power);
            return runtime.milliseconds() < time;
        };
    }

    //TODO: Replace when wrist using Servo Position
    // Auto func for wrist set position
    /* public Action wristAuto(double position){
        return telemetryPacket -> {
            wrist.setPosition(position);
            return false;
        };
    */

    // Control func for wrist
    public Action wristControl(boolean x, boolean y, double wrist_power){
        return telemetryPacket -> {
                if(x) wrist.setPower(1);
                else if(y) wrist.setPower(-1);
                else wrist.setPower(0);
                return false;
        };
    }
    public Action rollerControl(boolean x, boolean y, double roller_power){
        return telemetryPacket -> {
            if(x){
                roller_left.setPower(roller_power);
                roller_right.setPower(-roller_power);
            }
            else if(y){
                roller_right.setPower(roller_power);
                roller_left.setPower(-roller_power);
            }
            else{
                roller_left.setPower(0);
                roller_right.setPower(0);
            }
            return false;
        };

    }

    //TODO: Replace when wrist using Servo Position
    /* public Action wristControl(boolean x, boolean y){
        return telemetryPacket -> {
                if(x) wrist.setPosition(0.6);
                else if(y) wrist.setPosition(0);
                return false;
        };
       }
    */
    // Intake control
    public Action intakeControl(boolean x,boolean a, boolean b, boolean wrist_forward, boolean wrist_backward, double wrist_power){
        return telemetryPacket -> {
            if(x) Actions.runBlocking(open());
            else Actions.runBlocking(close());
            Actions.runBlocking(new ParallelAction(
                    wristControl(wrist_forward, wrist_backward, wrist_power),
                    rollerControl(a,b,1))
            );
            return false;
        };
    }
}