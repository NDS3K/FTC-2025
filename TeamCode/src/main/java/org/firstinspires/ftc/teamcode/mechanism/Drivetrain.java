package org.firstinspires.ftc.teamcode.mechanism;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain
{
    public DcMotorEx topLeft,backLeft,topRight,backRight ;

    public Drivetrain(HardwareMap hardwareMap)
    {
        topLeft = hardwareMap.get(DcMotorEx.class,"tl");
        backLeft = hardwareMap.get(DcMotorEx.class,"bl");
        backRight = hardwareMap.get(DcMotorEx.class,"br");
        topRight = hardwareMap.get(DcMotorEx.class,"tr");

        //TODO: Make sure all motors are on true direction
        topLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        topRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        //TODO: Brake all motors if needed
        // topRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    //Mecanum Control Basic
    public Action drive(double x , double y, double turn){
        return telemetryPacket -> {
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta -Math.PI/4);
            double cos = Math.cos(theta -Math.PI/4);
            double max = Math.max(Math.abs(sin),Math.abs(cos));
            double topLeftPower = power*cos/max + turn;
            double topRightPower = power*sin/max - turn;
            double backLeftPower = power*sin/max + turn;
            double backRightPower = power*cos/max - turn;
            topLeft.setPower(topLeftPower);
            topRight.setPower(topRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            return false;
        };
    }
}
