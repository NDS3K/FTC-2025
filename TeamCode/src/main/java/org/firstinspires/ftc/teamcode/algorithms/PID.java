package org.firstinspires.ftc.teamcode.algorithms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    ElapsedTime runtime = new ElapsedTime();
    double integralSum;
    double lastError = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    public PID(){}

    public double PIDControlDistance(double reference, double state){
        double error = reference - state;
        integralSum = error * (runtime.seconds());
        double derivative = (error - lastError)/runtime.seconds();
        lastError = error;
        runtime.reset();
        return (error*Kp)+(integralSum*Ki)+(derivative*Kd);
    }
    public double PIDControlAngle(double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum = error * (runtime.seconds());
        double derivative = (error - lastError) / runtime.seconds();
        lastError = error;
        runtime.reset();
        return  (error * Kp) + (integralSum * Ki) + (derivative * Kd);
    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

}