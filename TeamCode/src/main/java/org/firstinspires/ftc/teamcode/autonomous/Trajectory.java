package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Trajectory {
    TrajectoryActionBuilder path1,path2,path3,path4,path5,path6,path7;
    MecanumDrive drive;
    Pose2d initialPose = new Pose2d(-11.5,60,Math.toRadians(0));
    public Trajectory(){
        path1 = drive.actionBuilder(initialPose)
                .lineToY(38);
        path2 = path1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-20,38))
                .splineToConstantHeading(new Vector2d(-35,25),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-46,12),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(50)
                .lineToY(12)
                .strafeTo(new Vector2d(-56,12))
                .setTangent(Math.toRadians(270))
                .lineToY(50)
                .lineToY(12)
                .strafeTo(new Vector2d(-61,12))
                .setTangent(Math.toRadians(270))
                .lineToY(50)
                .strafeTo(new Vector2d(-35,50));
        path3 = path2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-10,40));
        path4 = path3.endTrajectory().fresh()
                .strafeTo(new Vector2d(-35,50));
        path5 = path4.endTrajectory().fresh()
                .strafeTo(new Vector2d(-8,40));
        path6 = path5.endTrajectory().fresh()
                .strafeTo(new Vector2d(-35,50));
        path7 = path6.endTrajectory().fresh()
                .strafeTo(new Vector2d(-6,40));
    }

}
