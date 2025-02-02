package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Scoring;

public class AutoMode extends LinearOpMode {
    Trajectory trajectory = new Trajectory();
    MecanumDrive drive = new MecanumDrive(hardwareMap, trajectory.initialPose);
    Scoring scoring = new Scoring(hardwareMap);
    Intake intake = new Intake(hardwareMap);
    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            trajectory.path1.build(),
                            trajectory.path2.build(),
                            trajectory.path3.build(),
                            trajectory.path4.build(),
                            trajectory.path5.build(),
                            trajectory.path6.build(),
                            trajectory.path7.build()
                    )
            );
        }
    }
}
