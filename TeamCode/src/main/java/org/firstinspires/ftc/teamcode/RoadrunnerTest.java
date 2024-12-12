package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RoadrunnerTest extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(-55,-72,Math.toRadians(90));

        drive.setPoseEstimate(startPos);


/*        Trajectory myTrajectory = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-55,-20),Math.toRadians(90))
                .build();
*/
        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .forward(50)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(7)
                        .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(48)
                        .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(48)
                        .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeLeft(10)
                        .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .back(44)
                        .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .forward(44)
                        .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .strafeLeft(7.5)
                        .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .back(44)
                        .build();
        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .forward(10)
                        .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj10.end())
                .strafeRight(6)
                .build();
        Trajectory traj122 = drive.trajectoryBuilder(traj12.end())
                .forward(40,
        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj122.end())
                .splineTo(new Vector2d(-48,-12),Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
        Trajectory traj43 = drive.trajectoryBuilder(traj11.end())
                .strafeLeft(14)
                        .build();


 /*       Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .strafeRight(27)
                        .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .back(2)
                        .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .strafeRight(80)
                        .build();
*/

/*        Trajectory myTrajectory = drive.trajectoryBuilder(startPos)
                        .forward(50)
                        .strafeLeft(5)
                        .back(40)
                        .forward(40)
                        .strafeLeft(8)
                        .back(40)
                        .build();*/



        waitForStart();

        if(isStopRequested()) return;


        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
        drive.followTrajectory(traj10);
        drive.followTrajectory(traj12);
        drive.followTrajectory(traj122);
        drive.followTrajectory(traj11);
        drive.followTrajectory(traj43);
//        drive.followTrajectory(traj13);
        //drive.followTrajectory(myTrajectory);
    }


}