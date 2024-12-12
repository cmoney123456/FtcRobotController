package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class redLeftZone extends LinearOpMode {

    DcMotor linear;



    //double wristPower = 0.75;
    //double clawPosition = 1;


    public void runOpMode() {
        linear = hardwareMap.get(DcMotorEx .class,"linearMotor");
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
       // Drive.init(hardwareMap,true);
       // imu.initIMU(hardwareMap);

       // double linearPos = Drive.linearPos();

        Pose2d startPos = new Pose2d(-55,-72,Math.toRadians(90));

        drive.setPoseEstimate(startPos);

/*        Trajectory myTrajectory = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-55,-20),Math.toRadians(90))
                .build();
*/
        Trajectory traj0 = drive.trajectoryBuilder(startPos)
                .strafeLeft(8)
                .build();
        Trajectory trajA = drive.trajectoryBuilder(traj0.end())
                .strafeRight(8)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(trajA.end())
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
        Trajectory traj12 = drive.trajectoryBuilder(traj9.end())
                .strafeRight(100,SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        Trajectory traj54 = drive.trajectoryBuilder(traj12.end())
                .back(8)
                        .build();

/*        Trajectory traj122 = drive.trajectoryBuilder(traj12.end())
                .forward(40,
        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();*/
        /*Trajectory traj11 = drive.trajectoryBuilder(traj12.end())
                .splineTo(new Vector2d(40,-72),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();*/
   /*     Trajectory traj43 = drive.trajectoryBuilder(traj11.end())
                .strafeLeft(10,SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();*/


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


        drive.followTrajectory(traj0);
        drive.followTrajectory(trajA);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);
 //       drive.followTrajectory(traj10);
        drive.followTrajectory(traj12);
 //       drive.followTrajectory(traj122);
        drive.followTrajectory(traj54);
 //       drive.followTrajectory(traj43);
//        drive.followTrajectory(traj13);
        //drive.followTrajectory(myTrajectory);




    }

    private void moveLinearSlideToPosition(int targetPosition) {
        linear.setTargetPosition(targetPosition);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(-.05);
    }


}