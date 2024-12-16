package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous (name="Linear Test" , group="Test")
public class LinearAutoTest extends LinearOpMode {
    DcMotor linear;

    public void runOpMode(){
        linear = hardwareMap.get(DcMotorEx.class,"linearMotor");
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double curPos = linearPos();



        waitForStart();
        while (opModeIsActive()) {
            if (curPos > -1500) {

                linear.setPower(-0.1);

            }
            telemetry.addData("LinearPos", curPos);
            //telemetry.addData("targPos", tarPos);
            telemetry.update();
            //sleep(5000);
        }
    }
    public double linearPos(){
        return linear.getCurrentPosition();

    }

}
