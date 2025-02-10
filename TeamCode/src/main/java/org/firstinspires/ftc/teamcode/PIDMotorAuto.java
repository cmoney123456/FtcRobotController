
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto PID Motor TeleOp with Joystick & Buttons", group="Linear OpMode")
public class PIDMotorAuto extends LinearOpMode {
    DcMotor testMotor;
    DcMotor slideMotor;
    Servo wrist;
    CRServo intakeLeft;
    CRServo intakeRight;

    // PID parameters
    double integral = 0;
    double lastError = 0;
    double Kp = 0.001; // Proportional gain
    double Ki = 0.00001; // Integral gain
    double Kd = 0.00001; // Derivative gain
    double targetPosition = 0; // Target position
    double slidetarPos = 0;
    double slideLastError = 0;
    double slideKp = 0.06;
    double slideKi = 0.0006;
    double slideKd = 0.0006;
    double slideIntegral = 0;
    double wristPos = 0.75;
    double intakePos;

    // Tuning and timing parameters
    final double POSITION_TOLERANCE = 10; // Acceptable error (tolerance) for reaching the target

    ElapsedTime PIDTimer = new ElapsedTime();

    // User-adjustable controls
    final double POSITION_INCREMENT = 50;  // The amount to increase or decrease the position per button press

    @Override
    public void runOpMode() {
        // Initialize motor
        testMotor = hardwareMap.dcMotor.get("liftMotor");
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Ensure motor stops moving when power is set to 0
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder to zero
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Use encoder feedback for PID control
        slideMotor = hardwareMap.dcMotor.get("armMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Ensure motor stops moving when power is set to 0
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the encoder to zero
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist = hardwareMap.servo.get("wrist");
        intakeLeft = hardwareMap.crservo.get("intakeLeft");
        intakeRight = hardwareMap.crservo.get("intakeRight");


        waitForStart();

        // Continuous loop to control the motor
        while (opModeIsActive()) {
            double motorPower = pidControl(targetPosition);
            double slidePower = slidepidControl(slidetarPos);


            targetPosition=-2100;
            if (testMotor.getCurrentPosition()<-1600){
                slidetarPos = -1850;
                if (slideMotor.getCurrentPosition()<-1800){
                    wristPos = 0.3;
                    sleep(500);
                    intakePos = -.5;
                }
            }


            motorPower = Math.max(-1, Math.min(1, motorPower));
            slidePower = Math.max(-1, Math.min(1,slidePower));

            // Set the motor power
            if (testMotor.getCurrentPosition()<-1600){
                testMotor.setPower(motorPower * 0.15);
            }
            else {
                testMotor.setPower(motorPower * 0.5);
            }
            if (slideMotor.getCurrentPosition()<-1500){
                slideMotor.setPower(slidePower * 0.1);
            }
            else {
                slideMotor.setPower(slidePower * 0.6);
            }

            wrist.setPosition(wristPos);
            intakeLeft.setPower(-intakePos);
            intakeRight.setPower(intakePos);

            telemetry.addData("Motor Position", testMotor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Slide Motor Position", slideMotor.getCurrentPosition());
            telemetry.addData("Slide Target Position", slidetarPos);
            telemetry.addData("Slide Motor Power", slidePower);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.addData("slide Kp", slideKp);
            telemetry.addData("slide Ki", slideKi);
            telemetry.addData("slide Kd", slideKd);
            telemetry.update();

            // Sleep to prevent busy waiting and allow time for feedback
            sleep(50);
        }
    }

    // PID control function
    private double pidControl(double targetPosition) {
        double currentPosition = testMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;
        double deltaTime = PIDTimer.time(); // Time since last call

        if (deltaTime > 0) {
            // Calculate integral and derivative
            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            // PID output
            double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // Update last error and reset timer for next calculation
            lastError = error;
            PIDTimer.reset();


            return output;
        } else {
            // If deltaTime is 0, return 0 to prevent division by zero (for the first loop iteration)
            return 0;
        }

    }
    // PID control function
    private double slidepidControl(double slidetarPos) {
        double slideCurPos = slideMotor.getCurrentPosition();
        double deltaTime = PIDTimer.time(); // Time since last call
        double slideError = slidetarPos - slideCurPos;

        if (deltaTime > 0) {
            // Calculate integral and derivative
            slideIntegral += slideError * deltaTime;
            double slideDerivative = (slideError - slideLastError) / deltaTime;

            // PID output
            double slideOutput = (slideKp * slideError) + (slideKi * slideIntegral) + (slideKd * slideDerivative);

            // Update last error and reset timer for next calculation
            slideLastError = slideError;
            PIDTimer.reset();


            return slideOutput;
        } else {
            // If deltaTime is 0, return 0 to prevent division by zero (for the first loop iteration)
            return 0;
        }
    }
    // This method can be added to check if the motor is near the target and stop PID control when the target is reached
    private boolean isAtTargetPosition(double targetPosition) {
        double currentPosition = testMotor.getCurrentPosition();
        return Math.abs(targetPosition - currentPosition) <= POSITION_TOLERANCE;
    }
}
