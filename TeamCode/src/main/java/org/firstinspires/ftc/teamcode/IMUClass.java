package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * {@link IMUClass} shows how to use the new universal {@link IMU} interface. This
 * interface may be used with the BNO055 IMU or the BHI260 IMU. It assumes that an IMU is configured
 * on the robot with the name "imu".
 * <p>
 * The sample will display the current Yaw, Pitch and Roll of the robot.<br>
 * With the correct orientation parameters selected, pitch/roll/yaw should act as follows:
 * <p>
 *   Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X) <br>
 *   Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y) <br>
 *   Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z) <br>
 * <p>
 * The yaw can be reset (to zero) by pressing the Y button on the gamepad (Triangle on a PS4 controller)
 * <p>
 * This specific sample assumes that the Hub is mounted on one of the three orthogonal planes
 * (X/Y, X/Z or Y/Z) and that the Hub has only been rotated in a range of 90 degree increments.
 * <p>
 * Note: if your Hub is mounted on a surface angled at some non-90 Degree multiple (like 30) look at
 *       the alternative SensorImuNonOrthogonal sample in this folder.
 * <p>
 * This "Orthogonal" requirement means that:
 * <p>
 * 1) The Logo printed on the top of the Hub can ONLY be pointing in one of six directions:
 *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
 * <p>
 * 2) The USB ports can only be pointing in one of the same six directions:<br>
 *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
 * <p>
 * So, To fully define how your Hub is mounted to the robot, you must simply specify:<br>
 *    logoFacingDirection<br>
 *    usbFacingDirection
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 * <p>
 * Finally, choose the two correct parameters to define how your Hub is mounted and edit this OpMode
 * to use those parameters.
 */
@Disabled

public class IMUClass
{
    // The IMU sensor object
    IMU imu;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    public void initIMU(HardwareMap hardwareMap)
    {
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

    }
    public double runIMU() throws InterruptedException {

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Retrieve Rotational Angles and Velocities
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void resetDegree()
    {
        imu.resetYaw();
    }
}
