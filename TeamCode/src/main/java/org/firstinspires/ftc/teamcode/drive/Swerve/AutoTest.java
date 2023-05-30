package org.firstinspires.ftc.teamcode.drive.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTest extends LinearOpMode {

    Robot robot;

    public void runOpMode() {
        robot = new Robot(this, true);
        robot.initIMU();

        //simple sequence to demonstrate the three main autonomous primitives

        //rotate modules to face to the right
        robot.driveController.rotateModules(Vector2d.RIGHT, true, 0.1, this);

        //drive 20 cm to the right (while facing forward)
        robot.driveController.drive(Vector2d.RIGHT, 20, 1, this);

        //turn to face robot right
        robot.driveController.rotateRobot(Angle.RIGHT, this);
    }
}
