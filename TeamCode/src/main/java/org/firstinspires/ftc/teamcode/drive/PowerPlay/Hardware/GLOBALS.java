package org.firstinspires.ftc.teamcode.drive.PowerPlay.Hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class GLOBALS {

    // CAMERA CONSTANTS
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    // Intake and Claw
    double CLAW_OPEN = 0.2;
    double CLAW_CLOSE = 1;

    double Intake = 1;
    double Outtake = -1;
    double stopIntake = 0;

    // lift pos
    double High_Goal = 1970;
    double hover_high = 1600;
    double Mid_Goal = 0;  // need to measured
    double Low_Goal = 0;  // need to be measured
    double down = 0;



}
