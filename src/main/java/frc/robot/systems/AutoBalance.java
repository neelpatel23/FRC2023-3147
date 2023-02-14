package frc.robot.systems;

import frc.robot.systems.hardware.NavX;
import frc.robot.Robot;

public class AutoBalance {
    private Robot robot = new Robot();
    private NavX nav = new NavX();

    double DriveX = robot.controller.getLeftX();
    double DriveY = robot.controller.getLeftY();

    double pitchAngleDegrees = nav.getPitchMotion();
    double rollAngleDegrees = nav.getRollMotion();

    
    
}
