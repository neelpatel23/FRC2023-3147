package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.math.controller.PIDController;

public class LimeLightSubsystem extends SubsystemBase {
    private final NetworkTableInstance inst;
    private final NetworkTable table;
    private final PIDController pidcontrol;
    private final float kp = 0.003125f;
    private final float ki = 0.01f;
    private final float kd = 0f;

    private float prev_tx = 0f;

    public LimeLightSubsystem() {
        pidcontrol = new PIDController(1, ki, kd);
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("limelight"); // initiate limelight table
    }
    
}
