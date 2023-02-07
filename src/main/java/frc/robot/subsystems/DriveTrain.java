package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
	// New Drive SubSystem
	public DriveTrain() {}

	public CommandBase driveTrainCommand() {
		return runOnce(
			() -> {

		});
	}

	public boolean driveBoolean() {
		return false;
	}

	@Override
	public void periodic() {
		// System.out.println("Please Work");
		return;
	}

	public void testPeriodic() {
		System.out.println("Messing Around");
		return;
	}

	@Override
	public void simulationPeriodic() {

	}
}
