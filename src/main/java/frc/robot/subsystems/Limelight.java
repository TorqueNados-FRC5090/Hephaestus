package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    // It's good practice to make these public or provide getters so your commands can access them
    public final NetworkTable limelightLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
    public final NetworkTable limelightRight = NetworkTableInstance.getDefault().getTable("limelight-right");
    public final NetworkTable limelightBack = NetworkTableInstance.getDefault().getTable("limelight-back");

    public Limelight() {
        // Limelight setup, if any
    }

    // Now requires you to specify WHICH table (camera) to read from
    public double getXOffset(NetworkTable table) {
        return table.getEntry("tx").getDouble(0.0); // Horizontal offset (degrees)
    }

    public double getYOffset(NetworkTable table) {
        return table.getEntry("ty").getDouble(0.0); // Vertical offset (degrees) - Fixed comment
    }

    public boolean hasTarget(NetworkTable table) {
        return table.getEntry("ta").getDouble(0.0) > 0.0; // If target area is > 0
    }

    public int getTargetId(NetworkTable table) {
        return (int) table.getEntry("tid").getDouble(0.0); // Returns the ID of the primary target
    }

    @Override
    public void periodic() {
        // Output Left Limelight Data
        SmartDashboard.putNumber("Left LL/X Offset", getXOffset(limelightLeft));
        SmartDashboard.putBoolean("Left LL/Has Target", hasTarget(limelightLeft));
        SmartDashboard.putNumber("Left LL/Target ID", getTargetId(limelightLeft));

        // Output Right Limelight Data
        SmartDashboard.putNumber("Right LL/X Offset", getXOffset(limelightRight));
        SmartDashboard.putBoolean("Right LL/Has Target", hasTarget(limelightRight));
        SmartDashboard.putNumber("Right LL/Target ID", getTargetId(limelightRight));

        // Output Back Limelight Data
        SmartDashboard.putNumber("Back LL/X Offset", getXOffset(limelightBack));
        SmartDashboard.putBoolean("Back LL/Has Target", hasTarget(limelightBack));
        SmartDashboard.putNumber("Back LL/Target ID", getTargetId(limelightBack));
    }
}