package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Aim extends Command{
    private SwerveSubsystem swerve;

    public Aim(SwerveSubsystem swerveSubsystem){
        this.swerve = swerveSubsystem;
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        double kP = 0.035;

        // Rotation
        double tX = LimelightHelpers.getTX("limelight-nest") * kP; 
        tX *= swerve.getSwerveDrive().getMaximumChassisAngularVelocity() * -1;

        // Range
        kP = 0.1;
        double forwardSpeed = LimelightHelpers.getTY("limelight-nest") * kP;
        forwardSpeed *= swerve.getSwerveDrive().getMaximumChassisVelocity() * -1;

        swerve.drive(new ChassisSpeeds(forwardSpeed, 0, tX));
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return swerve.getIsDriving();
    }
}
