package frc.robot.subsystems.lift;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase{
    private final SparkMax motorMain = new SparkMax(18, MotorType.kBrushless);

    public LiftSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(40) 
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        
        motorMain.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command holdDownCommand() {
        return new RunCommand(() -> motorMain.set(-0.1), this);
    }

    public Command extendCommand() {
        return new RunCommand(() -> motorMain.setVoltage(12), this)
            .withTimeout(2.0) // Safety: Stop after 2 seconds if no limit switch
            .finallyDo(() -> motorMain.setVoltage(0));
    }

    public Command retractCommand() {
        return new RunCommand(() -> motorMain.setVoltage(-12.0), this)
            .finallyDo(() -> motorMain.setVoltage(-0.1)); // Go back to holding tension
    }
}
