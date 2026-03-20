package frc.robot.subsystems.indexing;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.*;

public class IndexingSubsystem extends SubsystemBase{
    private final SparkMax intakeMotorOne = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax intakeMotorTwo = new SparkMax(10, MotorType.kBrushless);

    private int reverse = -1;
    private double power = 0.2;
    
    public IndexingSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
    
        config.follow(9);

        intakeMotorTwo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
        intakeMotorOne.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startIntakeCommand() {
        return new RunCommand(
            () -> intakeMotorOne.set(power * reverse), 
            this
        ).finallyDo(() -> intakeMotorOne.set(0));
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(() -> intakeMotorOne.set(0), this);
    }

    public Command toggleReverseCommand() {
        return new InstantCommand(() -> {
            reverse *= -1;
        });
    }
}
