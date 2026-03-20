package frc.robot.subsystems.indexing;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.*;

public class IndexingSubsystem extends SubsystemBase{
    private final SparkMax indexingMotorMain = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax indexingMotorTurret = new SparkMax(10, MotorType.kBrushless);

    private int reverse = -1;
    private boolean indexingMotorTurretDirection = false;
    private double power = 0.2;
    
    public IndexingSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
    
        config.follow(9, indexingMotorTurretDirection);

        indexingMotorTurret.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
        indexingMotorMain.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startIntakeCommand() {
        return new RunCommand(
            () -> indexingMotorMain.set(power * reverse), 
            this
        ).finallyDo(() -> indexingMotorMain.set(0));
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(() -> indexingMotorMain.set(0), this);
    }

    public Command toggleReverseCommand() {
        return new InstantCommand(() -> {
            reverse *= -1;
        });
    }
}
