package frc.robot.subsystems.indexing;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.*;

public class IndexingSubsystem extends SubsystemBase{
    private final SparkMax motorMain = new SparkMax(9, MotorType.kBrushless);
    private final SparkMax motorTurret = new SparkMax(10, MotorType.kBrushless);

    private int reverse = -1;
    private double power = 0.4;
    
    public IndexingSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
    
        // config.follow(9, indexingMotorTurretDirection);

        motorTurret.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
        motorMain.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startCommand() {
        return new RunCommand(
            () -> motorMain.set(power * reverse), 
            this 
        );
    }

    // public Command stopCommand() {
    //     return new InstantCommand(() -> motorMain.set(0), this);
    // }

    public Command toggleReverseCommand() {
        return new InstantCommand(() -> {
            reverse *= -1;
        });
    }


}
