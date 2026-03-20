package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax motorMain = new SparkMax(13 ,MotorType.kBrushless);
    private final SparkMax motorExtendOne = new SparkMax(11, MotorType.kBrushless);
    private final SparkMax motorExtendTwo = new SparkMax(12, MotorType.kBrushless);

    private double power = 0.2;
    private int reverse = -1;

    public IntakeSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
    
        config.follow(11, true);

        motorExtendTwo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorExtendOne.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorMain.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command startCommand() {
        return new RunCommand(
            () -> motorMain.set(power * reverse), 
            this
        ).finallyDo(() -> motorMain.set(0));
    }

    public Command stopCommand() {
        return new InstantCommand(() -> motorMain.set(0), this);
    }

    public Command toggleReverseCommand() {
        return new InstantCommand(() -> {
            reverse *= -1;
        });
    }

    // TODO: This function
    public Command extendCommand() {
        return new RunCommand(
            () -> {
                System.out.println("Extending");
            }, this);
    }
}
