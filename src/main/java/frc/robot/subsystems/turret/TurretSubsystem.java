package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import us.hebi.quickbuf.Utf8Decoder;

public class TurretSubsystem extends SubsystemBase{
    private SparkMax frontMotor, backMotor, hoodMotor, rotationMotor, uptakeMotor;

    private final double TICK_TO_DEG = 0.0;
    private final double MIN_ANGLE = -90.0;
    private final double MAX_ANGLE = 90.0;

    private PIDController rotationMotorController, hoodMotorController;
    
    public TurretSubsystem(){
        frontMotor = new SparkMax(11, MotorType.kBrushless);
        backMotor = new SparkMax(12, MotorType.kBrushless);
        hoodMotor = new SparkMax(13, MotorType.kBrushless);
        rotationMotor = new SparkMax(14, MotorType.kBrushless);
        uptakeMotor = new SparkMax(10, MotorType.kBrushless);
        
        uptakeMotor.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    


        rotationMotorController = new PIDController(0.15, 0, 0);
        hoodMotorController = new PIDController(0.15, 0, 0);
    }

    public Command shoot(){
        return new RunCommand(() -> {
            uptakeMotor.set(1.0);
        }).finallyDo(() -> uptakeMotor.set(0.0));
    }

    public Command shootForever(){
        return new InstantCommand(() -> {
            uptakeMotor.set(1.0);
        });
    }

    public double whatIsTheShootPower(double range){
        // TODO math
        return 1.0;
    }

    public void adjustHood(double range){
        // TODO math
        hoodMotorController.setSetpoint(0.5);
    }

    public void alignTurret(){
        // tX is the horizontal angle
        double ang = LimelightHelpers.getTX("turretLimelight");
        ang %= MAX_ANGLE - MIN_ANGLE;
        ang = degToTick(ang);
        rotationMotorController.setSetpoint(ang);
    }

    public void periodic(){
        // Align the turret
        alignTurret();
        double rotPower = clamp(rotationMotorController.calculate(rotationMotor.getEncoder().getPosition()), -0.5, 0.5);
        rotationMotor.set(rotPower);
        // Calculate hood angle
        double range = LimelightHelpers.getTY("turretLimelight");
        adjustHood(range);
        double hoodPower = clamp(hoodMotorController.calculate(hoodMotor.getEncoder().getPosition()), -0.5, 0.5);
        hoodMotor.set(hoodPower);
        // Calculate the shooting power based on the limelight range
        double power = whatIsTheShootPower(range);

        frontMotor.set(power);
        backMotor.set(power);
    }


    public double clamp(double d, double min, double max){
        if(d > max) return max;
        if(d < min) return min;
        return d;
    }

    public double degToTick(double ang){
        double calc = ang / TICK_TO_DEG;
        return calc;
    }
}
