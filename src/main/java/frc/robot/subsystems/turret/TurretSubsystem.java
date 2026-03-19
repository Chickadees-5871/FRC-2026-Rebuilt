package frc.robot.subsystems.turret;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class TurretSubsystem extends SubsystemBase{
    private SparkMax frontMotor, backMotor, hoodMotor, rotationMotor;

    private PIDController rotationMotorController, hoodMotorController;
    
    public TurretSubsystem(){
        frontMotor = new SparkMax(11, MotorType.kBrushless);
        backMotor = new SparkMax(12, MotorType.kBrushless);
        hoodMotor = new SparkMax(13, MotorType.kBrushless);
        rotationMotor = new SparkMax(14, MotorType.kBrushless);

        rotationMotorController = new PIDController(0.15, 0, 0);
        hoodMotorController = new PIDController(0.15, 0, 0);
    }

    public void shootABall(){
        double range = LimelightHelpers.getTY("turretLimelight");
        // Calculate the shooting power based on the limelight range
        double power = whatIsTheShootPower(range);

        frontMotor.set(power);
        backMotor.set(power);
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

        rotationMotorController.setSetpoint(ang); // fix this, need to find relationship between encoder tick and angle
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
    }


    public double clamp(double d, double min, double max){
        if(d > max) return max;
        if(d < min) return min;
        return d;
    }
}
