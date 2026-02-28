package frc.robot.utils;

public class Constants {

    public class DriverStation {
        public static final int xboxControllerID = 0;
        public static final int leftFlightStickID = 1;
        public static final int rightFlightStickID = 2;
    }

    public class CANBus {

        public static final int FRAngle = 4;
        public static final int FRDrive = 3;
        public static final int FLAngle = 2;
        public static final int FLDrive = 1;
        public static final int BRAngle = 8;
        public static final int BRDrive = 7;
        public static final int BLAngle = 6;
        public static final int BLDrive = 5;
        // Intake 11-20
        public static final int Intake1 = 11;
        public static final int pivotIntake1 = 12;
        public static final int pivotIntake2 = 13;

        // Agitators 21-30
        public static final int agitator = 21;

        // Shooters 31-40
        public static final int feeder = 31;
        public static final int shooter = 32;
        // Climb 41-50
        public static final int climb1 = 41;
        public static final int climb2 = 42;
        public static final int colorSensor = 45;

    }

    public class Manipulator {

        public static final double autoAgitatorSpeed = 0.5;
        public static final double autoIntakeSpeed = 0.5;
        public static final double autoShooterSpeed = 0.5;
        public static final double shooterProportion = 0;
        public static final double shooterIntegral = 0;
        public static final double shooterDerivative = 0;
        public static final double feederProportion = 0;
        public static final double feederIntegral = 0;
        public static final double feederDerivative = 0;
        public static final double pivotProportion = 0;
        public static final double pivotDerivative = 0;
        public static final double pivotIntegral = 0;
        public static final double climberLProportion = 0;
        public static final double climberLDerivative = 0;
        public static final double climberLIntegral = 0;
        public static final double climberRProportion = 0;
        public static final double climberRDerivative = 0;
        public static final double climberRIntegral = 0;
        public static final double teleopAgitatorSpeed = 0.7;
        public static final double teleopIntakeSpeed = 0.7;
        public static final double teleopShooterSpeed = 0.7;
        public static final double intakeUpRotations = 0;
        public static final double intakeDownRotations = 0;
        public static final double idleSpeed = 0;
        public static final double FeederSpeed = 0;
        public static final double climberRotationsUp = 0;
        public static final double climberRotationsDown = 0;

    }

    public class FieldConstants {

        public static final double redGoalY = 0;
        public static final double redGoalX = 0;
        public static final double blueGoalY = 0;
        public static final double blueGoalX = 0;
        public static final double redClimberY = 0;
        public static final double redClimberX = 0;
        public static final double blueClimberY = 0;
        public static final double blueClimberX = 0;

    }

}
