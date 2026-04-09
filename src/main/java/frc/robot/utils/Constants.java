package frc.robot.utils;

import edu.wpi.first.math.util.Units;

public class Constants {

    public class DriverStation {
        public static final int xboxControllerID = 0;
        public static final int leftFlightStickID = 1;
        public static final int rightFlightStickID = 2;
        public static final double DriveSpeedMultiplier = Units.feetToMeters(15.724);;
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
        public static final int shooter1 = 32;
        public static final int shooter2 = 33;
        // Climb 41-50
        public static final int climb1 = 41;
        public static final int climb2 = 42;

    }

    public class Manipulator {

        public static final double autoAgitatorSpeed = 1;
        public static final double autoShooterSpeed = 1000;
        public static final double shooterProportion = 0.02;
        public static final double shooterIntegral = 0;
        public static final double shooterDerivative = 0;
        public static final double feederProportion = 0;
        public static final double feederIntegral = 0;
        public static final double feederDerivative = 0;
        public static final double intakeUpRotations = 0;
        public static final double intakeDownRotations = -4; // gear ratio * output rotations
        public static final double pivotProportion = .1; // go to xx% speed at full error
        public static final double pivotDerivative = 20;
        public static final double pivotIntegral = .1;
        public static final double climberLProportion = 0;
        public static final double climberLDerivative = 0;
        public static final double climberLIntegral = 0;
        public static final double climberRProportion = 0;
        public static final double climberRDerivative = 0;
        public static final double climberRIntegral = 0;
        public static final double intakeRollerSpeed = 1;

        public static final double LongShooterSpeed = 3700;
        public static final double MediumShooterSpeed = 3000;
        public static final double ShortShooterSpeed = 500;
         public static final double offShooterSpeed = 0;

        public static final double idleSpeed = 0;
        public static final double FeederSpeed = .72;
        public static final double climberRotationsUp = 0;
        public static final double climberRotationsDown = 0;
        public static final double currentDerivLim = 10;

        public static final int maxPivotCurrentLimit = 5;


        public static final double minEncoderpose = -0.4142857909202576;
//intake speeds
        public static final double intakeSpeedOut = 0;
         public static final double intakeSpeedIn = 0;


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
}
