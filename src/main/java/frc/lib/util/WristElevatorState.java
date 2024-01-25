package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristElevatorState {

        public final Rotation2d angle;
        public final double height;


        /**
         * Create a new Superstructure State
         * 
         * @param angle angle of wrist with 0 being horizontal
         * @param height height of elevator with 0 being bottom
         */
        public WristElevatorState(Rotation2d angle, double height) {
            this.angle = angle;
            this.height = height;
        }

        /**
         * Create a new Superstructure State
         * 
         * @param angle (degrees) angle of wrist with 0 being horizontal
         * @param height height of elevator with 0 being bottom
         */
        public WristElevatorState(double angle, double height) {
            this(Rotation2d.fromDegrees(angle), height);
        }


    }