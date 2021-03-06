/* Team 5687 (C)2020-2021 */
package org.frc5687.rapidreact;


public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and
     * then in numerical order. Note that for CAN, ids must be unique per device type, but not
     * across types. Thus, you cannot have two SparkMax controllers with Id 0, but you can have a
     * SparkMax with Id 0 and a TalonSRX with Id 0.
     */
    public static class CAN {
        /**
         *          N
         *          |
         *      E -- -- W
         *          |
         *          S
         */

        public static class TALONFX {
            public static final int NORTH_EAST_OUTER = 1;
            public static final int NORTH_EAST_INNER = 2;
            public static final int SOUTH_EAST_INNER = 3;
            public static final int SOUTH_EAST_OUTER = 4;
            public static final int SOUTH_WEST_INNER = 5;
            public static final int SOUTH_WEST_OUTER = 6;
            public static final int NORTH_WEST_INNER = 8;
            public static final int NORTH_WEST_OUTER = 7;
            public static final int CATAPULT_SPRING = 9;
            public static final int INTAKE_ROLLER = 13;

            public static final int STATIONARY_CLIMBER = 15;
            public static final int ROCKER_CLIMBER = 14;
        }
        
        public static class CANDLE{
            public static final int PORT = 0;
        }

        public static class SPARKMAX {
            public static final int WINCH_BABY_NEO = 10;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order. Note that
     * for PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {
        public static final int INTAKE_STOPPER = 0;
    }

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order. Note that
     * for PCH only one device can connect to each port, so the numbers should be unique.
     * This is the REVRobotics Pneumatic Control Hub.
     */
    public static class PCH {
        public static final int COMP = 1;
        public static final int INTAKE_HIGH = 10;
        public static final int INTAKE_LOW = 5;
        public static int RELEASE_PIN_HIGH = 7;
        public static int RELEASE_PIN_LOW = 8;
        public static final int CLIMBER_IN = 6;
        public static final int CLIMBER_OUT = 9;
        public static final int INDEXER_OUT = 11;
        public static final int INDEXER_IN = 4;
    }

    /**
     * There should be an entry here for each PDP breaker, preferrably in numerical order. Note that
     * only on device can be connected to each breaker, so the numbers should be unique.
     */
    public static class PDP {}

    /**
     * There should be an entry here for each Analgo port, preferrably in numerical order. Note that
     * for Analog only one device can connect to each port, so the numbers should be unique.
     */
    public static class Analog {
        public static final int MODE_SWITCH = 0;
        public static final int POSITION_SWITCH = 1;
    }
    /**
     * There should be an entry here for each DIO port, preferrably in numerical order. Note that
     * for DIO only one device can connect to each port, so the numbers should be unique.
     */
    public static class DIO {
        public static final int NORTH_EAST = 0;
        public static final int SOUTH_EAST = 1;
        public static final int SOUTH_WEST = 2;
        public static final int NORTH_WEST = 3;

        public static final int SPRING_HALL_EFFECT = 4;
        public static final int ARM_HALL_EFFECT = 5;
        public static final int INTAKE_HALL_EFFECT = 22;

        public static final int PROXIMITY_SENSOR = 6;
        public static final int INTAKE_PROXIMITY_SENSOR = 9;

        // public static final int ROCKER_ARM_TOP_HALL = 9; 
        public static final int ROCKER_ARM_BOTTOM_HALL = 7;
        public static final int STATIONARY_ARM_BOTTOM_HALL = 8;

    }
}