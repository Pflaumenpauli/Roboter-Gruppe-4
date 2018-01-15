package parkingRobot;

/**
 * Created by paulforster on 30.05.17.
 */

public interface IGuidance {
    /**
     * states for the main finite state machine. This main states are requirements because they invoke different
     * display modes in the human machine interface.
     */
    enum CurrentStatus {
        /**
         * indicates that robot is following the line and maybe detecting parking slots
         */
        DRIVING,
        /**
         * indicates that robot is not doing anything -> pause
         */
        INACTIVE,
        /**
         * indicates that robot is performing an parking maneuver
         */
        PARKING,
        /**
         * indicates that shutdown of main program has initiated
         */
        EXIT
    }
}
