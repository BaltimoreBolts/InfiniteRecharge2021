package frc.robot;

public final class Globals {
    public static boolean[] PCArray = {false, false, false, false};
    public static powerCellStates powerCellState;
    public static enum powerCellStates {
        IDLE("Idle"), // robot not doing anything
        HARVESTING("Harvesting"), // state of intaking balls
        SHOOTING("Shooting"), // state of shooting balls
        PURGING("Purging"); // state of removing balls

        private String name;
        private powerCellStates(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }
}
