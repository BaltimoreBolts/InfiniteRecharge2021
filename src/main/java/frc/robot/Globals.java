package frc.robot;

public final class Globals {
    public static class PCArray {
        private static boolean[] PCArray = { false, false, false };

        public static void moveUp() {
            PCArray[2] = PCArray[1];
            PCArray[1] = PCArray[0];
            PCArray[0] = false;
        }

        public static void moveDown() {
            PCArray[0] = PCArray[1];
            PCArray[1] = PCArray[2];
            PCArray[2] = false;
        }

        public static void intakePC(){
            PCArray[0] = true;
        }

        // returns lowest ball position, or -1 if no balls loaded
        public static int getLowestPCPos() {
            int PCPos = -1;
            int arr_size = PCArray.length;
            for (int i = 0; i < arr_size; i++) {
                if (PCArray[i]) {
                    PCPos = i;
                    break;
                }
            }
            return PCPos;
        }

        // returns highest ball position, or -1 if no balls loaded
        public static int getHighestPCPos() {
            int PCPos = -1;
            int arr_size = PCArray.length;
            for (int i = arr_size; i > 0; i--) {
                if (PCArray[i - 1]) {
                    PCPos = i;
                    break;
                }
            }
            return PCPos;
        }

        @Override
        public String toString() {
            return String.format("[0: %b | 1: %b | 2: %b]", PCArray[0], PCArray[1], PCArray[2]);
        }

    }

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
