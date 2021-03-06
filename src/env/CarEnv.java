import java.util.HashMap;
import java.util.List;
import java.util.Map;

import cartago.Artifact;
import cartago.OPERATION;
import cartago.ObsProperty;
import rest.RestClient;
import rest.StateRest;

public class CarEnv extends Artifact {

    private static final boolean IS_SIMULATOR = true;

    RestClient<List<Double>> car_env = new RestClient<>();

    @OPERATION
    public void init() {
        Map<String, String> parameters = new HashMap<>();
        parameters.put("simulator", Boolean.toString(IS_SIMULATOR));
        StateRest<List<Double>> state = car_env.initialize("CarEnv", parameters);
        Double[] lidar_data = state.getState().get(2).toArray(new Double[0]);
        defineObsProperty("lidar_data", (Object[]) lidar_data);
        String target = target_name((int) state.getState().get(1).get(0).doubleValue());
        defineObsProperty("target", target);
        String position = position_name((int) state.getState().get(0).get(0).doubleValue());
        defineObsProperty("position", position);
        defineObsProperty("reward", state.getReward());
    }

    @OPERATION
    public void move(String move) {

        StateRest<List<Double>> state;
        switch (move) {
            case "forward":
                state = car_env.step(0);
                break;
            case "right":
                state = car_env.step(1);
                break;
            case "left":
                state = car_env.step(2);
                break;
            case "lightly_right":
                state = car_env.step(3);
                break;
            case "lightly_left":
                state = car_env.step(4);
                break;
            default:
                state = car_env.step(5);
                break;
        }

        updatePercepts(state);
    }

    @OPERATION
    public void reset_to_position(String position) {
        StateRest<List<Double>> state = null;
        switch (position) {
            case "A":
                state = car_env.step(-2);
                break;
            case "B":
                state = car_env.step(-3);
                break;
            case "C":
                state = car_env.step(-4);
                break;
            default:
                state = car_env.step(-5);
        }
        updatePercepts(state);
    }

    @OPERATION
    public void new_target() {
        StateRest<List<Double>> state = car_env.step(-1);
        updatePercepts(state, true);
    }

    private String target_name(int index) {
        switch (index) {
            case 0:
                return "END1";
            case 1:
                return "END2";
            default:
                return "";
        }
    }

    private String position_name(int index) {
        switch (index) {
            case 0:
                return "A";
            case 1:
                return "B";
            case 2:
                return "C";
            case 3:
                return "END1";
            case 4:
                return "END2";
            case 5:
                return "START";
            default:
                return "";
        }
    }

    private void updatePercepts(StateRest<List<Double>> state) { updatePercepts(state, false); }

    private void updatePercepts(StateRest<List<Double>> state, boolean new_target) {
        ObsProperty lidar_data_prop = getObsProperty("lidar_data");
        Double[] lidar_data = state.getState().get(2).toArray(new Double[0]);
        lidar_data_prop.updateValues((Object[]) lidar_data);

        ObsProperty target_prop = getObsProperty("target");
        String target = target_name((int) state.getState().get(1).get(0).doubleValue());
        if(target_prop.getValue() != target || new_target) {
            target_prop.updateValues(target);
        }

        ObsProperty position_prop = getObsProperty("position");
        String position = position_name((int) state.getState().get(0).get(0).doubleValue());
        if(position_prop.getValue() != position) {
            position_prop.updateValues(position);
            if (!hasObsProperty("new_position"))
                defineObsProperty("new_position");
        } else {
            try {
                removeObsProperty("new_position");
            } catch (IllegalArgumentException e) {}
        }
        
        ObsProperty reward = getObsProperty("reward");
        reward.updateValue(0, state.getReward());

        if (state.isTerminal()) {
            if (!hasObsProperty("crash"))
                defineObsProperty("crash");
        } else {
            try {
                removeObsProperty("crash");
            } catch (IllegalArgumentException e) {}
        }
    }
}
