import java.util.HashMap;
import java.util.Map;

import cartago.Artifact;
import cartago.OPERATION;
import cartago.ObsProperty;
import rest.RestClient;
import rest.StateRest;

public class CarEnv extends Artifact{

    private static final boolean IS_SIMULATOR = true;
    
    RestClient<Double> car_env = new RestClient<>();

    @OPERATION
    public void init() {
        Map<String, String> parameters = new HashMap<>();
        parameters.put("simulator", Boolean.toString(IS_SIMULATOR));
        StateRest<Double> state = car_env.initialize("CarEnv", parameters);
        defineObsProperty("lidar_data", state.getState().toArray(new Double[0]));
        defineObsProperty("reward", state.getReward());
    }

    @OPERATION
    public void move(String move) {

        StateRest<Double> state;
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
    
    private void updatePercepts(StateRest<Double> state) {
        ObsProperty lidar_data = getObsProperty("lidar_data");
        lidar_data.updateValues(state.getState().toArray(new Double[0]));
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
