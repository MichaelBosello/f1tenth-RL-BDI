import java.util.HashMap;
import java.util.Map;

import cartago.Artifact;
import cartago.OPERATION;
import cartago.ObsProperty;
import rest.RestClient;
import rest.StateRest;

import jason.asSyntax.Atom;

public class CarEnv extends Artifact{
    
    RestClient<Double> car_env = new RestClient<>();

    @OPERATION
    public void init() {
        Map<String, String> parameters = new HashMap<>();
        StateRest<Double> state = car_env.initialize("CarEnv", parameters);
        defineObsProperty("lidar_data", state.getState().get(0));
        defineObsProperty("cart_velocity", state.getState().get(1));
        defineObsProperty("pole_position", state.getState().get(2));
        defineObsProperty("pole_velocity", state.getState().get(3));
        
        defineObsProperty("rl_parameter", new Atom("policy"), new Atom("egreedy"));
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
        lidar_data.updateValue(0, state.getState().get(0));

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
