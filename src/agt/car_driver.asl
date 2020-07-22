rl_algorithm(carenv, dqn).
rl_parameter(policy, egreedy).

rl_observe(carenv, lidar_data(list(1080))).

rl_reward(carenv, R) :- reward(R).

rl_terminal(carenv) :- crash.

!start.

+!start : true <- rl.execute(carenv); !start. //!start in order to continue after the end of the episode

@action[rl_goal(carenv), rl_param(direction(set(forward, right, left)))]
+!move(Direction) <- move(Direction).

{ include("$jacamoJar/templates/common-cartago.asl") }
