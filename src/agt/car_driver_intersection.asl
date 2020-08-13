rl_parameter(policy, egreedy).

rl_algorithm(follow_street, dqn).
rl_observe(follow_street, lidar_data(list(1080))).
rl_reward(follow_street, R) :- reward(R).
rl_terminal(follow_street) :- crash.

rl_algorithm(turn_left, dqn).
rl_observe(turn_left, lidar_data(list(1080))).
rl_reward(turn_left, R) :- reward(R).
rl_terminal(turn_left) :- crash.

rl_algorithm(go_forward, dqn).
rl_observe(go_forward, lidar_data(list(1080))).
rl_reward(go_forward, R) :- reward(R).
rl_terminal(go_forward) :- crash.


+target("END1") : true <-
    +target_point("A");
    !follow_street;
    +starting_point("A");
    +target_point("B");
    !turn_left;
    +starting_point("B");
    +target_point("END1");
    !follow_street;
    new_target.

+target("END2") : true <-
    +target_point("A");
    !follow_street;
    +starting_point("A");
    +target_point("C");
    !go_forward;
    +starting_point("C");
    +target_point("END2");
    !follow_street.



+!follow_street : target_point(P) & position(P) <-
    +rl_reward(follow_street, 10);
    rl.execute(follow_street);
    -rl_reward(follow_street, 10).

+!follow_street : starting_point(P) & position(P) <-
    rl.execute(follow_street);
    !follow_street.

+!follow_street : position("") <-
    rl.execute(follow_street);
    !follow_street.

+!follow_street : true <-
    ?starting_point(P);
    +rl_reward(follow_street, -10);
    rl.execute(follow_street);
    -rl_reward(follow_street, -10);
    reset_to_position(P);
    !follow_street.



+!turn_left : target_point(P) & position(P) <-
    +rl_reward(turn_left, 10);
    rl.execute(turn_left);
    -rl_reward(turn_left, 10).

+!turn_left : starting_point(P) & position(P) <-
    rl.execute(turn_left);
    !turn_left.

+!turn_left : position("") <-
    rl.execute(turn_left);
    !turn_left.

+!turn_left : true <-
    ?starting_point(P);
    +rl_reward(turn_left, -10);
    rl.execute(turn_left);
    -rl_reward(turn_left, -10);
    reset_to_position(P);
    !turn_left.



+!go_forward : target_point(P) & position(P) <-
    +rl_reward(go_forward, 10);
    rl.execute(go_forward);
    -rl_reward(go_forward, 10).

+!go_forward : starting_point(P) & position(P) <-
    rl.execute(go_forward);
    !go_forward.

+!go_forward : position("") <-
    rl.execute(go_forward);
    !go_forward.

+!go_forward : true <-
    ?starting_point(P);
    +rl_reward(go_forward, -10);
    rl.execute(go_forward);
    -rl_reward(go_forward, -10);
    reset_to_position(P);
    !go_forward.




@action[rl_goal(follow_street), rl_param(direction(set(forward, right, left)))]
+!move(Direction) <- move(Direction).

@action[rl_goal(turn_left), rl_param(direction(set(forward, right, left)))]
+!move(Direction) <- move(Direction).

@action[rl_goal(go_forward), rl_param(direction(set(forward, right, left)))]
+!move(Direction) <- move(Direction).




{ include("$jacamoJar/templates/common-cartago.asl") }
