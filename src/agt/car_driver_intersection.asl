rl_parameter(policy, egreedy).

rl_algorithm(follow_street, dqn).
rl_observe(follow_street, lidar_data(list(1080))).
rl_terminal(follow_street) :- crash.
rl_terminal(follow_street) :- new_position.
rl_reward(follow_street, R) :- reward(R).
rl_reward(follow_street, 10) :- target_point(P) & position(P).
rl_reward(follow_street, -10) :- new_position & position(P)
                                    & not target_point(P) & not starting_point(P)
                                    & not position("").

rl_algorithm(turn_left, dqn).
rl_observe(turn_left, lidar_data(list(1080))).
rl_terminal(turn_left) :- crash.
rl_terminal(turn_left) :- new_position.
rl_reward(turn_left, R) :- reward(R).
rl_reward(turn_left, 10) :- target_point(P) & position(P).
rl_reward(turn_left, -10) :- new_position & position(P)
                                    & not target_point(P) & not starting_point(P)
                                    & not position("").

rl_algorithm(go_forward, dqn).
rl_observe(go_forward, lidar_data(list(1080))).
rl_terminal(go_forward) :- crash.
rl_terminal(go_forward) :- new_position.
rl_reward(go_forward, R) :- reward(R).
rl_reward(go_forward, 10) :- target_point(P) & position(P).
rl_reward(go_forward, -10) :- new_position & position(P)
                                    & not target_point(P) & not starting_point(P)
                                    & not position("").


+target("END1") : true <-
    .println("");
    .println("target: END1");
    -+starting_point("START");
    -+target_point("A");
    !follow_street;
    .println("reached sub-target A");
    -+starting_point("A");
    -+target_point("B");
    !turn_left;
    .println("reached sub-target B");
    -+starting_point("B");
    -+target_point("END1");
    !follow_street;
    .println("reached target END1");
    .println("getting new target");
    new_target.

+target("END2") : true <-
    .println("");
    .println("target: END2");
    -+starting_point("START");
    -+target_point("A");
    !follow_street;
    .println("reached sub-target A");
    -+starting_point("A");
    -+target_point("C");
    !go_forward;
    .println("reached sub-target C");
    -+starting_point("C");
    -+target_point("END2");
    !follow_street;
    .println("reached target END2");
    .println("getting new target");
    new_target.



+!follow_street : target_point(P) & position(P) <-
    move("stop");
    .println("reward +10: reached ", P).

+!follow_street : starting_point(P) & position(P) <-
    rl.execute(follow_street);
    !follow_street.

+!follow_street : position("") <-
    rl.execute(follow_street);
    !follow_street.

+!follow_street : true <-
    ?starting_point(P);
    move("stop");
    .println("reward -10: wrong direction taken");
    .println("resetting to point ", P);
    reset_to_position(P);
    !follow_street.



+!turn_left : target_point(P) & position(P) <-
    move("stop");
    .println("reward +10: reached ", P).

+!turn_left : starting_point(P) & position(P) <-
    rl.execute(turn_left);
    !turn_left.

+!turn_left : position("") <-
    rl.execute(turn_left);
    !turn_left.

+!turn_left : true <-
    ?starting_point(P);
    move("stop");
    .println("reward -10: wrong direction taken");
    .println("resetting to point ", P);
    reset_to_position(P);
    !turn_left.



+!go_forward : target_point(P) & position(P) <-
    move("stop");
    .println("reward +10: reached ", P).

+!go_forward : starting_point(P) & position(P) <-
    rl.execute(go_forward);
    !go_forward.

+!go_forward : position("") <-
    rl.execute(go_forward);
    !go_forward.

+!go_forward : true <-
    ?starting_point(P);
    move("stop");
    .println("reward -10: wrong direction taken");
    .println("resetting to point ", P);
    reset_to_position(P);
    !go_forward.




@action1[rl_goal(follow_street), rl_param(direction(set(forward, right, left)))]
+!move(Direction) <- move(Direction).

@action2[rl_goal(turn_left), rl_param(direction(set(forward, right, left)))]
+!move(Direction) <- move(Direction).

@action3[rl_goal(go_forward), rl_param(direction(set(forward, right, left)))]
+!move(Direction) <- move(Direction).




{ include("$jacamoJar/templates/common-cartago.asl") }
