use goaprs::{Action, GoapError, GoapVisualizer, Planner, State};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_goap_workflow() {
        // Create actions
        let mut gather_wood = Action::new("gather_wood", 1.0).unwrap();
        gather_wood.preconditions.set("has_axe", "true");
        gather_wood.effects.set("has_wood", "true");

        let mut build_house = Action::new("build_house", 2.0).unwrap();
        build_house.preconditions.set("has_wood", "true");
        build_house.effects.set("has_house", "true");

        // Create the planner with available actions
        let planner = Planner::new(vec![gather_wood, build_house]);

        // Define the current state
        let mut current_state = State::new();
        current_state.set("has_axe", "true");
        current_state.set("has_wood", "false");
        current_state.set("has_house", "false");

        // Define the goal state
        let mut goal_state = State::new();
        goal_state.set("has_house", "true");

        // Find a plan
        let plan = planner.plan(&current_state, &goal_state).unwrap();
        assert_eq!(plan.len(), 2);
        assert_eq!(plan[0].name, "gather_wood");
        assert_eq!(plan[1].name, "build_house");
    }

    #[test]
    fn test_impossible_goal() {
        let mut action = Action::new("impossible_action", 1.0).unwrap();
        action.preconditions.set("impossible", "true");
        action.effects.set("goal", "true");

        let planner = Planner::new(vec![action]);
        let current_state = State::new();
        let mut goal_state = State::new();
        goal_state.set("goal", "true");

        let result = planner.plan(&current_state, &goal_state);
        assert!(result.is_err());
        assert!(matches!(result, Err(GoapError::NoPlanFound)));
    }

    #[test]
    fn test_multiple_paths_to_goal() {
        let mut action1 = Action::new("cheap_action", 1.0).unwrap();
        action1.preconditions.set("start", "true");
        action1.effects.set("goal", "true");

        let mut action2 = Action::new("expensive_action", 5.0).unwrap();
        action2.preconditions.set("start", "true");
        action2.effects.set("goal", "true");

        let planner = Planner::new(vec![action1, action2]);
        let mut current_state = State::new();
        current_state.set("start", "true");
        let mut goal_state = State::new();
        goal_state.set("goal", "true");

        let plan = planner.plan(&current_state, &goal_state).unwrap();
        assert_eq!(plan.len(), 1);
        assert_eq!(plan[0].name, "cheap_action");
    }

    #[test]
    fn test_invalid_action_cost() {
        let result = Action::new("invalid_action", 0.0);
        assert!(result.is_err());
        assert!(matches!(result, Err(GoapError::InvalidActionCost)));

        let result = Action::new("invalid_action", -1.0);
        assert!(result.is_err());
        assert!(matches!(result, Err(GoapError::InvalidActionCost)));
    }

    #[test]
    fn test_visualize_planning() {
        // Create actions
        let mut gather_wood = Action::new("gather_wood", 1.0).unwrap();
        gather_wood.preconditions.set("has_axe", "true");
        gather_wood.effects.set("has_wood", "true");

        let mut build_house = Action::new("build_house", 2.0).unwrap();
        build_house.preconditions.set("has_wood", "true");
        build_house.effects.set("has_house", "true");

        let actions = vec![gather_wood.clone(), build_house.clone()];

        // Create the planner with available actions
        let planner = Planner::new(actions.clone());

        // Define the current state
        let mut current_state = State::new();
        current_state.set("has_axe", "true");
        current_state.set("has_wood", "false");
        current_state.set("has_house", "false");

        // Define the goal state
        let mut goal_state = State::new();
        goal_state.set("has_house", "true");

        // Find a plan
        let plan = planner.plan(&current_state, &goal_state).unwrap();

        // Visualize the plan
        let visualizer = GoapVisualizer::new();
        visualizer
            .visualize_plan(
                &actions,
                &current_state,
                &goal_state,
                &plan,
                "planning_visualization.dot",
            )
            .unwrap();
    }
}
