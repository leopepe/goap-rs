use crate::search::{AStarSearch, SearchAlgorithm};
use crate::{Action, Result, State};

/// The GOAP planner that finds the optimal sequence of actions
pub struct Planner {
    actions: Vec<Action>,
    search_algorithm: Box<dyn SearchAlgorithm>,
}

impl Planner {
    /// Create a new planner with the given actions using A* search
    pub fn new(actions: Vec<Action>) -> Self {
        Self {
            actions,
            search_algorithm: Box::new(AStarSearch),
        }
    }

    /// Create a new planner with the given actions and search algorithm
    pub fn with_search_algorithm(
        actions: Vec<Action>,
        search_algorithm: Box<dyn SearchAlgorithm>,
    ) -> Self {
        Self {
            actions,
            search_algorithm,
        }
    }

    /// Find a plan to achieve the goal state from the current state
    pub fn plan(&self, current_state: &State, goal_state: &State) -> Result<Vec<Action>> {
        self.search_algorithm
            .search(&self.actions, current_state, goal_state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::search::{AStarSearch, DijkstraSearch};
    use crate::{Action, State};

    fn make_action(
        name: &str,
        cost: f32,
        pre: Vec<(&str, bool)>,
        eff: Vec<(&str, bool)>,
    ) -> Action {
        let mut action = Action::new(name, cost).unwrap();
        for (k, v) in pre {
            action.preconditions.set(k, v);
        }
        for (k, v) in eff {
            action.effects.set(k, v);
        }
        action
    }

    #[test]
    fn test_simple_plan() {
        // a -> b -> c
        let a = make_action("a", 1.0, vec![("start", true)], vec![("mid", true)]);
        let b = make_action("b", 1.0, vec![("mid", true)], vec![("end", true)]);
        let c = make_action("c", 1.0, vec![("end", true)], vec![("goal", true)]);
        let planner = Planner::new(vec![a.clone(), b.clone(), c.clone()]);

        let mut initial = State::new();
        initial.set("start", true);
        initial.set("mid", false);
        initial.set("end", false);
        initial.set("goal", false);

        let mut goal = State::new();
        goal.set("goal", true);

        let plan = planner.plan(&initial, &goal).unwrap();
        let names: Vec<_> = plan.iter().map(|a| a.name.as_str()).collect();
        assert_eq!(names, ["a", "b", "c"]);
    }

    #[test]
    fn test_no_plan_found() {
        let a = make_action("a", 1.0, vec![("foo", true)], vec![("bar", true)]);
        let planner = Planner::new(vec![a]);
        let mut initial = State::new();
        initial.set("foo", false);
        let mut goal = State::new();
        goal.set("bar", true);
        let result = planner.plan(&initial, &goal);
        assert!(result.is_err());
    }

    #[test]
    fn test_plan_with_multiple_paths() {
        // Two ways to reach the goal, but one is cheaper
        let a = make_action("a", 1.0, vec![("start", true)], vec![("goal", true)]);
        let b = make_action("b", 5.0, vec![("start", true)], vec![("goal", true)]);
        let planner = Planner::new(vec![a.clone(), b.clone()]);
        let mut initial = State::new();
        initial.set("start", true);
        let mut goal = State::new();
        goal.set("goal", true);
        let plan = planner.plan(&initial, &goal).unwrap();
        // Should pick the cheaper action
        assert_eq!(plan[0].name, "a");
    }

    #[test]
    fn test_different_search_algorithms() {
        let a = make_action("a", 1.0, vec![("start", true)], vec![("goal", true)]);
        let b = make_action("b", 5.0, vec![("start", true)], vec![("goal", true)]);
        let actions = vec![a.clone(), b.clone()];

        let mut initial = State::new();
        initial.set("start", true);
        let mut goal = State::new();
        goal.set("goal", true);

        // Test A* search
        let astar_planner = Planner::with_search_algorithm(actions.clone(), Box::new(AStarSearch));
        let astar_plan = astar_planner.plan(&initial, &goal).unwrap();
        assert_eq!(astar_plan[0].name, "a");

        // Test Dijkstra search
        let dijkstra_planner = Planner::with_search_algorithm(actions, Box::new(DijkstraSearch));
        let dijkstra_plan = dijkstra_planner.plan(&initial, &goal).unwrap();
        assert_eq!(dijkstra_plan[0].name, "a");
    }

    #[test]
    fn test_complex_planning_scenario() {
        // Create a complex set of actions with multiple preconditions and effects
        let mut actions = Vec::new();

        // Action 1: Gather Resources
        let mut gather_resources = Action::new("gather_resources", 1.0).unwrap();
        gather_resources.preconditions.set("has_tools", true);
        gather_resources.preconditions.set("has_energy", true);
        gather_resources.preconditions.set("has_permission", true);
        gather_resources.preconditions.set("weather_good", true);
        gather_resources.preconditions.set("area_safe", true);
        gather_resources.effects.set("has_wood", true);
        gather_resources.effects.set("has_stone", true);
        gather_resources.effects.set("has_metal", true);
        gather_resources.effects.set("has_weather_protection", true);
        actions.push(gather_resources);

        // Action 2: Process Materials
        let mut process_materials = Action::new("process_materials", 2.0).unwrap();
        process_materials.preconditions.set("has_wood", true);
        process_materials.preconditions.set("has_stone", true);
        process_materials.preconditions.set("has_metal", true);
        process_materials.preconditions.set("has_workshop", true);
        process_materials.preconditions.set("has_skills", true);
        process_materials.effects.set("has_processed_wood", true);
        process_materials.effects.set("has_processed_stone", true);
        process_materials.effects.set("has_processed_metal", true);
        actions.push(process_materials);

        // Action 3: Build Foundation
        let mut build_foundation = Action::new("build_foundation", 3.0).unwrap();
        build_foundation
            .preconditions
            .set("has_processed_stone", true);
        build_foundation
            .preconditions
            .set("has_processed_metal", true);
        build_foundation.preconditions.set("has_blueprint", true);
        build_foundation.preconditions.set("has_equipment", true);
        build_foundation.preconditions.set("has_workers", true);
        build_foundation.effects.set("has_foundation", true);
        build_foundation.effects.set("has_structure_started", true);
        actions.push(build_foundation);

        // Action 4: Build Walls
        let mut build_walls = Action::new("build_walls", 4.0).unwrap();
        build_walls.preconditions.set("has_foundation", true);
        build_walls.preconditions.set("has_processed_wood", true);
        build_walls.preconditions.set("has_processed_stone", true);
        build_walls.preconditions.set("has_structure_started", true);
        build_walls
            .preconditions
            .set("has_weather_protection", true);
        build_walls.effects.set("has_walls", true);
        build_walls.effects.set("has_basic_structure", true);
        actions.push(build_walls);

        // Action 5: Install Roof
        let mut install_roof = Action::new("install_roof", 5.0).unwrap();
        install_roof.preconditions.set("has_basic_structure", true);
        install_roof.preconditions.set("has_processed_wood", true);
        install_roof.preconditions.set("has_processed_metal", true);
        install_roof
            .preconditions
            .set("has_roofing_materials", true);
        install_roof.preconditions.set("has_safety_equipment", true);
        install_roof.effects.set("has_roof", true);
        actions.push(install_roof);

        // Action 6: Install Windows
        let mut install_windows = Action::new("install_windows", 2.0).unwrap();
        install_windows.preconditions.set("has_walls", true);
        install_windows.preconditions.set("has_roof", true);
        install_windows.preconditions.set("has_windows", true);
        install_windows.preconditions.set("has_tools", true);
        install_windows
            .preconditions
            .set("has_weather_protection", true);
        install_windows.effects.set("has_installed_windows", true);
        install_windows.effects.set("has_natural_light", true);
        actions.push(install_windows);

        // Action 7: Install Doors
        let mut install_doors = Action::new("install_doors", 2.0).unwrap();
        install_doors.preconditions.set("has_walls", true);
        install_doors.preconditions.set("has_roof", true);
        install_doors.preconditions.set("has_doors", true);
        install_doors.preconditions.set("has_tools", true);
        install_doors
            .preconditions
            .set("has_weather_protection", true);
        install_doors.effects.set("has_installed_doors", true);
        install_doors.effects.set("has_access_points", true);
        actions.push(install_doors);

        // Action 8: Install Utilities
        let mut install_utilities = Action::new("install_utilities", 6.0).unwrap();
        install_utilities
            .preconditions
            .set("has_basic_structure", true);
        install_utilities.preconditions.set("has_roof", true);
        install_utilities
            .preconditions
            .set("has_utility_plans", true);
        install_utilities
            .preconditions
            .set("has_utility_materials", true);
        install_utilities
            .preconditions
            .set("has_certified_electrician", true);
        install_utilities.effects.set("has_electricity", true);
        install_utilities.effects.set("has_plumbing", true);
        install_utilities.effects.set("has_utilities", true);
        actions.push(install_utilities);

        // Action 9: Interior Finishing
        let mut interior_finishing = Action::new("interior_finishing", 4.0).unwrap();
        interior_finishing.preconditions.set("has_utilities", true);
        interior_finishing
            .preconditions
            .set("has_installed_windows", true);
        interior_finishing
            .preconditions
            .set("has_installed_doors", true);
        interior_finishing
            .preconditions
            .set("has_finishing_materials", true);
        interior_finishing
            .preconditions
            .set("has_interior_design_plan", true);
        interior_finishing
            .effects
            .set("has_finished_interior", true);
        interior_finishing.effects.set("has_livable_space", true);
        actions.push(interior_finishing);

        // Action 10: Final Inspection
        let mut final_inspection = Action::new("final_inspection", 1.0).unwrap();
        final_inspection
            .preconditions
            .set("has_finished_interior", true);
        final_inspection.preconditions.set("has_utilities", true);
        final_inspection
            .preconditions
            .set("has_installed_windows", true);
        final_inspection
            .preconditions
            .set("has_installed_doors", true);
        final_inspection
            .preconditions
            .set("has_safety_checks", true);
        final_inspection.effects.set("has_passed_inspection", true);
        final_inspection.effects.set("has_completed_house", true);
        actions.push(final_inspection);

        // Create the planner with all actions
        let planner = Planner::new(actions.clone());

        // Define the initial state with all required starting conditions
        let mut current_state = State::new();
        current_state.set("has_tools", true);
        current_state.set("has_energy", true);
        current_state.set("has_permission", true);
        current_state.set("weather_good", true);
        current_state.set("area_safe", true);
        current_state.set("has_workshop", true);
        current_state.set("has_skills", true);
        current_state.set("has_blueprint", true);
        current_state.set("has_equipment", true);
        current_state.set("has_workers", true);
        current_state.set("has_roofing_materials", true);
        current_state.set("has_safety_equipment", true);
        current_state.set("has_windows", true);
        current_state.set("has_doors", true);
        current_state.set("has_utility_plans", true);
        current_state.set("has_utility_materials", true);
        current_state.set("has_certified_electrician", true);
        current_state.set("has_finishing_materials", true);
        current_state.set("has_interior_design_plan", true);
        current_state.set("has_safety_checks", true);

        // Define the goal state
        let mut goal_state = State::new();
        goal_state.set("has_completed_house", true);

        // Find a plan
        let plan = planner.plan(&current_state, &goal_state).unwrap();

        // Verify the plan
        assert!(!plan.is_empty());
        assert_eq!(plan.last().unwrap().name, "final_inspection");

        // Verify that the plan contains all necessary steps in the correct order
        let plan_names: Vec<_> = plan.iter().map(|a| a.name.as_str()).collect();
        assert!(plan_names.contains(&"gather_resources"));
        assert!(plan_names.contains(&"process_materials"));
        assert!(plan_names.contains(&"build_foundation"));
        assert!(plan_names.contains(&"build_walls"));
        assert!(plan_names.contains(&"install_roof"));
        assert!(plan_names.contains(&"install_windows"));
        assert!(plan_names.contains(&"install_doors"));
        assert!(plan_names.contains(&"install_utilities"));
        assert!(plan_names.contains(&"interior_finishing"));
        assert!(plan_names.contains(&"final_inspection"));

        // Verify that the plan respects dependencies
        let mut state = current_state.clone();
        for action in &plan {
            // Verify all preconditions are met
            for (key, value) in action.preconditions.values() {
                assert_eq!(
                    state.get(key),
                    Some(*value),
                    "Precondition not met for action {}: {} should be {}",
                    action.name,
                    key,
                    value
                );
            }
            // Apply effects
            for (key, value) in action.effects.values() {
                state.set(key, *value);
            }
        }
        // Verify final state matches goal
        for (key, value) in goal_state.values() {
            assert_eq!(
                state.get(key),
                Some(*value),
                "Goal not achieved: {} should be {}",
                key,
                value
            );
        }
    }
}
