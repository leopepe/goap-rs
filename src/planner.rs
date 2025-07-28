//! # Planner Module for Goal-Oriented Action Planning (GOAP)
//!
//! The planner is the central component of a GOAP system, responsible for:
//! - Finding optimal sequences of actions to achieve goals
//! - Coordinating between world states and available actions
//! - Using search algorithms to determine the best plan
//!
//! ## Overview
//!
//! Goal-Oriented Action Planning is a decision-making system that:
//! 1. Starts with a current world state
//! 2. Defines a goal state to achieve
//! 3. Considers all available actions with their preconditions and effects
//! 4. Finds the optimal sequence of actions to transform the current state into the goal state
//!
//! The `Planner` brings these components together by using search algorithms (like A*) to
//! find efficient paths through the action space.
//!
//! ## Basic Usage
//!
//! ```
//! use goaprs::{Action, Planner, State};
//!
//! // Step 1: Create actions that define what your agent can do
//! let mut chop_tree = Action::new("chop_tree", 2.0).unwrap();
//! chop_tree.preconditions.set("has_axe", true);
//! chop_tree.effects.set("has_wood", true);
//!
//! let mut craft_axe = Action::new("craft_axe", 1.0).unwrap();
//! craft_axe.preconditions.set("has_metal", true);
//! craft_axe.effects.set("has_axe", true);
//!
//! let mut mine_ore = Action::new("mine_ore", 3.0).unwrap();
//! mine_ore.preconditions.set("has_pickaxe", true);
//! mine_ore.effects.set("has_metal", true);
//!
//! let mut craft_pickaxe = Action::new("craft_pickaxe", 1.0).unwrap();
//! craft_pickaxe.preconditions.set("has_wood", true);
//! craft_pickaxe.effects.set("has_pickaxe", true);
//!
//! // Step 2: Create a planner with all available actions
//! let actions = vec![chop_tree, craft_axe, mine_ore, craft_pickaxe];
//! let planner = Planner::new(actions);
//!
//! // Step 3: Define current state
//! let mut current_state = State::new();
//! current_state.set("has_wood", true); // We start with some wood
//!
//! // Step 4: Define goal state
//! let mut goal_state = State::new();
//! goal_state.set("has_metal", true); // We want to have metal
//!
//! // Step 5: Generate a plan
//! let plan = planner.plan(&current_state, &goal_state).unwrap();
//!
//! // The planner will find: craft_pickaxe -> mine_ore
//! for action in &plan {
//!     println!("Action: {}", action.name);
//! }
//! ```

use crate::search::{AStarSearch, SearchAlgorithm};
use crate::{Action, Result, State};

/// The GOAP planner that finds the optimal sequence of actions to achieve a goal state.
///
/// The planner takes a set of available actions and uses a search algorithm
/// (by default A*) to find the most efficient sequence of actions that will
/// transform the current state into the goal state.

///
/// # Examples
///
/// ```
/// use goaprs::{Action, Planner, State};
///
/// // Create available actions
/// let mut goto_store = Action::new("go_to_store", 1.0).unwrap();
/// goto_store.preconditions.set("at_home", true);
/// goto_store.effects.set("at_store", true);
/// goto_store.effects.set("at_home", false);
///
/// let mut buy_food = Action::new("buy_food", 2.0).unwrap();
/// buy_food.preconditions.set("at_store", true);
/// buy_food.preconditions.set("has_money", true);
/// buy_food.effects.set("has_food", true);
///
/// // Create a planner with these actions
/// let planner = Planner::new(vec![goto_store, buy_food]);
///
/// // Define current state
/// let mut current_state = State::new();
/// current_state.set("at_home", true);
/// current_state.set("has_money", true);
///
/// // Define goal state
/// let mut goal_state = State::new();
/// goal_state.set("has_food", true);
///
/// // Find plan to achieve goal
/// let plan = planner.plan(&current_state, &goal_state).unwrap();
///
/// // The plan should include both actions in the correct order
/// assert_eq!(plan.len(), 2);
/// assert_eq!(plan[0].name, "go_to_store");
/// assert_eq!(plan[1].name, "buy_food");
/// ```
/// The core planning component in a GOAP system.
///
/// A `Planner` is responsible for finding an optimal sequence of actions that will
/// transform a current world state into a desired goal state. It maintains a collection
/// of all available actions and uses a search algorithm to find the most efficient plan.
///
/// The planner represents the "brain" of a GOAP agent, making decisions about which actions
/// to take and in what order to achieve goals most efficiently.
///
/// # Architecture
///
/// The `Planner` works with several other components:
/// - `Action`: Represents things the agent can do, with preconditions and effects
/// - `State`: Represents world states (current state and goal state)
/// - `SearchAlgorithm`: Finds optimal paths through the action space (A* by default)
///
/// # Performance Considerations
///
/// - The efficiency of planning depends on the number of available actions
/// - More complex action interdependencies can lead to longer planning times
/// - Using a good heuristic with A* search can dramatically improve performance
///
/// # Examples
///
/// Basic usage with default A* search:
///
/// ```
/// use goaprs::{Action, Planner, State};
///
/// // Define actions for a cooking agent
/// let mut get_ingredients = Action::new("get_ingredients", 1.0).unwrap();
/// get_ingredients.preconditions.set("has_money", true);
/// get_ingredients.effects.set("has_ingredients", true);
///
/// let mut cook_meal = Action::new("cook_meal", 2.0).unwrap();
/// cook_meal.preconditions.set("has_ingredients", true);
/// cook_meal.effects.set("has_meal", true);
///
/// // Create a planner with these actions
/// let planner = Planner::new(vec![get_ingredients, cook_meal]);
///
/// // Define current state - we have money but no ingredients
/// let mut current_state = State::new();
/// current_state.set("has_money", true);
///
/// // Define goal state - we want a meal
/// let mut goal_state = State::new();
/// goal_state.set("has_meal", true);
///
/// // Generate a plan
/// let plan = planner.plan(&current_state, &goal_state).unwrap();
///
/// // The plan will be: get_ingredients -> cook_meal
/// assert_eq!(plan.len(), 2);
/// assert_eq!(plan[0].name, "get_ingredients");
/// assert_eq!(plan[1].name, "cook_meal");
/// ```
pub struct Planner {
    /// Available actions that can be used in planning
    actions: Vec<Action>,
    /// The algorithm used to search for an optimal plan
    search_algorithm: Box<dyn SearchAlgorithm>,
}

impl Planner {
    /// Creates a new planner with the given actions using A* search algorithm.
    ///
    /// This constructor initializes a planner with the provided actions and the default
    /// A* search algorithm, which is a good general-purpose choice for GOAP planning.
    /// The A* algorithm efficiently finds optimal plans by using a heuristic to guide
    /// the search process.
    ///
    /// # Arguments
    ///
    /// * `actions` - A vector of available actions to use for planning
    ///
    /// # Returns
    ///
    /// A new `Planner` instance configured with the given actions and default A* search
    ///
    /// # Examples
    ///
    /// Basic example with a single action:
    ///
    /// ```
    /// use goaprs::{Action, Planner};
    ///
    /// // Create an action for moving to a destination
    /// let mut move_action = Action::new("move", 1.0).unwrap();
    /// move_action.preconditions.set("can_move", true);
    /// move_action.effects.set("at_destination", true);
    ///
    /// // Create a planner with this action
    /// let planner = Planner::new(vec![move_action]);
    /// ```
    ///
    /// Example with multiple interdependent actions:
    ///
    /// ```
    /// use goaprs::{Action, Planner, State};
    ///
    /// // Create actions for a farming scenario
    /// let mut till_soil = Action::new("till_soil", 2.0).unwrap();
    /// till_soil.preconditions.set("has_hoe", true);
    /// till_soil.effects.set("soil_prepared", true);
    ///
    /// let mut plant_seeds = Action::new("plant_seeds", 1.0).unwrap();
    /// plant_seeds.preconditions.set("soil_prepared", true);
    /// plant_seeds.preconditions.set("has_seeds", true);
    /// plant_seeds.effects.set("plants_growing", true);
    ///
    /// let mut harvest_crops = Action::new("harvest_crops", 3.0).unwrap();
    /// harvest_crops.preconditions.set("plants_growing", true);
    /// harvest_crops.effects.set("has_food", true);
    ///
    /// // Create a planner with these farming actions
    /// let farming_planner = Planner::new(vec![till_soil, plant_seeds, harvest_crops]);
    ///
    /// // Now this planner can be used to generate farming plans
    /// ```
    pub fn new(actions: Vec<Action>) -> Self {
        Self {
            actions,
            search_algorithm: Box::new(AStarSearch::default()),
        }
    }

    /// Creates a new planner with the given actions and a custom search algorithm.
    ///
    /// This constructor gives you more control over how plans are generated by allowing
    /// you to specify a custom search algorithm. Different algorithms may be better
    /// suited for particular planning scenarios:
    ///
    /// - `AStarSearch`: Good general-purpose algorithm that uses a heuristic to find optimal plans efficiently
    /// - `DijkstraSearch`: Finds optimal plans without using a heuristic, which may be more thorough but slower
    /// - Custom algorithms: You can implement your own search algorithms for specialized needs
    ///
    /// # Arguments
    ///
    /// * `actions` - A vector of available actions to use for planning
    /// * `search_algorithm` - The search algorithm to use for finding plans
    ///
    /// # Returns
    ///
    /// A new `Planner` instance configured with the given actions and search algorithm
    ///
    /// # Examples
    ///
    /// Using Dijkstra's algorithm instead of A*:
    ///
    /// ```
    /// use goaprs::{Action, DijkstraSearch, Planner};
    ///
    /// // Create some actions
    /// let pickup = Action::new("pickup", 1.0).unwrap();
    /// let drop = Action::new("drop", 1.0).unwrap();
    ///
    /// // Create a planner with Dijkstra's algorithm
    /// let actions = vec![pickup, drop];
    /// let planner = Planner::with_search_algorithm(actions, Box::new(DijkstraSearch));
    /// ```
    ///
    /// Using a custom search algorithm:
    ///
    /// ```
    /// use goaprs::{Action, GoapError, Result, SearchAlgorithm, State, Planner};
    ///
    /// // Define a custom search algorithm
    /// struct CustomSearch;
    ///
    /// impl SearchAlgorithm for CustomSearch {
    ///     fn search(
    ///         &self,
    ///         actions: &[Action],
    ///         current_state: &State,
    ///         goal_state: &State,
    ///     ) -> Result<Vec<Action>> {
    ///         // Simple greedy implementation
    ///         let mut plan = Vec::new();
    ///         let mut state = current_state.clone();
    ///
    ///         while !state.satisfies(goal_state) {
    ///             // Find the first applicable action
    ///             if let Some(action) = actions.iter().find(|a| a.can_perform(&state)) {
    ///                 action.apply_effects(&mut state);
    ///                 plan.push(action.clone());
    ///             } else {
    ///                 return Err(GoapError::NoPlanFound);
    ///             }
    ///         }
    ///
    ///         Ok(plan)
    ///     }
    /// }
    ///
    /// // Create planner with custom search
    /// let actions = vec![Action::new("example", 1.0).unwrap()];
    /// let planner = Planner::with_search_algorithm(actions, Box::new(CustomSearch));
    /// ```
    pub fn with_search_algorithm(
        actions: Vec<Action>,
        search_algorithm: Box<dyn SearchAlgorithm + Send + Sync>,
    ) -> Self {
        Self {
            actions,
            search_algorithm,
        }
    }

    /// Finds a plan to achieve the goal state from the current state
    ///
    /// This method uses the planner's search algorithm to find the most efficient
    /// sequence of actions that will transform the current state into the goal state.
    ///
    /// # Arguments
    ///
    /// * `current_state` - The starting state
    /// * `goal_state` - The target state to achieve
    ///
    /// # Returns
    ///
    /// * `Ok(Vec<Action>)` - A sequence of actions that achieves the goal
    /// * `Err(GoapError)` - If no valid plan can be found
    ///
    /// # Errors
    ///
    /// Returns `GoapError::NoPlanFound` if no valid plan can be found to achieve the goal.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::{Action, Planner, State};
    ///
    /// // Create action and planner
    /// let mut action = Action::new("take_action", 1.0).unwrap();
    /// action.effects.set("goal_achieved", true);
    /// let planner = Planner::new(vec![action]);
    ///
    /// // Define states
    /// let current_state = State::new();
    /// let mut goal_state = State::new();
    /// goal_state.set("goal_achieved", true);
    ///
    /// // Find plan
    /// let plan = planner.plan(&current_state, &goal_state).unwrap();
    /// assert_eq!(plan.len(), 1);
    /// ```
    /// Finds a plan to achieve the goal state from the current state.
    ///
    /// This is the main method of the GOAP planner. It uses the configured search algorithm
    /// to find the most efficient sequence of actions that will transform the current state
    /// into the goal state.
    ///
    /// The planning process:
    /// 1. Checks if the current state already satisfies the goal
    /// 2. If not, searches through possible action sequences
    /// 3. Evaluates different paths based on action costs
    /// 4. Returns the optimal plan or an error if no plan is possible
    ///
    /// # Arguments
    ///
    /// * `current_state` - The starting state representing the current world conditions
    /// * `goal_state` - The target state that the agent wants to achieve
    ///
    /// # Returns
    ///
    /// * `Ok(Vec<Action>)` - A sequence of actions that achieves the goal (empty if the goal is already satisfied)
    /// * `Err(GoapError::NoPlanFound)` - If no valid plan can be found to achieve the goal
    ///
    /// # Errors
    ///
    /// Returns `GoapError::NoPlanFound` if no valid plan can be found to achieve the goal.
    /// This can happen if:
    /// - The goal contains conditions that no action can establish
    /// - Actions exist that could achieve the goal, but their preconditions cannot be satisfied
    /// - The action set forms a dependency cycle that prevents reaching the goal
    ///
    /// # Examples
    ///
    /// Finding a simple plan:
    ///
    /// ```
    /// use goaprs::{Action, Planner, State};
    ///
    /// // Create a simple action
    /// let mut action = Action::new("light_fire", 1.0).unwrap();
    /// action.preconditions.set("has_matches", true);
    /// action.effects.set("fire_lit", true);
    ///
    /// // Create planner
    /// let planner = Planner::new(vec![action]);
    ///
    /// // Define states
    /// let mut current_state = State::new();
    /// current_state.set("has_matches", true);
    ///
    /// let mut goal_state = State::new();
    /// goal_state.set("fire_lit", true);
    ///
    /// // Find plan
    /// let plan = planner.plan(&current_state, &goal_state).unwrap();
    /// assert_eq!(plan.len(), 1);
    /// assert_eq!(plan[0].name, "light_fire");
    /// ```
    ///
    /// Handling when no plan is possible:
    ///
    /// ```
    /// use goaprs::{Action, GoapError, Planner, State};
    ///
    /// // Create an action that requires something we don't have
    /// let mut action = Action::new("cook_meal", 1.0).unwrap();
    /// action.preconditions.set("has_ingredients", true);
    /// action.effects.set("has_food", true);
    ///
    /// let planner = Planner::new(vec![action]);
    ///
    /// // We don't have ingredients and no way to get them
    /// let current_state = State::new();
    ///
    /// // We want food
    /// let mut goal_state = State::new();
    /// goal_state.set("has_food", true);
    ///
    /// // No plan should be possible
    /// let result = planner.plan(&current_state, &goal_state);
    /// assert!(matches!(result, Err(GoapError::NoPlanFound)));
    /// ```
    ///
    /// When the goal is already satisfied:
    ///
    /// ```
    /// use goaprs::{Action, Planner, State};
    ///
    /// let planner = Planner::new(vec![]);
    ///
    /// let mut state = State::new();
    /// state.set("goal_met", true);
    ///
    /// let mut goal = State::new();
    /// goal.set("goal_met", true);
    ///
    /// // Plan should be empty since goal is already satisfied
    /// let plan = planner.plan(&state, &goal).unwrap();
    /// assert!(plan.is_empty());
    /// ```
    pub fn plan(&self, current_state: &State, goal_state: &State) -> Result<Vec<Action>> {
        self.search_algorithm
            .search(&self.actions, current_state, goal_state)
    }
}

/// Implementation of Clone for Planner.
///
/// Note that cloning a planner will preserve the actions but will reset the search
/// algorithm to the default A* search, since trait objects cannot be directly cloned.
///
/// # Examples
///
/// ```
/// use goaprs::{Action, DijkstraSearch, Planner};
///
/// // Create a planner with a custom search algorithm
/// let actions = vec![Action::new("example", 1.0).unwrap()];
/// let original = Planner::with_search_algorithm(actions, Box::new(DijkstraSearch));
///
/// // When cloned, it will use the default A* search
/// let cloned = original.clone();
/// ```
impl Clone for Planner {
    fn clone(&self) -> Self {
        // Since we can't clone the boxed trait object directly,
        // we create a new planner with the same actions and a new
        // default search algorithm (AStarSearch)
        Self::new(self.actions.clone())
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
        let astar_planner =
            Planner::with_search_algorithm(actions.clone(), Box::new(AStarSearch::default()));
        let astar_plan = astar_planner.plan(&initial, &goal).unwrap();
        assert_eq!(astar_plan[0].name, "a");

        // Test Dijkstra search
        let dijkstra_planner =
            Planner::with_search_algorithm(actions, Box::new(DijkstraSearch::default()));
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
