//! # Search Module for Goal-Oriented Action Planning (GOAP)
//!
//! This module provides the search algorithms that find sequences of actions to achieve goals.
//! It's the core planning component of the GOAP system, responsible for determining which
//! actions to take and in what order.
//!
//! ## Search Concepts
//!
//! GOAP planning works by searching through the space of possible actions to find
//! a path from the current world state to a goal state. The search module provides:
//!
//! - **Search Algorithms**: Implementations like A* and Dijkstra's algorithm that find optimal plans
//! - **Heuristic Functions**: Guide search algorithms to more efficiently find solutions
//! - **Cost Functions**: Determine which plans are considered "better" than others
//!
//! ## Basic Usage
//!
//! ```
//! use goaprs::{Action, State, AStarSearch, SearchAlgorithm};
//!
//! // Create some actions
//! let mut get_axe = Action::new("get_axe", 1.0).unwrap();
//! get_axe.preconditions.set("at_store", true);
//! get_axe.effects.set("has_axe", true);
//!
//! let mut go_to_store = Action::new("go_to_store", 2.0).unwrap();
//! go_to_store.preconditions.set("at_home", true);
//! go_to_store.effects.set("at_store", true);
//! go_to_store.effects.set("at_home", false);
//!
//! let mut chop_wood = Action::new("chop_wood", 3.0).unwrap();
//! chop_wood.preconditions.set("has_axe", true);
//! chop_wood.effects.set("has_wood", true);
//!
//! // Define initial state
//! let mut initial_state = State::new();
//! initial_state.set("at_home", true);
//!
//! // Define goal state
//! let mut goal_state = State::new();
//! goal_state.set("has_wood", true);
//!
//! // Create a search algorithm
//! let search = AStarSearch::default();
//!
//! // Find a plan
//! let actions = vec![get_axe, go_to_store, chop_wood];
//! let plan = search.search(&actions, &initial_state, &goal_state).unwrap();
//!
//! // The plan will contain the sequence of actions to achieve the goal
//! // [go_to_store, get_axe, chop_wood]
//! for action in &plan {
//!     println!("Action: {}", action.name);
//! }
//! ```
//!
//! ## Available Search Algorithms
//!
//! This module provides two primary search algorithms:
//!
//! 1. **A* Search** (`AStarSearch`): Uses a heuristic to efficiently find optimal plans
//! 2. **Dijkstra's Algorithm** (`DijkstraSearch`): Finds the lowest-cost plan without using a heuristic
//!
//! ## Custom Search Algorithms
//!
//! You can implement your own search algorithms by implementing the `SearchAlgorithm` trait:
//!
//! ```
//! use goaprs::{Action, GoapError, Result, SearchAlgorithm, State};
//!
//! // A greedy search that always picks the cheapest immediate action
//! struct GreedySearch;
//!
//! impl SearchAlgorithm for GreedySearch {
//!     fn search(
//!         &self,
//!         actions: &[Action],
//!         current_state: &State,
//!         goal_state: &State,
//!     ) -> Result<Vec<Action>> {
//!         let mut plan = Vec::new();
//!         let mut state = current_state.clone();
//!
//!         // Keep adding actions until we reach the goal
//!         while !state.satisfies(goal_state) {
//!             // Find the cheapest applicable action
//!             let next_action = actions
//!                 .iter()
//!                 .filter(|a| a.can_perform(&state))
//!                 .min_by(|a, b| a.cost.partial_cmp(&b.cost).unwrap_or(std::cmp::Ordering::Equal));
//!
//!             if let Some(action) = next_action {
//!                 // Apply action and add to plan
//!                 action.apply_effects(&mut state);
//!                 plan.push(action.clone());
//!             } else {
//!                 return Err(GoapError::NoPlanFound);
//!             }
//!         }
//!
//!         Ok(plan)
//!     }
//! }
//! ```
//!
//! ## Custom Heuristics
//!
//! You can also create custom heuristics by implementing the `HeuristicStrategy` trait:
//!
//! ```
//! use goaprs::{HeuristicStrategy, State};
//!
//! // A weighted heuristic that prioritizes certain goals
//! struct WeightedHeuristic {
//!     priority_keys: Vec<String>,
//! }
//!
//! impl HeuristicStrategy for WeightedHeuristic {
//!     fn calculate(&self, state: &State, goal: &State) -> f32 {
//!         goal.values()
//!             .iter()
//!             .map(|(key, &value)| {
//!                 let weight = if self.priority_keys.contains(key) { 2.0 } else { 1.0 };
//!                 if state.get(key) != Some(value) {
//!                     weight
//!                 } else {
//!                     0.0
//!                 }
//!             })
//!             .sum()
//!     }
//! }
//! ```

use crate::{Action, GoapError, Result, State};
use std::cmp::Ordering;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashSet};

/// Trait defining the interface for search algorithms used in GOAP.
///
/// This trait abstracts the specific algorithm used to find a sequence of actions
/// that transforms the current state into the goal state. Implementations include
/// A* search and Dijkstra's algorithm.
///
/// Search algorithms are at the heart of GOAP planning, as they determine the
/// optimal sequence of actions to achieve a goal state from the current state.
///
/// # Examples
///
/// ```
/// use goaprs::{Action, GoapError, SearchAlgorithm, State};
///
/// // A simple search algorithm that builds a plan one step at a time
/// struct SimpleSearchAlgorithm;
///
/// impl SearchAlgorithm for SimpleSearchAlgorithm {
///     fn search(
///         &self,
///         actions: &[Action],
///         current_state: &State,
///         goal_state: &State,
///     ) -> Result<Vec<Action>, GoapError> {
///         // If we already satisfy the goal, no actions needed
///         if current_state.satisfies(goal_state) {
///             return Ok(Vec::new());
///         }
///
///         // Try to find a single action that gets us to the goal
///         for action in actions {
///             if action.can_perform(current_state) {
///                 let mut new_state = current_state.clone();
///                 action.apply_effects(&mut new_state);
///                 if new_state.satisfies(goal_state) {
///                     return Ok(vec![action.clone()]);
///                 }
///             }
///         }
///
///         // For a real implementation, we would need to handle multi-step plans
///         Err(GoapError::NoPlanFound)
///     }
/// }
///
/// // Example of using a search algorithm
/// let mut action1 = Action::new("pickup_axe", 1.0).unwrap();
/// action1.preconditions.set("axe_available", true);
/// action1.effects.set("has_axe", true);
///
/// let mut actions = vec![action1];
///
/// let mut initial = State::new();
/// initial.set("axe_available", true);
///
/// let mut goal = State::new();
/// goal.set("has_axe", true);
///
/// let search = SimpleSearchAlgorithm;
/// let plan = search.search(&actions, &initial, &goal).unwrap();
/// assert_eq!(plan.len(), 1);
/// assert_eq!(plan[0].name, "pickup_axe");
/// ```
pub trait SearchAlgorithm {
    /// Finds a sequence of actions that transforms the current state into the goal state.
    ///
    /// # Arguments
    ///
    /// * `actions` - Available actions to consider in the search
    /// * `current_state` - The initial state of the world
    /// * `goal_state` - The target state to achieve
    ///
    /// # Returns
    ///
    /// * `Ok(Vec<Action>)` - A sequence of actions that achieves the goal
    /// * `Err(GoapError)` - If no valid plan can be found
    fn search(
        &self,
        actions: &[Action],
        current_state: &State,
        goal_state: &State,
    ) -> Result<Vec<Action>>;
}

/// A trait for heuristic functions used in search algorithms.
///
/// Heuristics estimate the cost from a given state to the goal state,
/// helping guide search algorithms toward promising solutions. A good
/// heuristic can dramatically improve search performance.
///
/// Heuristic functions must be:
/// - **Admissible**: Never overestimate the actual cost to the goal
/// - **Consistent**: The estimated cost from state A to the goal is no greater than
///   the cost from A to B plus the estimated cost from B to the goal
///
/// # Examples
///
/// ```
/// use goaprs::{HeuristicStrategy, State};
///
/// // A custom heuristic that counts missing keys in the goal state
/// struct MissingKeysHeuristic;
///
/// impl HeuristicStrategy for MissingKeysHeuristic {
///     fn calculate(&self, state: &State, goal: &State) -> f32 {
///         // Count how many keys from the goal are missing in the current state
///         goal.values()
///             .keys()
///             .filter(|key| !state.values().contains_key(*key))
///             .count() as f32
///     }
/// }
///
/// // Usage with states
/// let mut current = State::new();
/// current.set("has_axe", true);
///
/// let mut goal = State::new();
/// goal.set("has_axe", true);
/// goal.set("has_wood", true);
/// goal.set("at_camp", true);
///
/// let heuristic = MissingKeysHeuristic;
/// let estimate = heuristic.calculate(&current, &goal);
/// assert_eq!(estimate, 2.0); // Two keys missing: "has_wood" and "at_camp"
/// ```
pub trait HeuristicStrategy: Send + Sync {
    /// Calculates a heuristic value from a state to the goal state.
    ///
    /// This method estimates the cost or distance from the current state to the goal state.
    /// Lower values indicate states that are closer to the goal.
    ///
    /// # Arguments
    ///
    /// * `state` - The current state to evaluate
    /// * `goal` - The goal state we're trying to reach
    ///
    /// # Returns
    ///
    /// A float representing the estimated cost to reach the goal from this state
    fn calculate(&self, state: &State, goal: &State) -> f32;
}

/// Default heuristic that counts the number of unsatisfied conditions.
///
/// This heuristic simply counts how many key-value pairs in the goal state
/// differ from the current state. It's simple but effective for many GOAP
/// scenarios where each action typically changes one state condition.
///
/// # Examples
///
/// ```
/// use goaprs::{DefaultHeuristic, HeuristicStrategy, State};
///
/// let mut current = State::new();
/// current.set("has_axe", true);
/// current.set("at_forest", false);
///
/// let mut goal = State::new();
/// goal.set("has_axe", true);
/// goal.set("at_forest", true);
/// goal.set("has_wood", true);
///
/// let heuristic = DefaultHeuristic;
/// let distance = heuristic.calculate(&current, &goal);
/// assert_eq!(distance, 2.0); // Two unsatisfied conditions: at_forest and has_wood
/// ```
pub struct DefaultHeuristic;

impl HeuristicStrategy for DefaultHeuristic {
    fn calculate(&self, state: &State, goal: &State) -> f32 {
        // Count the number of unsatisfied conditions
        goal.values()
            .iter()
            .filter(|(key, &value)| state.get(key) != Some(value))
            .count() as f32
    }
}

/// Zero heuristic for algorithms like Dijkstra that don't use heuristics.
///
/// This heuristic always returns 0, which effectively turns A* search into
/// Dijkstra's algorithm. It's useful when you want to find the optimal path
/// based solely on action costs without any heuristic guidance.
///
/// # Examples
///
/// ```
/// use goaprs::{ZeroHeuristic, HeuristicStrategy, State};
///
/// let mut state1 = State::new();
/// state1.set("position", true);
///
/// let mut state2 = State::new();
/// state2.set("position", false);
///
/// let heuristic = ZeroHeuristic;
/// let estimate = heuristic.calculate(&state1, &state2);
/// assert_eq!(estimate, 0.0); // Always returns zero
/// ```
pub struct ZeroHeuristic;

impl HeuristicStrategy for ZeroHeuristic {
    fn calculate(&self, _state: &State, _goal: &State) -> f32 {
        0.0
    }
}

/// Represents a node in the search space.
#[derive(Debug, Clone)]
struct Node {
    /// The state at this node
    state: State,
    /// Reference to parent node
    parent: Option<usize>,
    /// Action that led to this state (from parent)
    action: Option<Action>,
    /// Path cost from start to this node
    g_cost: f32,
    /// Estimated cost from this node to goal
    h_cost: f32,
}

impl Node {
    /// Creates a new node.
    fn new(
        state: State,
        parent: Option<usize>,
        action: Option<Action>,
        g_cost: f32,
        h_cost: f32,
    ) -> Self {
        Self {
            state,
            parent,
            action,
            g_cost,
            h_cost,
        }
    }

    /// Total estimated cost (f = g + h).
    fn f_cost(&self) -> f32 {
        self.g_cost + self.h_cost
    }
}

/// A wrapper for Node that implements proper ordering for the priority queue.
#[derive(Debug, Clone)]
struct NodeWrapper {
    /// Index of the node in the node list
    idx: usize,
    /// Reference to the node for cost comparison
    f_cost: f32,
    /// Path cost for tie-breaking
    g_cost: f32,
}

impl PartialEq for NodeWrapper {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost == other.f_cost && self.g_cost == other.g_cost
    }
}

impl Eq for NodeWrapper {}

impl PartialOrd for NodeWrapper {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // First compare by f_cost
        let f_cmp = self.f_cost.partial_cmp(&other.f_cost);
        if f_cmp != Some(Ordering::Equal) {
            return f_cmp;
        }
        // If f_costs are equal, prefer paths that are further along (higher g_cost)
        self.g_cost.partial_cmp(&other.g_cost)
    }
}

impl Ord for NodeWrapper {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

/// Manages the state of a graph search.
struct SearchContext {
    /// List of all nodes created during the search
    nodes: Vec<Node>,
    /// Priority queue of nodes to explore (contains indices to nodes)
    open_set: BinaryHeap<Reverse<NodeWrapper>>,
    /// Set of indices of nodes that have been explored
    closed_set: HashSet<usize>,
}

impl SearchContext {
    /// Creates a new search context with the initial state.
    fn new(initial_state: &State, goal_state: &State, heuristic: &dyn HeuristicStrategy) -> Self {
        let mut nodes = Vec::new();
        let mut open_set = BinaryHeap::new();

        // Create initial node
        let initial_node = Node::new(
            initial_state.clone(),
            None,
            None,
            0.0,
            heuristic.calculate(initial_state, goal_state),
        );

        nodes.push(initial_node);

        // Add initial node to open set
        open_set.push(Reverse(NodeWrapper {
            idx: 0,
            f_cost: nodes[0].f_cost(),
            g_cost: 0.0,
        }));

        Self {
            nodes,
            open_set,
            closed_set: HashSet::new(),
        }
    }

    /// Gets the next node to explore from the open set.
    fn next_node(&mut self) -> Option<usize> {
        // Keep popping until we find a node that's not in the closed set
        while let Some(Reverse(node_wrapper)) = self.open_set.pop() {
            if !self.closed_set.contains(&node_wrapper.idx) {
                return Some(node_wrapper.idx);
            }
        }
        None
    }

    /// Marks a node as visited (adds to closed set).
    fn mark_visited(&mut self, node_idx: usize) {
        self.closed_set.insert(node_idx);
    }

    /// Gets applicable actions from the available actions.
    fn applicable_actions<'a>(&self, actions: &'a [Action], state: &State) -> Vec<&'a Action> {
        let mut applicable = actions
            .iter()
            .filter(|a| a.can_perform(state))
            .collect::<Vec<_>>();

        // Sort by cost to try cheaper actions first
        applicable.sort_by(|a, b| a.cost.partial_cmp(&b.cost).unwrap_or(Ordering::Equal));
        applicable
    }

    /// Generate successor node.
    fn generate_successor(
        &mut self,
        parent_idx: usize,
        action: &Action,
        heuristic: &dyn HeuristicStrategy,
        goal_state: &State,
    ) -> usize {
        let parent = &self.nodes[parent_idx];

        // Apply action to generate new state
        let mut new_state = parent.state.clone();
        action.apply_effects(&mut new_state);

        // Calculate new costs
        let new_g_cost = parent.g_cost + action.cost;
        let new_h_cost = heuristic.calculate(&new_state, goal_state);

        // Create new node
        let new_node = Node::new(
            new_state,
            Some(parent_idx),
            Some(action.clone()),
            new_g_cost,
            new_h_cost,
        );

        let new_idx = self.nodes.len();
        self.nodes.push(new_node);
        new_idx
    }

    /// Processes a successor node, adding it to the open set if appropriate.
    // Process successor
    fn process_successor(&mut self, node_idx: usize) -> bool {
        let node = &self.nodes[node_idx];

        // Check if we've already found a better path to this state
        if let Some(existing_idx) = self.find_node_with_state(&node.state) {
            if existing_idx != node_idx && self.nodes[existing_idx].g_cost <= node.g_cost {
                return false;
            }
        }

        // Add to open set
        self.open_set.push(Reverse(NodeWrapper {
            idx: node_idx,
            f_cost: node.f_cost(),
            g_cost: node.g_cost,
        }));

        true
    }

    /// Finds a node with the same state, if one exists.
    fn find_node_with_state(&self, state: &State) -> Option<usize> {
        self.nodes.iter().position(|n| &n.state == state)
    }

    /// Reconstructs the path from the initial state to the given node.
    fn reconstruct_path(&self, node_idx: usize) -> Vec<Action> {
        let mut path = Vec::new();
        let mut current_idx = node_idx;

        while let Some(node) = self.nodes.get(current_idx) {
            if let Some(action) = &node.action {
                path.push(action.clone());
            }

            if let Some(parent_idx) = node.parent {
                current_idx = parent_idx;
            } else {
                break;
            }
        }

        path.reverse();
        path
    }
}

/// A* search algorithm implementation.
///
/// A* is an informed search algorithm that finds the least-cost path from the initial state
/// to the goal state. It uses a heuristic function to guide the search toward promising paths,
/// which makes it more efficient than uninformed search algorithms like Dijkstra's.
///
/// A* guarantees an optimal solution when the heuristic is admissible (never overestimates
/// the actual cost to the goal).
///
/// # Examples
///
/// ```
/// use goaprs::{Action, AStarSearch, SearchAlgorithm, State};
///
/// // Create our actions
/// let mut move_to_forest = Action::new("move_to_forest", 2.0).unwrap();
/// move_to_forest.preconditions.set("at_home", true);
/// move_to_forest.effects.set("at_forest", true);
/// move_to_forest.effects.set("at_home", false);
///
/// let mut collect_wood = Action::new("collect_wood", 3.0).unwrap();
/// collect_wood.preconditions.set("at_forest", true);
/// collect_wood.effects.set("has_wood", true);
///
/// let mut move_to_home = Action::new("move_to_home", 2.0).unwrap();
/// move_to_home.preconditions.set("at_forest", true);
/// move_to_home.effects.set("at_home", true);
/// move_to_home.effects.set("at_forest", false);
///
/// // Create our starting state
/// let mut initial = State::new();
/// initial.set("at_home", true);
/// initial.set("has_wood", false);
///
/// // Define our goal
/// let mut goal = State::new();
/// goal.set("at_home", true);
/// goal.set("has_wood", true);
///
/// // Use A* to find a plan
/// let astar = AStarSearch::default();
/// let actions = vec![move_to_forest, collect_wood, move_to_home];
/// let plan = astar.search(&actions, &initial, &goal).unwrap();
///
/// // The plan should involve going to the forest, collecting wood, and returning home
/// assert_eq!(plan.len(), 3);
/// assert_eq!(plan[0].name, "move_to_forest");
/// assert_eq!(plan[1].name, "collect_wood");
/// assert_eq!(plan[2].name, "move_to_home");
/// ```
pub struct AStarSearch {
    heuristic: Box<dyn HeuristicStrategy>,
}

impl AStarSearch {
    /// Creates a new A* search with the given heuristic.
    ///
    /// This constructor allows you to specify a custom heuristic function
    /// to guide the A* search algorithm.
    ///
    /// # Arguments
    ///
    /// * `heuristic` - A boxed implementation of the `HeuristicStrategy` trait
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::{AStarSearch, DefaultHeuristic, HeuristicStrategy, State};
    ///
    /// // Create a custom heuristic
    /// struct CustomHeuristic;
    /// impl HeuristicStrategy for CustomHeuristic {
    ///     fn calculate(&self, state: &State, goal: &State) -> f32 {
    ///         // Simple implementation for example
    ///         goal.values().len() as f32
    ///     }
    /// }
    ///
    /// // Create A* with the custom heuristic
    /// let astar = AStarSearch::new(Box::new(CustomHeuristic));
    /// ```
    pub fn new(heuristic: Box<dyn HeuristicStrategy>) -> Self {
        Self { heuristic }
    }

    /// Creates a new A* search with the default heuristic.
    ///
    /// Uses the `DefaultHeuristic` which counts the number of unsatisfied
    /// conditions in the goal state. This is a good general-purpose heuristic
    /// for many GOAP scenarios.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::AStarSearch;
    ///
    /// // Create A* with the default heuristic
    /// let astar = AStarSearch::with_default_heuristic();
    /// ```
    pub fn with_default_heuristic() -> Self {
        Self {
            heuristic: Box::new(DefaultHeuristic),
        }
    }
}

impl Default for AStarSearch {
    fn default() -> Self {
        Self::with_default_heuristic()
    }
}

impl SearchAlgorithm for AStarSearch {
    fn search(
        &self,
        actions: &[Action],
        current_state: &State,
        goal_state: &State,
    ) -> Result<Vec<Action>> {
        // First, check if the goal is already satisfied by the current state
        if current_state.satisfies(goal_state) {
            return Ok(Vec::new()); // Goal already achieved
        }

        // Create search context
        let mut context = SearchContext::new(current_state, goal_state, self.heuristic.as_ref());

        // Main search loop
        while let Some(current_idx) = context.next_node() {
            let node = &context.nodes[current_idx].clone();

            // Check if we've reached the goal
            if goal_state
                .values()
                .iter()
                .all(|(k, v)| node.state.get(k) == Some(*v))
            {
                return Ok(context.reconstruct_path(current_idx));
            }

            // Mark as visited
            context.mark_visited(current_idx);

            // Get applicable actions and generate successors
            for action in context.applicable_actions(actions, &node.state) {
                let successor_idx = context.generate_successor(
                    current_idx,
                    action,
                    self.heuristic.as_ref(),
                    goal_state,
                );

                context.process_successor(successor_idx);
            }
        }

        Err(GoapError::NoPlanFound)
    }
}

/// Dijkstra's algorithm implementation.
///
/// Dijkstra's algorithm is an uninformed search algorithm that finds the lowest-cost
/// path from the initial state to the goal state. Unlike A*, it doesn't use a heuristic
/// to guide the search, so it explores states based solely on their path cost.
///
/// Dijkstra's algorithm is equivalent to A* with a zero heuristic. It guarantees
/// an optimal solution but may explore more states than A* with a good heuristic.
///
/// # Examples
///
/// ```
/// use goaprs::{Action, DijkstraSearch, SearchAlgorithm, State};
///
/// // Create actions for a simple GOAP scenario
/// let mut pickup_wood = Action::new("pickup_wood", 1.0).unwrap();
/// pickup_wood.preconditions.set("near_wood", true);
/// pickup_wood.effects.set("has_wood", true);
///
/// let mut go_to_wood = Action::new("go_to_wood", 2.0).unwrap();
/// go_to_wood.preconditions.set("at_home", true);
/// go_to_wood.effects.set("near_wood", true);
/// go_to_wood.effects.set("at_home", false);
///
/// // Initial state - we're at home
/// let mut initial = State::new();
/// initial.set("at_home", true);
///
/// // Goal state - we want wood
/// let mut goal = State::new();
/// goal.set("has_wood", true);
///
/// // Use Dijkstra to find a plan
/// let dijkstra = DijkstraSearch::default();
/// let actions = vec![go_to_wood, pickup_wood];
/// let plan = dijkstra.search(&actions, &initial, &goal).unwrap();
///
/// // The plan should be: go to the wood, then pick it up
/// assert_eq!(plan.len(), 2);
/// assert_eq!(plan[0].name, "go_to_wood");
/// assert_eq!(plan[1].name, "pickup_wood");
/// ```
pub struct DijkstraSearch;

impl Default for DijkstraSearch {
    fn default() -> Self {
        Self
    }
}

impl SearchAlgorithm for DijkstraSearch {
    fn search(
        &self,
        actions: &[Action],
        current_state: &State,
        goal_state: &State,
    ) -> Result<Vec<Action>> {
        // Dijkstra is A* with a zero heuristic
        let astar = AStarSearch::new(Box::new(ZeroHeuristic));
        astar.search(actions, current_state, goal_state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
    fn test_astar_search() {
        let a = make_action("a", 1.0, vec![("start", true)], vec![("goal", true)]);
        let b = make_action("b", 5.0, vec![("start", true)], vec![("goal", true)]);
        let actions = vec![a.clone(), b.clone()];

        let mut initial = State::new();
        initial.set("start", true);

        let mut goal = State::new();
        goal.set("goal", true);

        let astar = AStarSearch::default();
        let plan = astar.search(&actions, &initial, &goal).unwrap();
        assert_eq!(plan.len(), 1);
        assert_eq!(plan[0].name, "a");
    }

    #[test]
    fn test_dijkstra_search() {
        let a = make_action("a", 1.0, vec![("start", true)], vec![("goal", true)]);
        let b = make_action("b", 5.0, vec![("start", true)], vec![("goal", true)]);
        let actions = vec![a.clone(), b.clone()];

        let mut initial = State::new();
        initial.set("start", true);

        let mut goal = State::new();
        goal.set("goal", true);

        let dijkstra = DijkstraSearch::default();
        let plan = dijkstra.search(&actions, &initial, &goal).unwrap();
        assert_eq!(plan.len(), 1);
        assert_eq!(plan[0].name, "a");
    }

    #[test]
    fn test_multi_step_plan() {
        // Create actions for a multi-step problem
        let action1 = make_action(
            "action1",
            1.0,
            vec![("condition1", true)],
            vec![("condition2", true)],
        );
        let action2 = make_action(
            "action2",
            1.0,
            vec![("condition2", true)],
            vec![("goal", true)],
        );

        let actions = vec![action1.clone(), action2.clone()];

        // Set up initial and goal states
        let mut initial = State::new();
        initial.set("condition1", true);

        let mut goal = State::new();
        goal.set("goal", true);

        // Test A* search
        let astar = AStarSearch::default();
        let plan = astar.search(&actions, &initial, &goal).unwrap();
        assert_eq!(plan.len(), 2);
        assert_eq!(plan[0].name, "action1");
        assert_eq!(plan[1].name, "action2");

        // Test Dijkstra search
        let dijkstra = DijkstraSearch::default();
        let plan = dijkstra.search(&actions, &initial, &goal).unwrap();
        assert_eq!(plan.len(), 2);
        assert_eq!(plan[0].name, "action1");
        assert_eq!(plan[1].name, "action2");
    }
}
