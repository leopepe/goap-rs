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
/// # Examples
///
/// ```
/// use goaprs::{Action, GoapError, SearchAlgorithm, State};
///
/// struct SimpleSearchAlgorithm;
///
/// impl SearchAlgorithm for SimpleSearchAlgorithm {
///     fn search(
///         &self,
///         actions: &[Action],
///         current_state: &State,
///         goal_state: &State,
///     ) -> Result<Vec<Action>, GoapError> {
///         // Simple implementation would go here
///         // This example just returns the first action that satisfies the goal
///         for action in actions {
///             if action.can_perform(current_state) {
///                 let mut new_state = current_state.clone();
///                 action.apply_effects(&mut new_state);
///                 if new_state.satisfies(goal_state) {
///                     return Ok(vec![action.clone()]);
///                 }
///             }
///         }
///         Err(GoapError::NoPlanFound)
///     }
/// }
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
pub trait HeuristicStrategy: Send + Sync {
    /// Calculates a heuristic value from a state to the goal state.
    fn calculate(&self, state: &State, goal: &State) -> f32;
}

/// Default heuristic that counts the number of unsatisfied conditions.
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
pub struct AStarSearch {
    heuristic: Box<dyn HeuristicStrategy>,
}

impl AStarSearch {
    /// Creates a new A* search with the given heuristic.
    pub fn new(heuristic: Box<dyn HeuristicStrategy>) -> Self {
        Self { heuristic }
    }

    /// Creates a new A* search with the default heuristic.
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
