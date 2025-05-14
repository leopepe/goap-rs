use crate::{Action, GoapError, Result, State};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};

/// Trait defining the interface for search algorithms
pub trait SearchAlgorithm {
    fn search(
        &self,
        actions: &[Action],
        current_state: &State,
        goal_state: &State,
    ) -> Result<Vec<Action>>;
}

/// A* search algorithm implementation
pub struct AStarSearch;

impl SearchAlgorithm for AStarSearch {
    fn search(
        &self,
        actions: &[Action],
        current_state: &State,
        goal_state: &State,
    ) -> Result<Vec<Action>> {
        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashSet::new();
        let mut nodes = Vec::new();

        // Add the initial node
        let initial_node = Node {
            state: current_state.clone(),
            parent: None,
            action: None,
            g_cost: 0.0,
            h_cost: heuristic(current_state, goal_state),
        };

        nodes.push(initial_node);
        open_set.push(0);

        while let Some(node_idx) = open_set.pop() {
            let node = nodes[node_idx].clone();

            if node.state.satisfies(goal_state) {
                return Ok(reconstruct_path(&nodes, node_idx));
            }

            closed_set.insert(node_idx);

            // Sort actions by cost to ensure we try cheaper actions first
            let mut sorted_actions: Vec<_> = actions.iter().collect();
            sorted_actions.sort_by(|a, b| a.cost.partial_cmp(&b.cost).unwrap_or(Ordering::Equal));

            for action in sorted_actions {
                if !action.can_perform(&node.state) {
                    continue;
                }

                let mut new_state = node.state.clone();
                action.apply_effects(&mut new_state);

                let new_g_cost = node.g_cost + action.cost;
                let new_h_cost = heuristic(&new_state, goal_state);

                // Check if we've already found a better path to this state
                if let Some(existing_idx) = nodes.iter().position(|n| n.state == new_state) {
                    if nodes[existing_idx].g_cost <= new_g_cost {
                        continue;
                    }
                }

                let new_node = Node {
                    state: new_state,
                    parent: Some(node_idx),
                    action: Some(action.clone()),
                    g_cost: new_g_cost,
                    h_cost: new_h_cost,
                };

                let new_node_idx = nodes.len();
                nodes.push(new_node);
                open_set.push(new_node_idx);
            }
        }

        Err(GoapError::NoPlanFound)
    }
}

/// Dijkstra's algorithm implementation
pub struct DijkstraSearch;

impl SearchAlgorithm for DijkstraSearch {
    fn search(
        &self,
        actions: &[Action],
        current_state: &State,
        goal_state: &State,
    ) -> Result<Vec<Action>> {
        let mut open_set = BinaryHeap::new();
        let mut closed_set = HashSet::new();
        let mut nodes = Vec::new();

        // Add the initial node
        let initial_node = Node {
            state: current_state.clone(),
            parent: None,
            action: None,
            g_cost: 0.0,
            h_cost: 0.0, // Dijkstra doesn't use heuristic
        };

        nodes.push(initial_node);
        open_set.push(0);

        while let Some(node_idx) = open_set.pop() {
            let node = nodes[node_idx].clone();

            if node.state.satisfies(goal_state) {
                return Ok(reconstruct_path(&nodes, node_idx));
            }

            closed_set.insert(node_idx);

            // Sort actions by cost to ensure we try cheaper actions first
            let mut sorted_actions: Vec<_> = actions.iter().collect();
            sorted_actions.sort_by(|a, b| a.cost.partial_cmp(&b.cost).unwrap_or(Ordering::Equal));

            for action in sorted_actions {
                if !action.can_perform(&node.state) {
                    continue;
                }

                let mut new_state = node.state.clone();
                action.apply_effects(&mut new_state);

                let new_g_cost = node.g_cost + action.cost;

                // Check if we've already found a better path to this state
                if let Some(existing_idx) = nodes.iter().position(|n| n.state == new_state) {
                    if nodes[existing_idx].g_cost <= new_g_cost {
                        continue;
                    }
                }

                let new_node = Node {
                    state: new_state,
                    parent: Some(node_idx),
                    action: Some(action.clone()),
                    g_cost: new_g_cost,
                    h_cost: 0.0, // Dijkstra doesn't use heuristic
                };

                let new_node_idx = nodes.len();
                nodes.push(new_node);
                open_set.push(new_node_idx);
            }
        }

        Err(GoapError::NoPlanFound)
    }
}

#[derive(Debug, Clone)]
struct Node {
    state: State,
    parent: Option<usize>,
    action: Option<Action>,
    g_cost: f32,
    h_cost: f32,
}

impl Node {
    fn f_cost(&self) -> f32 {
        self.g_cost + self.h_cost
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost() == other.f_cost()
    }
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // First compare by f_cost (total cost)
        let f_cost_cmp = other.f_cost().partial_cmp(&self.f_cost());
        if f_cost_cmp != Some(Ordering::Equal) {
            return f_cost_cmp;
        }
        // If f_costs are equal, compare by g_cost (path cost so far)
        other.g_cost.partial_cmp(&self.g_cost)
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(other).unwrap_or(Ordering::Equal)
    }
}

fn heuristic(state: &State, goal: &State) -> f32 {
    // Simple heuristic: count the number of unsatisfied conditions
    goal.values()
        .iter()
        .filter(|(key, &value)| state.get(key) != Some(value))
        .count() as f32
}

fn reconstruct_path(nodes: &[Node], mut current_idx: usize) -> Vec<Action> {
    let mut path = Vec::new();

    while let Some(node) = nodes.get(current_idx) {
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

        let astar = AStarSearch;
        let plan = astar.search(&actions, &initial, &goal).unwrap();
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

        let dijkstra = DijkstraSearch;
        let plan = dijkstra.search(&actions, &initial, &goal).unwrap();
        assert_eq!(plan[0].name, "a");
    }
}
