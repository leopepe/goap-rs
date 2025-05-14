use crate::{Action, Result, State};
use std::fs::File;
use std::io::Write;

/// A visualizer for GOAP plans that generates Graphviz DOT files
pub struct GoapVisualizer;

impl GoapVisualizer {
    /// Create a new GOAP visualizer
    pub fn new() -> Self {
        Self
    }

    /// Generate a DOT file visualization of a GOAP plan
    pub fn visualize_plan(
        &self,
        actions: &[Action],
        current_state: &State,
        goal_state: &State,
        plan: &[Action],
        filename: &str,
    ) -> Result<()> {
        let mut file = File::create(filename)?;

        // Write DOT file header
        writeln!(file, "digraph GOAP {{")?;
        writeln!(file, "    rankdir=LR;")?;
        writeln!(
            file,
            "    node [shape=box, style=filled, fillcolor=lightblue];"
        )?;
        writeln!(file, "    edge [fontsize=10];")?;

        // Write initial state
        writeln!(
            file,
            "    initial [label=\"Initial State\\n{}\", fillcolor=lightgreen];",
            Self::state_to_string(current_state)
        )?;

        // Write goal state
        writeln!(
            file,
            "    goal [label=\"Goal State\\n{}\", fillcolor=lightpink];",
            Self::state_to_string(goal_state)
        )?;

        // Write all available actions
        for (i, action) in actions.iter().enumerate() {
            writeln!(
                file,
                "    action_{} [label=\"{}\\nCost: {}\\nPre: {}\\nEff: {}\"];",
                i,
                action.name,
                action.cost,
                Self::state_to_string(&action.preconditions),
                Self::state_to_string(&action.effects)
            )?;
        }

        // Write edges from initial state to possible actions
        for (i, action) in actions.iter().enumerate() {
            if action.can_perform(current_state) {
                writeln!(file, "    initial -> action_{} [label=\"possible\"];", i)?;
            }
        }

        // Write edges from actions to goal state
        for (i, action) in actions.iter().enumerate() {
            let mut new_state = current_state.clone();
            action.apply_effects(&mut new_state);
            if new_state.satisfies(goal_state) {
                writeln!(file, "    action_{} -> goal [label=\"achieves\"];", i)?;
            }
        }

        // Highlight the chosen plan path
        writeln!(file, "    edge [color=red, penwidth=2.0];")?;
        for action in plan {
            if let Some(idx) = actions.iter().position(|a| a.name == action.name) {
                writeln!(file, "    action_{} [fillcolor=lightcoral];", idx)?;
            }
        }

        // Write closing brace
        writeln!(file, "}}")?;

        Ok(())
    }

    /// Helper method to convert a state to a string representation
    fn state_to_string(state: &State) -> String {
        state
            .values()
            .iter()
            .map(|(key, value)| format!("{}: {}", key, value))
            .collect::<Vec<_>>()
            .join("\\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_visualize_plan() {
        // Create a simple action
        let mut action = Action::new("test_action", 1.0).unwrap();
        action.preconditions.set("has_item", true);
        action.effects.set("goal_achieved", true);

        let actions = vec![action.clone()];
        let plan = vec![action];

        // Set up states
        let mut current_state = State::new();
        current_state.set("has_item", true);
        current_state.set("goal_achieved", false);

        let mut goal_state = State::new();
        goal_state.set("goal_achieved", true);

        // Create visualizer and generate DOT file
        let visualizer = GoapVisualizer::new();
        visualizer
            .visualize_plan(
                &actions,
                &current_state,
                &goal_state,
                &plan,
                "test_plan.dot",
            )
            .unwrap();

        // Verify file was created and contains expected content
        let content = std::fs::read_to_string("test_plan.dot").unwrap();
        assert!(content.contains("digraph GOAP"));
        assert!(content.contains("test_action"));
        assert!(content.contains("has_item: true"));
        assert!(content.contains("goal_achieved: true"));

        // Clean up
        std::fs::remove_file("test_plan.dot").unwrap();
    }
}
