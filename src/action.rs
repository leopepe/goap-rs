use crate::{GoapError, Result, State};

/// Represents an action in the GOAP system
#[derive(Debug, Clone)]
pub struct Action {
    /// The name of the action
    pub name: String,
    /// The cost of performing this action
    pub cost: f32,
    /// The preconditions that must be met to perform this action
    pub preconditions: State,
    /// The effects this action has on the world state
    pub effects: State,
}

impl Action {
    /// Create a new action
    pub fn new(name: impl Into<String>, cost: f32) -> Result<Self> {
        if cost <= 0.0 {
            return Err(GoapError::InvalidActionCost);
        }

        Ok(Self {
            name: name.into(),
            cost,
            preconditions: State::new(),
            effects: State::new(),
        })
    }

    /// Check if this action can be performed in the given state
    pub fn can_perform(&self, state: &State) -> bool {
        state.satisfies(&self.preconditions)
    }

    /// Apply this action's effects to the given state
    pub fn apply_effects(&self, state: &mut State) {
        state.apply_effects(&self.effects);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_valid_action() {
        let action = Action::new("test_action", 1.0).unwrap();
        assert_eq!(action.name, "test_action");
        assert_eq!(action.cost, 1.0);
        assert!(action.preconditions.values().is_empty());
        assert!(action.effects.values().is_empty());
    }

    #[test]
    fn test_create_invalid_action() {
        let result = Action::new("test_action", 0.0);
        assert!(matches!(result, Err(GoapError::InvalidActionCost)));

        let result = Action::new("test_action", -1.0);
        assert!(matches!(result, Err(GoapError::InvalidActionCost)));
    }

    #[test]
    fn test_can_perform_with_empty_preconditions() {
        let action = Action::new("test_action", 1.0).unwrap();
        let state = State::new();
        assert!(action.can_perform(&state));
    }

    #[test]
    fn test_can_perform_with_matching_preconditions() {
        let mut action = Action::new("test_action", 1.0).unwrap();
        action.preconditions.set("has_tool", true);

        let mut state = State::new();
        state.set("has_tool", true);

        assert!(action.can_perform(&state));
    }

    #[test]
    fn test_can_perform_with_unmatching_preconditions() {
        let mut action = Action::new("test_action", 1.0).unwrap();
        action.preconditions.set("has_tool", true);

        let mut state = State::new();
        state.set("has_tool", false);

        assert!(!action.can_perform(&state));
    }

    #[test]
    fn test_can_perform_with_missing_preconditions() {
        let mut action = Action::new("test_action", 1.0).unwrap();
        action.preconditions.set("has_tool", true);

        let state = State::new();
        assert!(!action.can_perform(&state));
    }

    #[test]
    fn test_apply_effects_empty() {
        let action = Action::new("test_action", 1.0).unwrap();
        let mut state = State::new();
        action.apply_effects(&mut state);
        assert!(state.values().is_empty());
    }

    #[test]
    fn test_apply_effects_single() {
        let mut action = Action::new("test_action", 1.0).unwrap();
        action.effects.set("has_result", true);

        let mut state = State::new();
        action.apply_effects(&mut state);

        assert_eq!(state.get("has_result"), Some(true));
    }

    #[test]
    fn test_apply_effects_multiple() {
        let mut action = Action::new("test_action", 1.0).unwrap();
        action.effects.set("has_result", true);
        action.effects.set("is_complete", true);

        let mut state = State::new();
        action.apply_effects(&mut state);

        assert_eq!(state.get("has_result"), Some(true));
        assert_eq!(state.get("is_complete"), Some(true));
    }

    #[test]
    fn test_apply_effects_overwrite() {
        let mut action = Action::new("test_action", 1.0).unwrap();
        action.effects.set("has_result", true);

        let mut state = State::new();
        state.set("has_result", false);
        action.apply_effects(&mut state);

        assert_eq!(state.get("has_result"), Some(true));
    }
}
