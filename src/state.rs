use std::collections::HashMap;

/// Represents the state of the world in the GOAP system
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    /// The state values
    #[cfg(not(test))]
    values: HashMap<String, bool>,
    #[cfg(test)]
    pub values: HashMap<String, bool>,
}

impl State {
    /// Create a new empty state
    pub fn new() -> Self {
        Self {
            values: HashMap::new(),
        }
    }

    /// Set a state value
    pub fn set(&mut self, key: impl Into<String>, value: bool) {
        self.values.insert(key.into(), value);
    }

    /// Get a state value
    pub fn get(&self, key: &str) -> Option<bool> {
        self.values.get(key).copied()
    }

    /// Check if this state satisfies another state's requirements
    pub fn satisfies(&self, other: &State) -> bool {
        other
            .values
            .iter()
            .all(|(key, &value)| self.values.get(key).map_or(false, |&v| v == value))
    }

    /// Apply the effects of an action to this state
    pub fn apply_effects(&mut self, effects: &State) {
        for (key, &value) in effects.values.iter() {
            self.values.insert(key.clone(), value);
        }
    }

    /// Crate-public getter for values (for planner/tests)
    pub(crate) fn values(&self) -> &HashMap<String, bool> {
        &self.values
    }
}

impl Default for State {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_state_is_empty() {
        let state = State::new();
        assert!(state.values().is_empty());
    }

    #[test]
    fn test_set_and_get() {
        let mut state = State::new();
        state.set("foo", true);
        assert_eq!(state.get("foo"), Some(true));
        state.set("foo", false);
        assert_eq!(state.get("foo"), Some(false));
        assert_eq!(state.get("bar"), None);
    }

    #[test]
    fn test_satisfies() {
        let mut state = State::new();
        state.set("a", true);
        state.set("b", false);

        let mut required = State::new();
        required.set("a", true);
        assert!(state.satisfies(&required));
        required.set("b", false);
        assert!(state.satisfies(&required));
        required.set("b", true);
        assert!(!state.satisfies(&required));
        required.set("c", true);
        assert!(!state.satisfies(&required));
    }

    #[test]
    fn test_apply_effects() {
        let mut state = State::new();
        state.set("x", false);
        state.set("y", false);

        let mut effects = State::new();
        effects.set("x", true);
        effects.set("z", true);

        state.apply_effects(&effects);
        assert_eq!(state.get("x"), Some(true));
        assert_eq!(state.get("y"), Some(false));
        assert_eq!(state.get("z"), Some(true));
    }

    #[test]
    fn test_default() {
        let state: State = Default::default();
        assert!(state.values().is_empty());
    }
}
