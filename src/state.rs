use std::collections::HashMap;

/// Represents the state of the world in the Goal-Oriented Action Planning (GOAP) system.
///
/// A `State` is a collection of key-value pairs where:
/// - Keys are strings representing state variables
/// - Values are booleans representing whether conditions are true or false
///
/// States are used to represent the current world state, action preconditions,
/// action effects, and goal states in the GOAP system.
///
/// # Examples
///
/// ```
/// use goaprs::State;
///
/// // Create a new state
/// let mut state = State::new();
///
/// // Set state variables
/// state.set("player_has_key", true);
/// state.set("door_is_open", false);
///
/// // Check state variables
/// assert_eq!(state.get("player_has_key"), Some(true));
/// assert_eq!(state.get("door_is_open"), Some(false));
/// assert_eq!(state.get("non_existent_variable"), None);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    /// The state values as key-value pairs
    #[cfg(not(test))]
    values: HashMap<String, bool>,
    #[cfg(test)]
    pub values: HashMap<String, bool>,
}

impl State {
    /// Creates a new empty state.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::State;
    ///
    /// let state = State::new();
    /// assert!(state.values().is_empty());
    /// ```
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

    /// Applies the effects of another state to this state.
    ///
    /// This method updates the current state by copying all key-value pairs
    /// from the effects state, potentially overwriting existing values.
    ///
    /// # Arguments
    ///
    /// * `effects` - The state containing effects to apply
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::State;
    ///
    /// // Create an initial state
    /// let mut state = State::new();
    /// state.set("has_key", false);
    /// state.set("door_open", false);
    ///
    /// // Create effects to apply
    /// let mut effects = State::new();
    /// effects.set("has_key", true);  // This will change
    /// effects.set("inventory_full", true);  // This will be added
    ///
    /// // Apply the effects
    /// state.apply_effects(&effects);
    ///
    /// // Verify changes
    /// assert_eq!(state.get("has_key"), Some(true));  // Changed
    /// assert_eq!(state.get("door_open"), Some(false));  // Unchanged
    /// assert_eq!(state.get("inventory_full"), Some(true));  // Added
    /// ```
    pub fn apply_effects(&mut self, effects: &Self) {
        for (key, &value) in effects.values.iter() {
            self.set(key, value);
        }
    }

    /// Gets all the key-value pairs in the state.
    ///
    /// # Returns
    ///
    /// A reference to the internal HashMap containing all state variables.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::State;
    ///
    /// let mut state = State::new();
    /// state.set("position_x", true);
    /// state.set("position_y", false);
    ///
    /// let values = state.values();
    /// assert_eq!(values.len(), 2);
    /// assert!(values.contains_key("position_x"));
    /// assert!(values.contains_key("position_y"));
    /// ```
    pub fn values(&self) -> &HashMap<String, bool> {
        &self.values
    }
}

/// Default implementation creates an empty state
///
/// # Examples
///
/// ```
/// use goaprs::State;
///
/// let state = State::default();
/// assert!(state.values().is_empty());
/// ```
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
