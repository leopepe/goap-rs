//! # State Module for Goal-Oriented Action Planning (GOAP)
//!
//! This module provides the fundamental `State` structure, which represents
//! world states, conditions, and effects within the GOAP system.
//!
//! ## What is State in GOAP?
//!
//! In Goal-Oriented Action Planning, "state" refers to a snapshot of the world or
//! agent at a particular moment. The GOAP system uses states in several ways:
//!
//! - **World State**: Representing the current state of the environment/agent
//! - **Goal State**: Defining desired conditions the agent wants to achieve
//! - **Preconditions**: Conditions that must be true for an action to be performed
//! - **Effects**: How actions change the world state when executed
//!
//! ## Core Features
//!
//! The `State` structure provides:
//! - Simple key-value storage for string-based state values
//! - Methods to check if one state satisfies another
//! - Ability to apply effects from one state to another
//! - Convenient access to the underlying state values
//!
//! ## Basic Usage
//!
//! ```
//! use goaprs::State;
//!
//! // Create states for a game agent
//! let mut current_state = State::new();
//! current_state.set("has_weapon", "sword");
//! current_state.set("at_destination", "false");
//! current_state.set("health", "100");
//!
//! // Define a goal state - what the agent wants to achieve
//! let mut goal_state = State::new();
//! goal_state.set("at_destination", "true");
//!
//! // Check if the current state satisfies the goal (it doesn't yet)
//! if !current_state.satisfies(&goal_state) {
//!     println!("Need to plan actions to reach the destination!");
//! }
//!
//! // Define effects of a "move" action
//! let mut move_effects = State::new();
//! move_effects.set("at_destination", "true");
//! move_effects.set("health", "80");  // movement depletes health
//!
//! // Apply the effects to the current state (simulating performing the action)
//! current_state.apply_effects(&move_effects);
//!
//! // Now the goal is satisfied
//! assert!(current_state.satisfies(&goal_state));
//! assert_eq!(current_state.get("health"), Some("80"));  // health was depleted
//! ```

use std::collections::HashMap;

/// Represents the state of the world in the Goal-Oriented Action Planning (GOAP) system.
///
/// A `State` is a collection of key-value pairs where:
/// - Keys are strings representing state variables
/// - Values are strings representing the state values
///
/// States are used to represent the current world state, action preconditions,
/// action effects, and goal states in the GOAP system.
///
/// # Role in GOAP
///
/// The `State` struct serves multiple roles in a GOAP system:
///
/// - **World State**: Tracks the current conditions of the environment and agent
/// - **Preconditions**: Defines what must be true before an action can be performed
/// - **Effects**: Describes how actions change the world when executed
/// - **Goals**: Specifies the conditions the agent is trying to achieve
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
/// state.set("player_has_key", "golden_key");
/// state.set("door_is_open", "false");
///
/// // Check state variables
/// assert_eq!(state.get("player_has_key"), Some("golden_key"));
/// assert_eq!(state.get("door_is_open"), Some("false"));
/// assert_eq!(state.get("non_existent_variable"), None);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    /// The state values as key-value pairs
    #[cfg(not(test))]
    values: HashMap<String, String>,
    #[cfg(test)]
    pub values: HashMap<String, String>,
}

impl State {
    /// Creates a new empty state.
    ///
    /// This is typically the first step when defining a world state, goal state,
    /// or action conditions in a GOAP system.
    ///
    /// # Returns
    ///
    /// A new `State` instance with no values set.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::State;
    ///
    /// // Create an empty state for an agent
    /// let state = State::new();
    /// assert!(state.values().is_empty());
    ///
    /// // States can be used for many purposes in GOAP:
    /// let mut world_state = State::new();  // Current world state
    /// let mut goal_state = State::new();   // Target state to achieve
    /// let mut preconditions = State::new(); // Action requirements
    /// let mut effects = State::new();      // Action results
    /// ```
    pub fn new() -> Self {
        Self {
            values: HashMap::new(),
        }
    }

    /// Sets a state value for the specified key.
    ///
    /// This method adds a new key-value pair to the state or updates an existing value.
    /// Keys are typically descriptive strings representing world conditions, and values
    /// are strings indicating the state of those conditions.
    ///
    /// # Arguments
    ///
    /// * `key` - The state variable name (automatically converted to String)
    /// * `value` - The string value to set
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::State;
    ///
    /// let mut state = State::new();
    ///
    /// // Setting initial values
    /// state.set("has_ammo", "30");
    /// state.set("enemy_visible", "false");
    ///
    /// // Updating an existing value
    /// state.set("has_ammo", "25"); // Ammo has been used
    ///
    /// assert_eq!(state.get("has_ammo"), Some("25"));
    /// ```
    pub fn set(&mut self, key: impl Into<String>, value: impl Into<String>) {
        self.values.insert(key.into(), value.into());
    }

    /// Gets the value for a state variable.
    ///
    /// Retrieves the string value associated with the specified key, or `None`
    /// if the key doesn't exist in the state.
    ///
    /// # Arguments
    ///
    /// * `key` - The state variable name to look up
    ///
    /// # Returns
    ///
    /// * `Some(&str)` - The value associated with the key if it exists
    /// * `None` - If the key doesn't exist in the state
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::State;
    ///
    /// let mut state = State::new();
    /// state.set("door_locked", "true");
    ///
    /// // Checking known values
    /// assert_eq!(state.get("door_locked"), Some("true"));
    ///
    /// // Handling missing values
    /// match state.get("window_open") {
    ///     Some("true") => println!("Window is open"),
    ///     Some("false") => println!("Window is closed"),
    ///     Some(value) => println!("Window state: {}", value),
    ///     None => println!("Unknown window state")
    /// }
    /// ```
    pub fn get(&self, key: &str) -> Option<&str> {
        self.values.get(key).map(|s| s.as_str())
    }

    /// Checks if this state satisfies another state's requirements.
    ///
    /// A state satisfies another state if for every key-value pair in the other state,
    /// this state has the same key with the same value. This state may contain
    /// additional key-value pairs that are not present in the other state.
    ///
    /// This is a core GOAP concept used to:
    /// - Check if current state meets goal requirements
    /// - Determine if an action's preconditions are satisfied
    /// - Validate intermediate states during planning
    ///
    /// # Arguments
    ///
    /// * `other` - The state whose requirements should be satisfied
    ///
    /// # Returns
    ///
    /// * `true` - If this state satisfies all requirements in the other state
    /// * `false` - If any requirement in the other state is not satisfied
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::State;
    ///
    /// // Current world state
    /// let mut world_state = State::new();
    /// world_state.set("has_key", "true");
    /// world_state.set("door_open", "false");
    /// world_state.set("has_weapon", "sword");
    ///
    /// // Goal state - just need the door to be open
    /// let mut goal_state = State::new();
    /// goal_state.set("door_open", "true");
    ///
    /// // Check if goal is satisfied (it's not yet)
    /// assert!(!world_state.satisfies(&goal_state));
    ///
    /// // Action preconditions - need a key to open the door
    /// let mut unlock_preconditions = State::new();
    /// unlock_preconditions.set("has_key", "true");
    /// unlock_preconditions.set("door_open", "false");
    ///
    /// // Check if action can be performed (preconditions are satisfied)
    /// assert!(world_state.satisfies(&unlock_preconditions));
    /// ```
    pub fn satisfies(&self, other: &State) -> bool {
        other
            .values
            .iter()
            .all(|(key, value)| self.values.get(key).map_or(false, |v| v == value))
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
    /// state.set("has_key", "false");
    /// state.set("door_open", "false");
    ///
    /// // Create effects to apply
    /// let mut effects = State::new();
    /// effects.set("has_key", "true");  // This will change
    /// effects.set("inventory_full", "true");  // This will be added
    ///
    /// // Apply the effects
    /// state.apply_effects(&effects);
    ///
    /// // Verify changes
    /// assert_eq!(state.get("has_key"), Some("true"));  // Changed
    /// assert_eq!(state.get("door_open"), Some("false"));  // Unchanged
    /// assert_eq!(state.get("inventory_full"), Some("true"));  // Added
    /// ```
    pub fn apply_effects(&mut self, effects: &Self) {
        for (key, value) in effects.values.iter() {
            self.set(key, value);
        }
    }

    /// Gets all the key-value pairs in the state.
    ///
    /// This method provides read-only access to the internal HashMap
    /// that stores all state variables. It's useful for iterating through
    /// all conditions, debugging state content, or when you need to perform
    /// more complex operations on the state data.
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
    /// state.set("position_x", "10");
    /// state.set("position_y", "20");
    /// state.set("has_weapon", "sword");
    ///
    /// // Access all state values
    /// let values = state.values();
    /// assert_eq!(values.len(), 3);
    /// assert!(values.contains_key("position_x"));
    /// assert!(values.contains_key("position_y"));
    ///
    /// // Iterate through all state conditions
    /// for (key, value) in values.iter() {
    ///     println!("Condition '{}' has value '{}'", key, value);
    /// }
    ///
    /// // Count how many conditions have a specific value
    /// let sword_count = values.values().filter(|&v| v == "sword").count();
    /// assert_eq!(sword_count, 1); // has_weapon is "sword"
    /// ```
    pub fn values(&self) -> &HashMap<String, String> {
        &self.values
    }
}

/// Default implementation creates an empty state.
///
/// This allows `State` to be used in contexts that require the `Default` trait,
/// such as when creating collections of states or when using certain Rust patterns
/// that leverage default values.
///
/// # Examples
///
/// ```
/// use goaprs::State;
///
/// // Create a state using Default
/// let state = State::default();
/// assert!(state.values().is_empty());
///
/// // Useful in contexts that require default values
/// let states: Vec<State> = vec![State::default(); 3];
/// assert_eq!(states.len(), 3);
/// assert!(states[0].values().is_empty());
///
/// // Create with default and then modify
/// let mut player_state = State::default();
/// player_state.set("health", "100");
/// player_state.set("has_weapon", "false");
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
        state.set("foo", "true");
        assert_eq!(state.get("foo"), Some("true"));
        state.set("foo", "false");
        assert_eq!(state.get("foo"), Some("false"));
        assert_eq!(state.get("bar"), None);
    }

    #[test]
    fn test_satisfies() {
        let mut state = State::new();
        state.set("a", "true");
        state.set("b", "false");

        let mut required = State::new();
        required.set("a", "true");
        assert!(state.satisfies(&required));
        required.set("b", "false");
        assert!(state.satisfies(&required));
        required.set("b", "true");
        assert!(!state.satisfies(&required));
        required.set("c", "true");
        assert!(!state.satisfies(&required));
    }

    #[test]
    fn test_apply_effects() {
        let mut state = State::new();
        state.set("x", "false");
        state.set("y", "false");

        let mut effects = State::new();
        effects.set("x", "true");
        effects.set("z", "true");

        state.apply_effects(&effects);
        assert_eq!(state.get("x"), Some("true"));
        assert_eq!(state.get("y"), Some("false"));
        assert_eq!(state.get("z"), Some("true"));
    }

    #[test]
    fn test_default() {
        let state: State = Default::default();
        assert!(state.values().is_empty());
    }
}
