//! World state representation for GOAP (Goal-Oriented Action Planning).
//!
//! This module provides the [`WorldState`] structure, which is the foundation for:
//! - Representing the current state of the environment
//! - Defining goal states for planning
//! - Calculating state differences for action planning
//! - Checking if one state satisfies another
//!
//! The world state is essentially a collection of key-value pairs, where each key
//! represents some aspect of the environment, and the value represents its current
//! state or condition.
//!
//! # Example
//!
//! ```
//! use goaprs::world_state::WorldState;
//! use std::collections::HashMap;
//!
//! // Create a world state representing a character in a game
//! let mut world_state = WorldState::new();
//! world_state.insert("health".to_string(), "100".to_string());
//! world_state.insert("has_weapon".to_string(), "true".to_string());
//! world_state.insert("at_location".to_string(), "town".to_string());
//!
//! // Create a goal state - the character wants to be at the dungeon
//! let mut goal_state = WorldState::new();
//! goal_state.insert("at_location".to_string(), "dungeon".to_string());
//!
//! // Check if the world state satisfies the goal (it doesn't yet)
//! assert!(!world_state.satisfies(&goal_state));
//!
//! // Update the world state
//! world_state.insert("at_location".to_string(), "dungeon".to_string());
//!
//! // Now the world state satisfies the goal
//! assert!(world_state.satisfies(&goal_state));
//! ```

use std::collections::HashMap;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::ops::{Deref, DerefMut};

/// `WorldState` represents the state of the world as a collection of key-value pairs.
///
/// It is used both for representing the current state of the world and goal states.
/// This is essentially a thin wrapper around a HashMap that adds some additional
/// functionality specific to GOAP.
#[derive(Clone, Debug, Default)]
pub struct WorldState {
    state: HashMap<String, String>,
}

impl WorldState {
    /// Creates a new empty WorldState.
    ///
    /// # Returns
    ///
    /// A new empty WorldState instance
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::WorldState;
    ///
    /// let world_state = WorldState::new();
    /// assert!(world_state.inner().is_empty());
    /// ```
    pub fn new() -> Self {
        Self {
            state: HashMap::new(),
        }
    }

    /// Creates a WorldState from an existing HashMap.
    ///
    /// # Arguments
    ///
    /// * `map` - A HashMap containing the initial state data
    ///
    /// # Returns
    ///
    /// A new WorldState initialized with the provided map
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::WorldState;
    /// use std::collections::HashMap;
    ///
    /// let mut map = HashMap::new();
    /// map.insert("position".to_string(), "home".to_string());
    /// map.insert("inventory".to_string(), "empty".to_string());
    ///
    /// // Create a WorldState from a HashMap
    /// let world_state = WorldState::from_hashmap(map);
    /// assert_eq!(world_state.inner().len(), 2);
    /// ```
    pub fn from_hashmap(map: HashMap<String, String>) -> Self {
        Self { state: map }
    }

    /// Gets a reference to the internal HashMap.
    ///
    /// This method provides direct access to the underlying HashMap
    /// that stores the state data.
    ///
    /// # Returns
    ///
    /// A reference to the internal HashMap
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::WorldState;
    ///
    /// let mut world_state = WorldState::new();
    /// world_state.insert("position".to_string(), "home".to_string());
    ///
    /// // Access the internal HashMap
    /// let inner = world_state.inner();
    /// assert_eq!(inner.get("position"), Some(&"home".to_string()));
    /// ```
    pub fn inner(&self) -> &HashMap<String, String> {
        &self.state
    }

    /// Gets a mutable reference to the internal HashMap.
    ///
    /// This method provides direct mutable access to the underlying HashMap,
    /// allowing for batch modifications or advanced operations.
    ///
    /// # Returns
    ///
    /// A mutable reference to the internal HashMap
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::WorldState;
    ///
    /// let mut world_state = WorldState::new();
    ///
    /// // Modify the internal HashMap directly
    /// {
    ///     let inner_mut = world_state.inner_mut();
    ///     inner_mut.insert("position".to_string(), "home".to_string());
    /// }
    ///
    /// assert_eq!(world_state.get("position"), Some(&"home".to_string()));
    /// ```
    pub fn inner_mut(&mut self) -> &mut HashMap<String, String> {
        &mut self.state
    }

    /// Checks if this WorldState satisfies the requirements in another WorldState.
    ///
    /// A WorldState satisfies another if for every key-value pair in the other state,
    /// this state contains the same key with the same value. This state may contain
    /// additional key-value pairs that are not present in the other state.
    ///
    /// This is a key concept in GOAP, as it allows checking if the current world state
    /// meets the goal requirements.
    ///
    /// # Arguments
    ///
    /// * `other` - The WorldState to check against
    ///
    /// # Returns
    ///
    /// `true` if this state satisfies the other state, `false` otherwise
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    ///
    /// // Current world state with multiple properties
    /// let mut current_state = WorldState::new();
    /// current_state.insert("health".to_string(), "100".to_string());
    /// current_state.insert("position".to_string(), "x:10,y:20".to_string());
    /// current_state.insert("has_key".to_string(), "true".to_string());
    ///
    /// // Goal state - just need to have the key
    /// let mut goal_state = WorldState::new();
    /// goal_state.insert("has_key".to_string(), "true".to_string());
    ///
    /// // Current state satisfies the goal because it has the required key-value pair
    /// assert!(current_state.satisfies(&goal_state));
    ///
    /// // If the goal requires something not in the current state, it doesn't satisfy
    /// let mut different_goal = WorldState::new();
    /// different_goal.insert("has_map".to_string(), "true".to_string());
    /// assert!(!current_state.satisfies(&different_goal));
    /// ```
    pub fn satisfies(&self, other: &WorldState) -> bool {
        for (key, value) in other.iter() {
            match self.get(key) {
                Some(self_value) if self_value == value => {}
                _ => return false,
            }
        }
        true
    }

    /// Checks if this WorldState exactly matches another WorldState.
    ///
    /// Two WorldStates match exactly if they have the same number of key-value pairs
    /// and all keys and values are identical. This is stricter than `satisfies()`,
    /// which only requires that the keys in the other state exist with the same values.
    ///
    /// # Arguments
    ///
    /// * `other` - The WorldState to compare with
    ///
    /// # Returns
    ///
    /// `true` if both states match exactly, `false` otherwise
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    ///
    /// let mut state1 = WorldState::new();
    /// state1.insert("key1".to_string(), "value1".to_string());
    /// state1.insert("key2".to_string(), "value2".to_string());
    ///
    /// // Identical state
    /// let mut state2 = WorldState::new();
    /// state2.insert("key1".to_string(), "value1".to_string());
    /// state2.insert("key2".to_string(), "value2".to_string());
    ///
    /// // Subset state
    /// let mut state3 = WorldState::new();
    /// state3.insert("key1".to_string(), "value1".to_string());
    ///
    /// assert!(state1.matches(&state2)); // Exact match
    /// assert!(!state1.matches(&state3)); // state3 is a subset, not an exact match
    /// ```
    pub fn matches(&self, other: &WorldState) -> bool {
        if self.len() != other.len() {
            return false;
        }
        self.satisfies(other)
    }

    /// Creates a new WorldState that is the result of applying changes to this WorldState.
    ///
    /// This method creates a clone of the current state and then applies all
    /// key-value pairs from the changes state, overwriting any existing values for those keys.
    ///
    /// This is useful for simulating the effects of actions on the world state.
    ///
    /// # Arguments
    ///
    /// * `changes` - WorldState containing the changes to apply
    ///
    /// # Returns
    ///
    /// A new WorldState with the changes applied
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    ///
    /// // Initial state
    /// let mut initial = WorldState::new();
    /// initial.insert("health".to_string(), "100".to_string());
    /// initial.insert("ammo".to_string(), "50".to_string());
    /// initial.insert("position".to_string(), "lobby".to_string());
    ///
    /// // Changes to apply
    /// let mut changes = WorldState::new();
    /// changes.insert("health".to_string(), "80".to_string()); // Health decreased
    /// changes.insert("ammo".to_string(), "45".to_string()); // Ammo decreased
    /// changes.insert("position".to_string(), "hallway".to_string()); // Position changed
    ///
    /// // Apply changes
    /// let new_state = initial.apply(&changes);
    ///
    /// // Check results
    /// assert_eq!(new_state.get("health"), Some(&"80".to_string()));
    /// assert_eq!(new_state.get("ammo"), Some(&"45".to_string()));
    /// assert_eq!(new_state.get("position"), Some(&"hallway".to_string()));
    ///
    /// // Original state remains unchanged
    /// assert_eq!(initial.get("health"), Some(&"100".to_string()));
    /// ```
    pub fn apply(&self, changes: &WorldState) -> Self {
        let mut new_state = self.clone();
        for (key, value) in changes.iter() {
            new_state.insert(key.clone(), value.clone());
        }
        new_state
    }

    /// Returns the difference between this WorldState and another.
    ///
    /// The result contains all key-value pairs from `other` that either:
    /// - Have a different value in this state
    /// - Don't exist in this state
    ///
    /// This method is useful for determining what needs to change to transform
    /// the current state into the desired state.
    ///
    /// # Arguments
    ///
    /// * `other` - The WorldState to compare with
    ///
    /// # Returns
    ///
    /// A new WorldState containing the differences
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    ///
    /// // Current state
    /// let mut current = WorldState::new();
    /// current.insert("health".to_string(), "100".to_string());
    /// current.insert("position".to_string(), "base".to_string());
    /// current.insert("has_weapon".to_string(), "true".to_string());
    ///
    /// // Target state
    /// let mut target = WorldState::new();
    /// target.insert("health".to_string(), "100".to_string()); // Same as current
    /// target.insert("position".to_string(), "mission".to_string()); // Different
    /// target.insert("has_key".to_string(), "true".to_string()); // New key
    ///
    /// // Calculate differences
    /// let differences = current.diff(&target);
    ///
    /// // Should contain only the different and new keys
    /// assert_eq!(differences.len(), 2);
    /// assert_eq!(differences.get("position"), Some(&"mission".to_string()));
    /// assert_eq!(differences.get("has_key"), Some(&"true".to_string()));
    /// assert!(differences.get("health").is_none()); // Same in both states
    /// ```
    pub fn diff(&self, other: &WorldState) -> Self {
        let mut diff = WorldState::new();

        for (key, other_value) in other.iter() {
            match self.get(key) {
                Some(self_value) if self_value != other_value => {
                    diff.insert(key.clone(), other_value.clone());
                }
                None => {
                    diff.insert(key.clone(), other_value.clone());
                }
                _ => {}
            }
        }

        diff
    }
}

impl Deref for WorldState {
    type Target = HashMap<String, String>;

    /// Provides read access to the underlying HashMap.
    ///
    /// This implementation allows WorldState to be used wherever a
    /// reference to a HashMap<String, String> is expected, enabling
    /// all read-only HashMap methods to be used directly on WorldState.
    fn deref(&self) -> &Self::Target {
        &self.state
    }
}

impl DerefMut for WorldState {
    /// Provides mutable access to the underlying HashMap.
    ///
    /// This implementation allows WorldState to be used wherever a
    /// mutable reference to a HashMap<String, String> is expected,
    /// enabling all HashMap modification methods to be used directly on WorldState.
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.state
    }
}

impl From<HashMap<String, String>> for WorldState {
    /// Creates a WorldState from a HashMap.
    ///
    /// This provides a convenient way to convert an existing HashMap into a WorldState.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    /// use std::collections::HashMap;
    ///
    /// let mut map = HashMap::new();
    /// map.insert("key".to_string(), "value".to_string());
    ///
    /// // Convert HashMap to WorldState
    /// let state: WorldState = map.into();
    /// assert_eq!(state.get("key"), Some(&"value".to_string()));
    /// ```
    fn from(state: HashMap<String, String>) -> Self {
        Self { state }
    }
}

impl From<WorldState> for HashMap<String, String> {
    /// Converts a WorldState back to a HashMap.
    ///
    /// This provides a convenient way to extract the underlying HashMap from a WorldState.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    /// use std::collections::HashMap;
    ///
    /// let mut state = WorldState::new();
    /// state.insert("key".to_string(), "value".to_string());
    ///
    /// // Convert WorldState to HashMap
    /// let map: HashMap<String, String> = state.into();
    /// assert_eq!(map.get("key"), Some(&"value".to_string()));
    /// ```
    fn from(world_state: WorldState) -> Self {
        world_state.state
    }
}

impl Hash for WorldState {
    /// Implements hashing for WorldState.
    ///
    /// This creates a consistent hash value for a WorldState by:
    /// 1. Sorting the key-value pairs alphabetically by key
    /// 2. Hashing each key-value pair in order
    ///
    /// This ensures that the same WorldState always produces the same hash value,
    /// regardless of the order in which keys were inserted.
    ///
    /// # Arguments
    ///
    /// * `state` - The hasher to update with the hash of this WorldState
    fn hash<H: Hasher>(&self, state: &mut H) {
        // Create a sorted vector of key-value pairs for consistent hashing
        let mut items: Vec<_> = self.state.iter().collect();
        items.sort_by(|a, b| a.0.cmp(b.0));

        for (key, value) in items {
            key.hash(state);
            value.hash(state);
        }
    }
}

impl PartialEq for WorldState {
    /// Compares two WorldStates for equality.
    ///
    /// WorldStates are considered equal if they have the exact same key-value pairs,
    /// regardless of insertion order.
    ///
    /// # Arguments
    ///
    /// * `other` - The WorldState to compare with
    ///
    /// # Returns
    ///
    /// `true` if both states match exactly, `false` otherwise
    fn eq(&self, other: &Self) -> bool {
        self.matches(other)
    }
}

/// Implements the `Eq` trait for `WorldState`.
///
/// This confirms that `WorldState` follows the reflexive, symmetric, and transitive
/// properties required for equality comparison.
impl Eq for WorldState {}

impl fmt::Display for WorldState {
    /// Formats the WorldState as a string.
    ///
    /// Displays the WorldState as a JSON-like object: `{key1: value1, key2: value2, ...}`
    ///
    /// # Arguments
    ///
    /// * `f` - The formatter to write to
    ///
    /// # Returns
    ///
    /// A `fmt::Result` indicating whether the operation succeeded or failed
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    ///
    /// let mut state = WorldState::new();
    /// state.insert("key1".to_string(), "value1".to_string());
    /// state.insert("key2".to_string(), "value2".to_string());
    ///
    /// // When converted to a string, it looks like a JSON object
    /// let state_string = state.to_string();
    /// // Note: actual order may vary due to HashMap iteration
    /// assert!(state_string == "{key1: value1, key2: value2}" ||
    ///         state_string == "{key2: value2, key1: value1}");
    /// ```
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{{")?;
        let mut first = true;
        for (key, value) in &self.state {
            if !first {
                write!(f, ", ")?;
            }
            write!(f, "{}: {}", key, value)?;
            first = false;
        }
        write!(f, "}}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_world_state_satisfies() {
        let mut state1 = WorldState::new();
        state1.insert("key1".to_string(), "value1".to_string());
        state1.insert("key2".to_string(), "value2".to_string());

        let mut state2 = WorldState::new();
        state2.insert("key1".to_string(), "value1".to_string());

        let mut state3 = WorldState::new();
        state3.insert("key1".to_string(), "different".to_string());

        assert!(state1.satisfies(&state2));
        assert!(!state2.satisfies(&state1));
        assert!(!state1.satisfies(&state3));
    }

    #[test]
    fn test_world_state_matches() {
        let mut state1 = WorldState::new();
        state1.insert("key1".to_string(), "value1".to_string());
        state1.insert("key2".to_string(), "value2".to_string());

        let mut state2 = WorldState::new();
        state2.insert("key1".to_string(), "value1".to_string());
        state2.insert("key2".to_string(), "value2".to_string());

        let mut state3 = WorldState::new();
        state3.insert("key1".to_string(), "value1".to_string());
        state3.insert("key2".to_string(), "different".to_string());

        assert!(state1.matches(&state2));
        assert!(state2.matches(&state1));
        assert!(!state1.matches(&state3));
    }

    #[test]
    fn test_world_state_apply() {
        let mut base = WorldState::new();
        base.insert("key1".to_string(), "value1".to_string());
        base.insert("key2".to_string(), "value2".to_string());

        let mut changes = WorldState::new();
        changes.insert("key2".to_string(), "updated".to_string());
        changes.insert("key3".to_string(), "value3".to_string());

        let result = base.apply(&changes);

        assert_eq!(result.get("key1"), Some(&"value1".to_string()));
        assert_eq!(result.get("key2"), Some(&"updated".to_string()));
        assert_eq!(result.get("key3"), Some(&"value3".to_string()));
    }

    #[test]
    fn test_world_state_diff() {
        let mut state1 = WorldState::new();
        state1.insert("key1".to_string(), "value1".to_string());
        state1.insert("key2".to_string(), "value2".to_string());

        let mut state2 = WorldState::new();
        state2.insert("key1".to_string(), "value1".to_string()); // Same
        state2.insert("key2".to_string(), "updated".to_string()); // Different
        state2.insert("key3".to_string(), "value3".to_string()); // New

        let diff = state1.diff(&state2);

        assert_eq!(diff.len(), 2);
        assert!(diff.get("key1").is_none()); // Not different
        assert_eq!(diff.get("key2"), Some(&"updated".to_string()));
        assert_eq!(diff.get("key3"), Some(&"value3".to_string()));
    }
}
