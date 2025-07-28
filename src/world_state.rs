//! # World State Module for Goal-Oriented Action Planning (GOAP)
//!
//! This module provides the [`WorldState`] structure, which is the foundation for:
//! - Representing the current state of the environment
//! - Defining goal states for planning
//! - Calculating state differences for action planning
//! - Checking if one state satisfies another
//!
//! ## What is GOAP?
//!
//! Goal-Oriented Action Planning (GOAP) is an AI planning architecture that dynamically
//! generates sequences of actions to achieve a goal state from a starting state. Unlike
//! hardcoded AI approaches, GOAP allows for flexible, adaptive decision-making by:
//!
//! 1. Defining the current world state
//! 2. Defining a desired goal state
//! 3. Finding a sequence of actions that transform the current state into the goal state
//!
//! ## The WorldState Structure
//!
//! The [`WorldState`] is essentially a collection of key-value pairs, where:
//! - **Keys** represent different aspects of the environment (e.g., "has_weapon", "health")
//! - **Values** represent the current state of those aspects (e.g., "true", "100")
//!
//! Both keys and values are stored as strings, providing flexibility in representation.
//!
//! ## Basic Example
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
//!
//! ## Planning with WorldState
//!
//! WorldState is a key component in GOAP planning. Here's how it fits into the planning process:
//!
//! ```
//! use goaprs::world_state::WorldState;
//!
//! // Current state of an agent
//! let mut current = WorldState::new();
//! current.insert("has_wood".to_string(), "false".to_string());
//! current.insert("has_axe".to_string(), "true".to_string());
//! current.insert("at_location".to_string(), "camp".to_string());
//!
//! // Goal state - we need wood
//! let mut goal = WorldState::new();
//! goal.insert("has_wood".to_string(), "true".to_string());
//!
//! // Calculate what needs to change (the "diff")
//! let needed_changes = current.diff(&goal);
//! println!("Changes needed: {}", needed_changes); // {has_wood: true}
//!
//! // After performing actions (e.g., going to forest, chopping wood)
//! current.insert("has_wood".to_string(), "true".to_string());
//!
//! // Now our current state satisfies the goal
//! assert!(current.satisfies(&goal));
//! ```
//!
//! ## Integration with Actions
//!
//! In a complete GOAP system, `WorldState` works together with `Action`s to create plans:
//!
//! ```
//! use goaprs::{Action, world_state::WorldState};
//!
//! // Current world state
//! let mut current_state = WorldState::new();
//! current_state.insert("has_axe".to_string(), "true".to_string());
//! current_state.insert("has_wood".to_string(), "false".to_string());
//! current_state.insert("at_location".to_string(), "home".to_string());
//!
//! // Goal state - we need wood
//! let mut goal_state = WorldState::new();
//! goal_state.insert("has_wood".to_string(), "true".to_string());
//!
//! // Create actions for our agent
//! let mut move_to_forest = Action::new("move_to_forest", 1.0).unwrap();
//! move_to_forest.preconditions.inner_mut().insert("at_location".to_string(), "home".to_string());
//! move_to_forest.effects.inner_mut().insert("at_location".to_string(), "forest".to_string());
//!
//! let mut chop_wood = Action::new("chop_wood", 2.0).unwrap();
//! chop_wood.preconditions.inner_mut().insert("has_axe".to_string(), "true".to_string());
//! chop_wood.preconditions.inner_mut().insert("at_location".to_string(), "forest".to_string());
//! chop_wood.effects.inner_mut().insert("has_wood".to_string(), "true".to_string());
//!
//! // Planning process (simplified)
//! let mut plan = Vec::new();
//! let mut simulated_state = current_state.clone();
//!
//! // Check if move_to_forest can be performed
//! if move_to_forest.can_perform(&simulated_state) {
//!     // Apply effects of move_to_forest
//!     move_to_forest.apply_effects(&mut simulated_state);
//!     plan.push("move_to_forest");
//! }
//!
//! // Check if chop_wood can be performed after moving
//! if chop_wood.can_perform(&simulated_state) {
//!     // Apply effects of chop_wood
//!     chop_wood.apply_effects(&mut simulated_state);
//!     plan.push("chop_wood");
//! }
//!
//! // Verify that the final state satisfies our goal
//! assert!(simulated_state.satisfies(&goal_state));
//! assert_eq!(plan, vec!["move_to_forest", "chop_wood"]);
//! ```
use std::collections::HashMap;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::ops::{Deref, DerefMut};

/// `WorldState` represents the state of the world as a collection of key-value pairs.
///
/// This structure is a core component of the GOAP system, used for:
/// - Representing the current state of the world/agent
/// - Defining goal states that the agent wants to achieve
/// - Calculating differences between states for planning
/// - Testing if preconditions for actions are satisfied
/// - Tracking the effects of actions
///
/// Internally, `WorldState` is a thin wrapper around a `HashMap<String, String>` that adds
/// GOAP-specific operations while retaining all the functionality of a standard HashMap
/// through implementation of the `Deref` and `DerefMut` traits.
///
/// # Examples
///
/// ```
/// use goaprs::world_state::WorldState;
///
/// // Create a world state for a game character
/// let mut character_state = WorldState::new();
///
/// // Add information about the character's current state
/// character_state.insert("health".to_string(), "100".to_string());
/// character_state.insert("mana".to_string(), "50".to_string());
/// character_state.insert("has_weapon".to_string(), "true".to_string());
/// character_state.insert("location".to_string(), "town".to_string());
///
/// // Use conditional logic based on state values
/// if character_state.get("health").unwrap() == "100" &&
///    character_state.get("has_weapon").unwrap() == "true" {
///     println!("Character is ready for combat!");
/// }
///
/// // Define a goal state for a quest
/// let mut quest_goal = WorldState::new();
/// quest_goal.insert("has_artifact".to_string(), "true".to_string());
/// quest_goal.insert("location".to_string(), "town".to_string());
///
/// // Check if current state satisfies the goal
/// if !character_state.satisfies(&quest_goal) {
///     println!("Character needs to find the artifact to complete the quest");
///
///     // Calculate what needs to change to achieve the goal
///     let missing_requirements = character_state.diff(&quest_goal);
///     println!("Missing requirements: {}", missing_requirements); // {has_artifact: true}
/// }
///
/// // Update the state after completing a task
/// character_state.insert("has_artifact".to_string(), "true".to_string());
/// assert!(character_state.satisfies(&quest_goal));
/// ```
#[derive(Clone, Debug, Default)]
pub struct WorldState {
    state: HashMap<String, String>,
}

impl WorldState {
    /// Creates a new empty WorldState.
    ///
    /// This is the standard way to initialize a fresh WorldState when starting
    /// to build a representation of the world or defining a goal.
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
    /// // Create an empty world state
    /// let world_state = WorldState::new();
    /// assert!(world_state.inner().is_empty());
    ///
    /// // Build it up with key-value pairs
    /// let mut agent_state = WorldState::new();
    /// agent_state.insert("name".to_string(), "agent_1".to_string());
    /// agent_state.insert("status".to_string(), "active".to_string());
    /// ```
    pub fn new() -> Self {
        Self {
            state: HashMap::new(),
        }
    }

    /// Creates a WorldState from an existing HashMap.
    ///
    /// This method is useful when you already have state data in a HashMap
    /// and want to convert it to a WorldState to use GOAP-specific operations.
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
    /// // Maybe you have state data from an external system
    /// let mut map = HashMap::new();
    /// map.insert("position".to_string(), "home".to_string());
    /// map.insert("inventory".to_string(), "empty".to_string());
    /// map.insert("energy".to_string(), "100".to_string());
    ///
    /// // Convert to a WorldState for GOAP planning
    /// let world_state = WorldState::from_hashmap(map);
    /// assert_eq!(world_state.inner().len(), 3);
    /// assert_eq!(world_state.get("energy"), Some(&"100".to_string()));
    /// ```
    pub fn from_hashmap(map: HashMap<String, String>) -> Self {
        Self { state: map }
    }

    /// Gets a reference to the internal HashMap.
    ///
    /// This method provides direct access to the underlying HashMap
    /// that stores the state data. This can be useful for operations
    /// that require HashMap-specific functionality not exposed by the
    /// WorldState methods.
    ///
    /// # Returns
    ///
    /// A reference to the internal HashMap
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::WorldState;
    /// use std::collections::HashMap;
    ///
    /// let mut world_state = WorldState::new();
    /// world_state.insert("position".to_string(), "home".to_string());
    /// world_state.insert("inventory".to_string(), "full".to_string());
    ///
    /// // Access the internal HashMap for advanced operations
    /// let inner = world_state.inner();
    ///
    /// // Use HashMap methods directly
    /// let keys: Vec<_> = inner.keys().cloned().collect();
    /// assert!(keys.contains(&"position".to_string()));
    /// assert!(keys.contains(&"inventory".to_string()));
    ///
    /// // Check size
    /// assert_eq!(inner.len(), 2);
    /// ```
    pub fn inner(&self) -> &HashMap<String, String> {
        &self.state
    }

    /// Gets a mutable reference to the internal HashMap.
    ///
    /// This method provides direct mutable access to the underlying HashMap,
    /// allowing for batch modifications or advanced operations like iteration
    /// with mutation. It's useful when you need to perform multiple related
    /// changes to the state.
    ///
    /// # Returns
    ///
    /// A mutable reference to the internal HashMap
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::WorldState;
    /// use std::collections::HashMap;
    ///
    /// let mut world_state = WorldState::new();
    ///
    /// // Batch insert multiple values using HashMap methods
    /// {
    ///     let inner_mut = world_state.inner_mut();
    ///     inner_mut.insert("position".to_string(), "home".to_string());
    ///     inner_mut.insert("health".to_string(), "100".to_string());
    ///     inner_mut.insert("ammo".to_string(), "50".to_string());
    ///
    ///     // Conditional update using HashMap methods
    ///     if inner_mut.contains_key("health") {
    ///         inner_mut.insert("status".to_string(), "healthy".to_string());
    ///     }
    /// }
    ///
    /// // Changes are reflected in the WorldState
    /// assert_eq!(world_state.get("position"), Some(&"home".to_string()));
    /// assert_eq!(world_state.get("status"), Some(&"healthy".to_string()));
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
    /// This is a **fundamental concept in GOAP**:
    /// - Used to check if actions' preconditions are met by the current state
    /// - Used to determine if a goal state has been achieved
    /// - Used during planning to evaluate potential action sequences
    ///
    /// # Arguments
    ///
    /// * `other` - The WorldState to check against (often a goal or precondition)
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
    /// current_state.insert("has_weapon".to_string(), "true".to_string());
    ///
    /// // Check if we meet the preconditions for entering a locked room
    /// let mut room_entry_conditions = WorldState::new();
    /// room_entry_conditions.insert("has_key".to_string(), "true".to_string());
    ///
    /// // Check if we meet combat readiness conditions
    /// let mut combat_ready = WorldState::new();
    /// combat_ready.insert("has_weapon".to_string(), "true".to_string());
    /// combat_ready.insert("health".to_string(), "100".to_string());
    ///
    /// // Check conditions for a special move that we can't perform yet
    /// let mut special_move = WorldState::new();
    /// special_move.insert("has_magic_scroll".to_string(), "true".to_string());
    ///
    /// // Evaluate which actions are possible
    /// assert!(current_state.satisfies(&room_entry_conditions)); // Can enter room
    /// assert!(current_state.satisfies(&combat_ready));          // Ready for combat
    /// assert!(!current_state.satisfies(&special_move));         // Can't use special move
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
    /// This method is useful for:
    /// - Checking for exact state equality
    /// - Verifying that two states are identical (not just that one satisfies the other)
    /// - Implementing caching or memoization based on world states
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
    /// // Creating states for comparison
    /// let mut player1 = WorldState::new();
    /// player1.insert("health".to_string(), "100".to_string());
    /// player1.insert("position".to_string(), "starting_area".to_string());
    /// player1.insert("inventory".to_string(), "empty".to_string());
    ///
    /// // Identical state (same keys and values)
    /// let mut player2 = WorldState::new();
    /// player2.insert("health".to_string(), "100".to_string());
    /// player2.insert("position".to_string(), "starting_area".to_string());
    /// player2.insert("inventory".to_string(), "empty".to_string());
    ///
    /// // Partial match (subset of keys with same values)
    /// let mut partial = WorldState::new();
    /// partial.insert("health".to_string(), "100".to_string());
    /// partial.insert("position".to_string(), "starting_area".to_string());
    ///
    /// // Different value for a key
    /// let mut different = WorldState::new();
    /// different.insert("health".to_string(), "100".to_string());
    /// different.insert("position".to_string(), "forest".to_string());
    /// different.insert("inventory".to_string(), "empty".to_string());
    ///
    /// assert!(player1.matches(&player2));     // Exact match
    /// assert!(!player1.matches(&partial));    // Not an exact match (fewer keys)
    /// assert!(!player1.matches(&different));  // Not an exact match (different value)
    ///
    /// // Compare with satisfies
    /// assert!(player1.satisfies(&partial));   // player1 satisfies partial
    /// assert!(!partial.satisfies(&player1));  // partial doesn't satisfy player1
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
    /// In GOAP, this is a critical method for:
    /// - Simulating the effects of actions without modifying the original state
    /// - Building potential future states during planning
    /// - Applying action effects during plan execution
    ///
    /// # Arguments
    ///
    /// * `changes` - WorldState containing the changes to apply
    ///
    /// # Returns
    ///
    /// A new WorldState with the changes applied, leaving the original unchanged
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    ///
    /// // Initial agent state
    /// let mut agent_state = WorldState::new();
    /// agent_state.insert("health".to_string(), "100".to_string());
    /// agent_state.insert("ammo".to_string(), "50".to_string());
    /// agent_state.insert("position".to_string(), "lobby".to_string());
    /// agent_state.insert("has_keycard".to_string(), "false".to_string());
    ///
    /// // Simulate a combat encounter
    /// let mut combat_effects = WorldState::new();
    /// combat_effects.insert("health".to_string(), "80".to_string());  // Took damage
    /// combat_effects.insert("ammo".to_string(), "35".to_string());    // Used ammo
    ///
    /// // Simulate finding a keycard
    /// let mut keycard_effects = WorldState::new();
    /// keycard_effects.insert("has_keycard".to_string(), "true".to_string());
    ///
    /// // Apply combat effects first
    /// let after_combat = agent_state.apply(&combat_effects);
    ///
    /// // Then simulate finding keycard
    /// let final_state = after_combat.apply(&keycard_effects);
    ///
    /// // Check final state after both changes
    /// assert_eq!(final_state.get("health"), Some(&"80".to_string()));
    /// assert_eq!(final_state.get("ammo"), Some(&"35".to_string()));
    /// assert_eq!(final_state.get("has_keycard"), Some(&"true".to_string()));
    /// assert_eq!(final_state.get("position"), Some(&"lobby".to_string())); // Unchanged
    ///
    /// // Original state remains untouched
    /// assert_eq!(agent_state.get("health"), Some(&"100".to_string()));
    /// assert_eq!(agent_state.get("has_keycard"), Some(&"false".to_string()));
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
    /// This method is a fundamental part of GOAP planning because it helps:
    /// - Identify what changes are needed to reach a goal state
    /// - Determine the gap between current state and desired state
    /// - Focus planning on only the necessary state changes
    ///
    /// # Arguments
    ///
    /// * `other` - The WorldState to compare with (often a goal state)
    ///
    /// # Returns
    ///
    /// A new WorldState containing only the differences
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::world_state::WorldState;
    ///
    /// // Current state of an agent
    /// let mut current = WorldState::new();
    /// current.insert("health".to_string(), "100".to_string());
    /// current.insert("position".to_string(), "base".to_string());
    /// current.insert("has_weapon".to_string(), "true".to_string());
    /// current.insert("ammo".to_string(), "50".to_string());
    ///
    /// // Goal state we want to achieve
    /// let mut goal = WorldState::new();
    /// goal.insert("health".to_string(), "100".to_string());     // Same as current
    /// goal.insert("position".to_string(), "mission".to_string()); // Different
    /// goal.insert("has_key".to_string(), "true".to_string());   // New key
    /// goal.insert("ammo".to_string(), "50".to_string());        // Same as current
    ///
    /// // Calculate what needs to change to reach the goal
    /// let differences = current.diff(&goal);
    ///
    /// // The differences only include what's different or missing
    /// assert_eq!(differences.len(), 2);
    /// assert_eq!(differences.get("position"), Some(&"mission".to_string()));
    /// assert_eq!(differences.get("has_key"), Some(&"true".to_string()));
    ///
    /// // Keys with the same values in both states aren't included
    /// assert!(differences.get("health").is_none());
    /// assert!(differences.get("ammo").is_none());
    ///
    /// // We can use this to focus our planning on just what needs to change
    /// println!("Changes needed to reach goal: {}", differences); // {position: mission, has_key: true}
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
