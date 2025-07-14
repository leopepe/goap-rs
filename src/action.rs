use crate::{GoapError, Result, State};
use std::fmt;

/// Response from executing an action
#[derive(Debug, Clone)]
pub struct ActionResponse {
    /// Standard output from the action execution
    stdout: String,
    /// Standard error from the action execution
    stderr: String,
    /// Return code (0 for success, non-zero for failure)
    return_code: i32,
}

impl ActionResponse {
    /// Creates a new action response
    pub fn new(stdout: String, stderr: String, return_code: i32) -> Self {
        Self {
            stdout,
            stderr,
            return_code,
        }
    }

    /// Gets the standard output from the action execution
    pub fn stdout(&self) -> &str {
        &self.stdout
    }

    /// Gets the standard error from the action execution
    pub fn stderr(&self) -> &str {
        &self.stderr
    }

    /// Gets the return code from the action execution
    pub fn return_code(&self) -> i32 {
        self.return_code
    }

    /// Gets a formatted response string
    pub fn response(&self) -> String {
        self.stdout.clone()
    }

    /// Checks if the action execution was successful
    pub fn is_success(&self) -> bool {
        self.return_code == 0
    }
}

impl fmt::Display for ActionResponse {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.stdout)
    }
}

/// Represents an action in the Goal-Oriented Action Planning (GOAP) system.
///
/// An action in GOAP has:
/// - A name that uniquely identifies it
/// - A cost representing the relative expense of performing the action
/// - Preconditions that must be met for the action to be executable
/// - Effects that describe how the world state changes after execution
///
/// # Examples
///
/// ```
/// use goaprs::{Action, State};
///
/// // Create a new action with a name and cost
/// let mut action = Action::new("move_to_target", 1.5).unwrap();
///
/// // Add preconditions - what must be true to perform this action
/// action.preconditions.set("has_map", true);
/// action.preconditions.set("target_visible", true);
///
/// // Add effects - what changes after performing this action
/// action.effects.set("at_target", true);
/// action.effects.set("energy", false); // Using energy
///
/// // Check if the action can be performed in the current state
/// let mut state = State::new();
/// state.set("has_map", true);
/// state.set("target_visible", true);
/// assert!(action.can_perform(&state));
/// ```
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
    /// Creates a new action with the specified name and cost.
    ///
    /// # Arguments
    ///
    /// * `name` - A unique identifier for the action
    /// * `cost` - A positive number representing the relative cost of this action
    ///
    /// # Returns
    ///
    /// A `Result` containing the new `Action` or an error if the cost is invalid.
    ///
    /// # Errors
    ///
    /// Returns `GoapError::InvalidActionCost` if the cost is zero or negative.
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::Action;
    ///
    /// // Create a valid action
    /// let action = Action::new("move", 1.0).unwrap();
    /// assert_eq!(action.name, "move");
    /// assert_eq!(action.cost, 1.0);
    ///
    /// // Invalid cost (negative) will return an error
    /// let invalid_action = Action::new("invalid", -1.0);
    /// assert!(invalid_action.is_err());
    /// ```
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

    /// Checks if this action can be performed in the given state.
    ///
    /// An action can be performed if all of its preconditions are satisfied
    /// by the current world state.
    ///
    /// # Arguments
    ///
    /// * `state` - The current world state to check against
    ///
    /// # Returns
    ///
    /// `true` if the action's preconditions are met in the given state, otherwise `false`
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::{Action, State};
    ///
    /// let mut action = Action::new("pick_up", 1.0).unwrap();
    /// action.preconditions.set("item_visible", true);
    /// action.preconditions.set("hands_free", true);
    ///
    /// // State that satisfies all preconditions
    /// let mut valid_state = State::new();
    /// valid_state.set("item_visible", true);
    /// valid_state.set("hands_free", true);
    /// assert!(action.can_perform(&valid_state));
    ///
    /// // State that doesn't satisfy all preconditions
    /// let mut invalid_state = State::new();
    /// invalid_state.set("item_visible", true);
    /// invalid_state.set("hands_free", false);
    /// assert!(!action.can_perform(&invalid_state));
    /// ```
    pub fn can_perform(&self, state: &State) -> bool {
        state.satisfies(&self.preconditions)
    }

    /// Applies this action's effects to the given state.
    ///
    /// This modifies the input state by applying all of the action's effects.
    /// Each effect will either add a new key-value pair to the state or
    /// overwrite an existing value.
    ///
    /// # Arguments
    ///
    /// * `state` - The world state to modify
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::{Action, State};
    ///
    /// let mut action = Action::new("pick_up_item", 1.0).unwrap();
    /// action.effects.set("has_item", true);
    /// action.effects.set("hands_free", false);
    ///
    /// let mut state = State::new();
    /// state.set("hands_free", true);
    /// state.set("item_visible", true);
    ///
    /// // Apply the effects of the action
    /// action.apply_effects(&mut state);
    ///
    /// // Verify the state was modified correctly
    /// assert_eq!(state.get("has_item"), Some(true));
    /// assert_eq!(state.get("hands_free"), Some(false));
    /// assert_eq!(state.get("item_visible"), Some(true)); // Unchanged
    /// ```
    pub fn apply_effects(&self, state: &mut State) {
        state.apply_effects(&self.effects);
    }

    /// Executes this action and returns a dummy response.
    ///
    /// This is a basic implementation that simply returns a success response
    /// with the action name. In a real application, you would override this
    /// method to implement actual behavior.
    ///
    /// # Returns
    ///
    /// A `Result` containing information about the action execution.
    pub async fn exec(&self) -> Result<ActionResponse> {
        Ok(ActionResponse::new(
            format!("Executed action: {}", self.name),
            String::new(),
            0,
        ))
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
