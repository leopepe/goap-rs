use thiserror::Error;

/// Custom error types for the Goal-Oriented Action Planning (GOAP) system.
///
/// This enum provides a comprehensive set of error variants that can occur
/// during GOAP operations, categorized by their source or nature.
///
/// # Examples
///
/// ```
/// use goaprs::GoapError;
/// use std::error::Error;
///
/// // Create an error instance
/// let error = GoapError::OperationFailed("could not complete operation".to_string());
///
/// // Error messages are formatted based on their variant
/// assert_eq!(format!("{}", error), "Operation failed: could not complete operation");
///
/// // Errors can be converted to std::error::Error trait objects
/// let boxed: Box<dyn Error> = Box::new(error);
/// ```
#[derive(Error, Debug)]
pub enum GoapError {
    /// A general operation failure with a message describing what went wrong
    #[error("Operation failed: {0}")]
    OperationFailed(String),

    // Sensor errors
    /// A general sensor-related error with a message describing what went wrong
    #[error("Sensor error: {0}")]
    Sensor(String),

    /// Error when attempting to assign multiple types to a sensor
    #[error("Sensor has multiple types")]
    SensorMultipleType,

    /// Error when attempting to access a non-existent sensor
    #[error("Sensor does not exist")]
    SensorDoesNotExist,

    /// Error when attempting to add a sensor with a name that already exists in the collection
    #[error("Sensor already in collection: {0}")]
    SensorAlreadyInCollection(String),

    // Plan errors
    /// A general plan-related error with a message describing what went wrong
    #[error("Plan error: {0}")]
    Plan(String),

    /// Error when planning algorithm fails to produce a valid action sequence
    #[error("Failed to produce a plan")]
    PlanFailed,

    // Action errors
    /// A general action-related error with a message describing what went wrong
    #[error("Action error: {0}")]
    Action(String),

    /// Error when attempting to assign multiple types to an action
    #[error("Action has multiple types")]
    ActionMultipleType,

    /// Error when attempting to add an action with a name that already exists in the collection
    #[error("Action already in collection: {0}")]
    ActionAlreadyInCollection(String),

    /// Error when attempting to create an action with zero or negative cost
    #[error("Action cost must be positive")]
    InvalidActionCost,

    /// Error when an action's preconditions aren't met in the current state
    #[error("Action precondition not met: {0}")]
    PreconditionNotMet(String),

    // IO errors
    /// A wrapper around standard IO errors
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    // Command execution errors
    /// Error when executing a shell command fails
    #[error("Command execution failed: {0}")]
    CommandExecution(String),

    // Graph errors
    /// A general graph-related error with a message describing what went wrong
    #[error("Graph error: {0}")]
    Graph(String),

    /// Error when path-finding algorithms cannot find a path in the graph
    #[error("No path found in graph")]
    NoPathFound,

    // Serialization errors
    /// A wrapper around serde_json serialization/deserialization errors
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    // Other errors
    /// Catch-all for errors that don't fit into other categories
    #[error("Other error: {0}")]
    Other(String),

    /// Error when attempting to access a plan that hasn't been generated
    #[error("No plan has been generated yet")]
    NoPlanGenerated,

    /// Error when the planner cannot find any valid sequence of actions to achieve the goal
    #[error("No valid plan found to achieve the goal")]
    NoPlanFound,

    /// Error when attempting an invalid state transition in the automaton
    #[error("Invalid state transition: {0}")]
    InvalidStateTransition(String),
}

/// Result type alias for GOAP operations
///
/// This is a convenient way to represent results of operations that might fail
/// with a `GoapError`.
///
/// # Examples
///
/// ```
/// use goaprs::{GoapError, Result};
///
/// fn might_fail(succeed: bool) -> Result<String> {
///     if succeed {
///         Ok("Operation succeeded".to_string())
///     } else {
///         Err(GoapError::OperationFailed("Operation failed".to_string()))
///     }
/// }
///
/// // Success case
/// assert!(might_fail(true).is_ok());
///
/// // Error case
/// assert!(might_fail(false).is_err());
/// ```
pub type Result<T> = std::result::Result<T, GoapError>;

#[cfg(test)]
mod tests {
    use super::*;
    use std::error::Error;

    #[test]
    fn test_no_plan_found_display() {
        let err = GoapError::NoPlanFound;
        assert_eq!(
            format!("{}", err),
            "No valid plan found to achieve the goal"
        );
    }

    #[test]
    fn test_invalid_state_transition_display() {
        let err = GoapError::InvalidStateTransition("foo".to_string());
        assert_eq!(format!("{}", err), "Invalid state transition: foo");
    }

    #[test]
    fn test_precondition_not_met_display() {
        let err = GoapError::PreconditionNotMet("bar".to_string());
        assert_eq!(format!("{}", err), "Action precondition not met: bar");
    }

    #[test]
    fn test_invalid_action_cost_display() {
        let err = GoapError::InvalidActionCost;
        assert_eq!(format!("{}", err), "Action cost must be positive");
    }

    #[test]
    fn test_error_trait() {
        let err = GoapError::NoPlanFound;
        let _ = err.source(); // Should be None
    }
}
