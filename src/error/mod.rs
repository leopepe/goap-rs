use thiserror::Error;

/// Custom error types for GOAP
#[derive(Error, Debug)]
pub enum GoapError {
    // General operation errors
    #[error("Operation failed: {0}")]
    OperationFailed(String),

    // Plan errors
    #[error("Plan error: {0}")]
    Plan(String),

    #[error("Failed to produce a plan")]
    PlanFailed,

    #[error("No valid plan found to achieve the goal")]
    NoPlanFound,

    #[error("No plan has been generated yet")]
    NoPlanGenerated,

    // State errors
    #[error("Invalid state transition: {0}")]
    InvalidStateTransition(String),

    // Sensor errors
    #[error("Sensor error: {0}")]
    Sensor(String),

    #[error("Sensor has multiple types")]
    SensorMultipleType,

    #[error("Sensor does not exist")]
    SensorDoesNotExist,

    #[error("Sensor already in collection: {0}")]
    SensorAlreadyInCollection(String),

    // Action errors
    #[error("Action error: {0}")]
    Action(String),

    #[error("Action has multiple types")]
    ActionMultipleType,

    #[error("Action already in collection: {0}")]
    ActionAlreadyInCollection(String),

    #[error("Action precondition not met: {0}")]
    PreconditionNotMet(String),

    #[error("Action cost must be positive")]
    InvalidActionCost,

    // Graph errors
    #[error("Graph error: {0}")]
    Graph(String),

    #[error("No path found in graph")]
    NoPathFound,

    // IO errors
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    // Command execution errors
    #[error("Command execution failed: {0}")]
    CommandExecution(String),

    // Serialization errors
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    // Other errors
    #[error("Other error: {0}")]
    Other(String),
}

/// Result type for GOAP operations
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

    #[test]
    fn test_operation_failed_display() {
        let err = GoapError::OperationFailed("test operation".to_string());
        assert_eq!(format!("{}", err), "Operation failed: test operation");
    }

    #[test]
    fn test_io_error_conversion() {
        use std::io::{Error as IoError, ErrorKind};
        let io_error = IoError::new(ErrorKind::NotFound, "file not found");
        let err: GoapError = io_error.into();
        assert!(matches!(err, GoapError::Io(_)));
    }
}
