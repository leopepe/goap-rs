use thiserror::Error;

#[derive(Error, Debug)]
pub enum GoapError {
    #[error("No valid plan found to achieve the goal")]
    NoPlanFound,
    #[error("Invalid state transition: {0}")]
    InvalidStateTransition(String),
    #[error("Action precondition not met: {0}")]
    PreconditionNotMet(String),
    #[error("Action cost must be positive")]
    InvalidActionCost,
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

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
