use thiserror::Error;

/// Custom error types for GOAP
#[derive(Error, Debug)]
pub enum Error {
    #[error("Operation failed: {0}")]
    OperationFailed(String),

    // Sensor errors
    #[error("Sensor error: {0}")]
    Sensor(String),

    #[error("Sensor has multiple types")]
    SensorMultipleType,

    #[error("Sensor does not exist")]
    SensorDoesNotExist,

    #[error("Sensor already in collection: {0}")]
    SensorAlreadyInCollection(String),

    // Plan errors
    #[error("Plan error: {0}")]
    Plan(String),

    #[error("Failed to produce a plan")]
    PlanFailed,

    // Action errors
    #[error("Action error: {0}")]
    Action(String),

    #[error("Action has multiple types")]
    ActionMultipleType,

    #[error("Action already in collection: {0}")]
    ActionAlreadyInCollection(String),

    // IO errors
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    // Command execution errors
    #[error("Command execution failed: {0}")]
    CommandExecution(String),

    // Graph errors
    #[error("Graph error: {0}")]
    Graph(String),

    #[error("No path found in graph")]
    NoPathFound,

    // Serialization errors
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    // Other errors
    #[error("Other error: {0}")]
    Other(String),

    #[error("No plan has been generated yet")]
    NoPlanGenerated,
}

/// Result type for GOAP operations
pub type Result<T> = std::result::Result<T, Error>;
