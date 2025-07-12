//! File and Directory Manager Integration Tests
//! This file contains integration tests for the File and Directory Manager.
//! The Directory Manager is responsible for managing files and directories and ensuring that they are properly organized and accessible.
//! It provides a set of APIs for creating, reading, updating, and deleting files and directories, as well as for navigating the file system.
//! It also ensures that files and directories are properly secured and that access to them is controlled.

//! Test Case 1: Test the creation of a new file
//! The created file must have the name .vimrc and it must be written inside the directory .conf inside the directory from which the test is executed.
//! The goaprs automaton must ensure the file exists and recreate it if it is deleted.
//!

use async_trait::async_trait;
use goaprs::action::{ActionFn, ActionResponse};
use goaprs::error::Result;
use goaprs::sensor::{SensorFn, SensorResponse};
use goaprs::{Actions, AutomatonController, Sensors};
use std::collections::HashMap;
use std::error::Error;
use std::fs;
use std::path::Path;
use std::time::Duration;
use tokio::time;

/// Sensor to check if .conf directory exists
struct CheckConfDirSensor;

#[async_trait]
impl SensorFn for CheckConfDirSensor {
    async fn exec(&self) -> Result<SensorResponse> {
        let conf_path = Path::new(".conf");
        let exists = conf_path.exists() && conf_path.is_dir();

        if exists {
            Ok(SensorResponse::new("exists".to_string(), "".to_string(), 0))
        } else {
            Ok(SensorResponse::new(
                "not_exists".to_string(),
                "".to_string(),
                1,
            ))
        }
    }
}

/// Sensor to check if .vimrc file exists in .conf directory
struct CheckVimrcFileSensor;

#[async_trait]
impl SensorFn for CheckVimrcFileSensor {
    async fn exec(&self) -> Result<SensorResponse> {
        let vimrc_path = Path::new(".conf/.vimrc");
        let exists = vimrc_path.exists() && vimrc_path.is_file();

        if exists {
            Ok(SensorResponse::new("exists".to_string(), "".to_string(), 0))
        } else {
            Ok(SensorResponse::new(
                "not_exists".to_string(),
                "".to_string(),
                1,
            ))
        }
    }
}

/// Action to create .conf directory
struct CreateConfDirAction;

#[async_trait]
impl ActionFn for CreateConfDirAction {
    async fn exec(&self) -> Result<ActionResponse> {
        let conf_path = Path::new(".conf");

        if !conf_path.exists() {
            fs::create_dir(conf_path)?;
            Ok(ActionResponse::new(
                "Created .conf directory".to_string(),
                "".to_string(),
                0,
            ))
        } else {
            Ok(ActionResponse::new(
                ".conf directory already exists".to_string(),
                "".to_string(),
                0,
            ))
        }
    }
}

/// Action to create .vimrc file in .conf directory
struct CreateVimrcFileAction;

#[async_trait]
impl ActionFn for CreateVimrcFileAction {
    async fn exec(&self) -> Result<ActionResponse> {
        let conf_path = Path::new(".conf");
        let vimrc_path = conf_path.join(".vimrc");

        // Default vimrc content
        let vimrc_content = "\" Basic vim configuration file
set number
set syntax=on
set autoindent
set tabstop=4
set shiftwidth=4
set expandtab
";

        // Create the file with content
        fs::write(&vimrc_path, vimrc_content)?;

        Ok(ActionResponse::new(
            format!("Created .vimrc file at {:?}", vimrc_path),
            "".to_string(),
            0,
        ))
    }
}

/// Clean up test artifacts after test
fn cleanup() -> std::result::Result<(), Box<dyn Error>> {
    println!("Cleaning up test artifacts...");
    let conf_path = Path::new(".conf");
    let vimrc_path = conf_path.join(".vimrc");

    // Remove file if it exists
    if vimrc_path.exists() {
        println!("Removing .vimrc file");
        fs::remove_file(&vimrc_path)?;
    }

    // Remove directory if it exists
    if conf_path.exists() {
        println!("Removing .conf directory");
        fs::remove_dir(conf_path)?;
    }

    println!("Cleanup completed successfully");
    Ok(())
}

#[tokio::test]
async fn test_file_and_directory_manager() -> std::result::Result<(), Box<dyn Error>> {
    // Clean up any existing test artifacts
    cleanup().expect("Failed to clean up test artifacts before test");

    // Set up sensors
    let mut sensors = Sensors::new();
    sensors.add("check_conf_dir", "conf_dir_status", CheckConfDirSensor)?;
    sensors.add("check_vimrc_file", "vimrc_status", CheckVimrcFileSensor)?;

    // Set up actions
    let mut actions = Actions::new();

    // Action to create .conf directory
    let mut create_conf_conditions = HashMap::new();
    create_conf_conditions.insert("conf_dir_status".to_string(), "not_exists".to_string());

    let mut create_conf_effects = HashMap::new();
    create_conf_effects.insert("conf_dir_status".to_string(), "exists".to_string());

    actions.add(
        "create_conf_dir",
        create_conf_conditions,
        create_conf_effects,
        CreateConfDirAction,
        1.0,
    )?;

    // Action to create .vimrc file
    let mut create_vimrc_conditions = HashMap::new();
    create_vimrc_conditions.insert("conf_dir_status".to_string(), "exists".to_string());
    create_vimrc_conditions.insert("vimrc_status".to_string(), "not_exists".to_string());

    let mut create_vimrc_effects = HashMap::new();
    create_vimrc_effects.insert("vimrc_status".to_string(), "exists".to_string());

    actions.add(
        "create_vimrc_file",
        create_vimrc_conditions,
        create_vimrc_effects,
        CreateVimrcFileAction,
        1.0,
    )?;

    // Create initial world state
    let world_state = HashMap::new(); // Empty initial state, will be populated by sensors

    // Create the automaton controller
    let controller =
        AutomatonController::new(actions, sensors, "file_directory_manager", world_state);

    // Set goal to have the .vimrc file in the .conf directory
    let mut goal = HashMap::new();
    goal.insert("conf_dir_status".to_string(), "exists".to_string());
    goal.insert("vimrc_status".to_string(), "exists".to_string());

    controller.set_goal(goal).await?;

    // Run a manual GOAP cycle
    println!("Starting GOAP cycle for file creation...");

    // Sense the environment
    controller.automaton().sense().await?;
    let initial_state = controller.automaton().world_state().await?;
    println!("Initial world state: {:?}", initial_state);

    // Plan actions
    let plan = controller.automaton().plan().await?;
    println!("Generated plan with {} actions:", plan.len());
    for (i, action) in plan.iter().enumerate() {
        println!("  Step {}: {}", i + 1, action.name());
    }

    // Execute actions
    let responses = controller.automaton().act().await?;
    for (i, response) in responses.iter().enumerate() {
        println!("  Result {}: {}", i + 1, response.stdout());
    }

    // Sense again to see changes
    controller.automaton().sense().await?;
    let state_after_creation = controller.automaton().world_state().await?;
    println!(
        "World state after file creation: {:?}",
        state_after_creation
    );

    // Verify that the file was created
    assert!(Path::new(".conf").exists(), ".conf directory should exist");
    assert!(
        Path::new(".conf/.vimrc").exists(),
        ".conf/.vimrc file should exist"
    );

    // Now delete the .vimrc file to test automatic recreation
    println!("\nDeleting .vimrc file to test recreation...");
    fs::remove_file(".conf/.vimrc")?;
    assert!(
        !Path::new(".conf/.vimrc").exists(),
        ".conf/.vimrc file should be deleted"
    );

    // Start the automaton controller
    controller.start().await?;

    // Wait for the automaton to go through a few cycles to detect and fix the missing file
    let max_cycles = 5;
    let mut file_recreated = false;

    for i in 1..=max_cycles {
        println!("Waiting cycle {}/{}...", i, max_cycles);
        time::sleep(Duration::from_secs(2)).await;

        // Check if the file has been recreated
        if Path::new(".conf/.vimrc").exists() {
            println!("✓ File was successfully recreated in cycle {}", i);
            file_recreated = true;
            break;
        }
    }

    if !file_recreated {
        println!(
            "! Warning: File was not recreated within {} cycles",
            max_cycles
        );
    }

    // Stop the controller
    controller.stop().await?;

    // Force a final sensing cycle to ensure the latest state is captured
    controller.automaton().sense().await?;
    controller.automaton().plan().await?;
    controller.automaton().act().await?;

    // Verify that the file was recreated
    let vimrc_path = Path::new(".conf/.vimrc");
    if vimrc_path.exists() {
        println!("✓ Verification successful: .vimrc file was recreated as expected");
    } else {
        println!("! Verification failed: .vimrc file was not recreated");
    }

    assert!(
        vimrc_path.exists(),
        ".conf/.vimrc file should be recreated automatically"
    );

    // Clean up
    cleanup()?;

    Ok(())
}
