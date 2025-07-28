//! This module contains integration tests for the ActorAutomatonController implementing a Dotfiles Manager automaton.
//! Test Case 1: dotfiles manager sync config from git repository
//! Context:
//! The goal of the automaton is to synchronize the dotfiles between the local machine and the remote repository.
//! The automaton will perform the following actions:
//! - Check if the dotfiles have changed locally
//! - Check if the dotfiles have changed remotely
//! - If the dotfiles have changed locally, update the remote repository
//! - If the dotfiles have changed remotely, update the local machine
//! - If the dotfiles have not changed, do nothing
//! The automaton uses the repository https://github.com/lepepe/dotfiles as source of truth.
//! References:
//! - https://github.com/lepepe/dotfiles
//!

use goaprs::utils::actor::{ActionFn, Fact, PlannerFn, SensorFn};
use goaprs::utils::shell_command::ShellCommand;
use goaprs::utils::ActorAutomatonController;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tempfile::TempDir;
use tokio::time::sleep;

// World state to track dotfiles status
#[derive(Debug, Clone)]
struct DotfilesState {
    local_changes: bool,
    remote_changes: bool,
    last_local_commit: String,
    last_remote_commit: String,
    local_path: PathBuf,
    repository_url: String,
    repository_initialized: bool,
}

impl DotfilesState {
    fn new(local_path: PathBuf, repository_url: &str) -> Self {
        Self {
            local_changes: false,
            remote_changes: false,
            last_local_commit: String::new(),
            last_remote_commit: String::new(),
            local_path,
            repository_url: repository_url.to_string(),
            repository_initialized: false,
        }
    }
}

// Sensors

/// Sensor to detect if the local dotfiles repository has been initialized
#[derive(Debug)]
struct RepoInitializedSensor {
    state: Arc<Mutex<DotfilesState>>,
}

impl RepoInitializedSensor {
    fn new(state: Arc<Mutex<DotfilesState>>) -> Self {
        Self { state }
    }
}

#[async_trait::async_trait]
impl SensorFn for RepoInitializedSensor {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> Vec<Fact> {
        let mut state = self.state.lock().unwrap();
        let git_dir = state.local_path.join(".git");

        state.repository_initialized = git_dir.exists() && git_dir.is_dir();

        vec![Fact::new(
            "repository_initialized",
            &state.repository_initialized.to_string(),
            "RepoInitializedSensor",
        )]
    }
}

/// Sensor to detect changes in local dotfiles
#[derive(Debug)]
struct LocalChangesSensor {
    state: Arc<Mutex<DotfilesState>>,
}

impl LocalChangesSensor {
    fn new(state: Arc<Mutex<DotfilesState>>) -> Self {
        Self { state }
    }

    fn get_local_commit(&self, path: &std::path::Path) -> String {
        let mut cmd = ShellCommand::new(format!("cd {} && git rev-parse HEAD", path.display()), 5);
        match cmd.run() {
            Ok((stdout, _, status)) if status == 0 => stdout.trim().to_string(),
            Ok((_, stderr, _)) => {
                println!("Error getting local commit: {}", stderr);
                String::new()
            }
            Err(e) => {
                println!("Command failed: {}", e);
                String::new()
            }
        }
    }

    fn has_uncommitted_changes(&self, path: &std::path::Path) -> bool {
        let mut cmd = ShellCommand::new(
            format!("cd {} && git status --porcelain", path.display()),
            5,
        );
        match cmd.run() {
            Ok((stdout, _, status)) if status == 0 => !stdout.trim().is_empty(),
            Ok((_, stderr, _)) => {
                println!("Error checking for uncommitted changes: {}", stderr);
                false
            }
            Err(e) => {
                println!("Command failed: {}", e);
                false
            }
        }
    }
}

#[async_trait::async_trait]
#[async_trait::async_trait]
impl SensorFn for LocalChangesSensor {
    async fn exec(&self, world_state: &HashMap<String, Fact>) -> Vec<Fact> {
        // Only check for changes if repo is initialized
        let repo_initialized = world_state
            .get("repository_initialized")
            .map_or(false, |f| f.data() == "true");

        if !repo_initialized {
            println!("Repository not initialized, reporting no local changes");
            return vec![Fact::new("local_changes", "false", "LocalChangesSensor")];
        }

        // Get state data with minimal lock time
        let path = {
            let state = self.state.lock().unwrap();
            state.local_path.clone()
        };

        // Execute synchronous commands outside the lock
        let current_commit = self.get_local_commit(&path);
        let has_uncommitted = self.has_uncommitted_changes(&path);

        let mut state = self.state.lock().unwrap();
        // Local changes exist if the commit hash changed or there are uncommitted changes
        state.local_changes = has_uncommitted
            || (!current_commit.is_empty()
                && !state.last_local_commit.is_empty()
                && current_commit != state.last_local_commit);

        if !current_commit.is_empty() {
            state.last_local_commit = current_commit;
        }

        vec![Fact::new(
            "local_changes",
            &state.local_changes.to_string(),
            "LocalChangesSensor",
        )]
    }
}

/// Sensor to detect changes in remote repository
#[derive(Debug)]
struct RemoteChangesSensor {
    state: Arc<Mutex<DotfilesState>>,
}

impl RemoteChangesSensor {
    fn new(state: Arc<Mutex<DotfilesState>>) -> Self {
        Self { state }
    }

    fn fetch_and_get_remote_commit(&self, path: &std::path::Path) -> String {
        // Fetch from remote to get latest changes
        println!("Fetching from remote repository...");
        let mut fetch_cmd = ShellCommand::new(format!("cd {} && git fetch", path.display()), 20);
        match fetch_cmd.run() {
            Ok((_, _, status)) if status == 0 => {
                println!("Fetch completed successfully");
            }
            Ok((_, stderr, _)) => {
                println!("Fetch completed with error: {}", stderr);
                return String::new();
            }
            Err(e) => {
                println!("Fetch failed: {}", e);
                return String::new();
            }
        }

        // Get the remote commit hash
        let mut cmd = ShellCommand::new(
            format!("cd {} && git rev-parse origin/main", path.display()),
            5,
        );
        match cmd.run() {
            Ok((stdout, _, status)) if status == 0 => stdout.trim().to_string(),
            Ok((_, stderr, _)) => {
                println!("Error getting remote commit: {}", stderr);
                String::new()
            }
            Err(e) => {
                println!("Command failed: {}", e);
                String::new()
            }
        }
    }
}

#[async_trait::async_trait]
impl SensorFn for RemoteChangesSensor {
    async fn exec(&self, world_state: &HashMap<String, Fact>) -> Vec<Fact> {
        // Only check for changes if repo is initialized
        let repo_initialized = world_state
            .get("repository_initialized")
            .map_or(false, |f| f.data() == "true");

        if !repo_initialized {
            println!("Repository not initialized, reporting no remote changes");
            return vec![Fact::new("remote_changes", "false", "RemoteChangesSensor")];
        }

        // Get path with minimal lock time
        let path = {
            let state = self.state.lock().unwrap();
            state.local_path.clone()
        };

        // Get remote commit without holding lock
        let current_remote_commit = self.fetch_and_get_remote_commit(&path);

        let mut state = self.state.lock().unwrap();
        // Remote changes exist if the remote commit hash differs from our last known remote commit
        state.remote_changes = !current_remote_commit.is_empty()
            && !state.last_remote_commit.is_empty()
            && current_remote_commit != state.last_remote_commit;

        if !current_remote_commit.is_empty() {
            state.last_remote_commit = current_remote_commit;
        }

        vec![Fact::new(
            "remote_changes",
            &state.remote_changes.to_string(),
            "RemoteChangesSensor",
        )]
    }
}

// Actions

/// Action to clone the repository if it doesn't exist
struct CloneRepositoryAction {
    state: Arc<Mutex<DotfilesState>>,
}

impl CloneRepositoryAction {
    fn new(state: Arc<Mutex<DotfilesState>>) -> Self {
        Self { state }
    }
}

#[async_trait::async_trait]
impl ActionFn for CloneRepositoryAction {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> bool {
        println!("Executing: Clone repository action");

        let state_guard = self.state.lock().unwrap();
        let local_path = state_guard.local_path.clone();
        let repo_url = state_guard.repository_url.clone();
        drop(state_guard); // Release the lock before running command

        // Remove directory if it exists to ensure clean clone
        if local_path.exists() {
            match fs::remove_dir_all(&local_path) {
                Ok(_) => println!("Removed existing directory"),
                Err(e) => {
                    println!("Failed to remove directory: {}", e);
                    return false;
                }
            }
        }

        // Create parent directories if needed
        if let Some(parent) = local_path.parent() {
            match fs::create_dir_all(parent) {
                Ok(_) => {}
                Err(e) => {
                    println!("Failed to create parent directories: {}", e);
                    return false;
                }
            }
        }

        // Clone the repository
        println!(
            "Cloning repository from {} to {}",
            repo_url,
            local_path.display()
        );
        let start_time = Instant::now();

        let mut cmd = ShellCommand::new(
            format!("git clone {} {}", repo_url, local_path.display()),
            60, // Allow up to 60 seconds for clone
        );

        match cmd.run() {
            Ok((_, _stderr, status)) if status == 0 => {
                println!(
                    "Repository cloned successfully in {:?}",
                    start_time.elapsed()
                );

                // Update state
                let mut state = self.state.lock().unwrap();
                state.repository_initialized = true;
                drop(state); // Release lock before next command

                // Get latest commit
                let mut commit_cmd = ShellCommand::new(
                    format!("cd {} && git rev-parse HEAD", local_path.display()),
                    5,
                );

                if let Ok((stdout, _, status)) = commit_cmd.run() {
                    if status == 0 {
                        let commit = stdout.trim().to_string();
                        let mut state = self.state.lock().unwrap();
                        state.last_local_commit = commit.clone();
                        state.last_remote_commit = commit;
                    }
                }

                true
            }
            Ok((_, stderr, _)) => {
                println!("Failed to clone repository: {}", stderr);
                false
            }
            Err(e) => {
                println!("Error executing git clone: {}", e);
                false
            }
        }
    }
}

/// Action to pull changes from remote
struct PullRemoteChangesAction {
    state: Arc<Mutex<DotfilesState>>,
}

impl PullRemoteChangesAction {
    fn new(state: Arc<Mutex<DotfilesState>>) -> Self {
        Self { state }
    }
}

#[async_trait::async_trait]
impl ActionFn for PullRemoteChangesAction {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> bool {
        println!("Executing: Pull remote changes action");

        let state_guard = self.state.lock().unwrap();
        let local_path = state_guard.local_path.clone();
        drop(state_guard);

        // Pull the latest changes with timeout
        println!("Pulling changes from remote repository...");
        let start_time = Instant::now();

        let mut cmd = ShellCommand::new(
            format!("cd {} && git pull origin main", local_path.display()),
            30, // 30 seconds timeout for network operation
        );

        match cmd.run() {
            Ok((_, _stderr, status)) if status == 0 => {
                println!("Pulled changes successfully in {:?}", start_time.elapsed());

                // Update state
                let mut state = self.state.lock().unwrap();
                state.remote_changes = false;
                drop(state); // Release lock before next command

                // Get latest commit
                let mut commit_cmd = ShellCommand::new(
                    format!("cd {} && git rev-parse HEAD", local_path.display()),
                    5,
                );

                if let Ok((stdout, _, status)) = commit_cmd.run() {
                    if status == 0 {
                        let commit = stdout.trim().to_string();
                        let mut state = self.state.lock().unwrap();
                        state.last_local_commit = commit.clone();
                        state.last_remote_commit = commit;
                    }
                }

                true
            }
            Ok((_, stderr, _)) => {
                println!("Failed to pull changes: {}", stderr);
                false
            }
            Err(e) => {
                println!("Error executing git pull: {}", e);
                false
            }
        }
    }
}

/// Action to push local changes to remote
struct PushLocalChangesAction {
    state: Arc<Mutex<DotfilesState>>,
}

impl PushLocalChangesAction {
    fn new(state: Arc<Mutex<DotfilesState>>) -> Self {
        Self { state }
    }
}

#[async_trait::async_trait]
impl ActionFn for PushLocalChangesAction {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> bool {
        println!("Executing: Push local changes action");

        let state_guard = self.state.lock().unwrap();
        let local_path = state_guard.local_path.clone();
        drop(state_guard);

        // Check for uncommitted changes
        println!("Checking for uncommitted changes...");
        let mut status_cmd = ShellCommand::new(
            format!("cd {} && git status --porcelain", local_path.display()),
            5,
        );

        let has_uncommitted = match status_cmd.run() {
            Ok((stdout, _, status)) if status == 0 => !stdout.trim().is_empty(),
            Ok((_, stderr, _)) => {
                println!("Git status command failed: {}", stderr);
                false
            }
            Err(e) => {
                println!("Error checking git status: {}", e);
                false
            }
        };

        if has_uncommitted {
            // Add all changes
            println!("Adding changes to git...");
            let mut add_cmd =
                ShellCommand::new(format!("cd {} && git add .", local_path.display()), 10);

            match add_cmd.run() {
                Ok((_, _stderr, status)) if status == 0 => {
                    println!("Changes added successfully");
                }
                Ok((_, stderr, _)) => {
                    println!("Failed to add changes: {}", stderr);
                    return false;
                }
                Err(e) => {
                    println!("Error adding files: {}", e);
                    return false;
                }
            }

            // Commit changes
            println!("Committing changes...");
            let mut commit_cmd = ShellCommand::new(
                format!(
                    "cd {} && git commit -m \"Auto-sync dotfiles\"",
                    local_path.display()
                ),
                10,
            );

            match commit_cmd.run() {
                Ok((_, _stderr, status)) if status == 0 => {
                    println!("Changes committed successfully");
                }
                Ok((_, stderr, _)) => {
                    println!("Failed to commit changes: {}", stderr);
                    return false;
                }
                Err(e) => {
                    println!("Error committing changes: {}", e);
                    return false;
                }
            }
        }

        // Push changes
        println!("Pushing changes to remote repository...");
        let start_time = Instant::now();

        let mut push_cmd = ShellCommand::new(
            format!("cd {} && git push origin main", local_path.display()),
            30, // 30 seconds timeout for network operation
        );

        match push_cmd.run() {
            Ok((_, _stderr, status)) if status == 0 => {
                println!("Pushed changes successfully in {:?}", start_time.elapsed());

                // Update state
                let mut state = self.state.lock().unwrap();
                state.local_changes = false;
                drop(state); // Release lock before next command

                // Get latest commit
                let mut commit_cmd = ShellCommand::new(
                    format!("cd {} && git rev-parse HEAD", local_path.display()),
                    5,
                );

                if let Ok((stdout, _, status)) = commit_cmd.run() {
                    if status == 0 {
                        let commit = stdout.trim().to_string();
                        let mut state = self.state.lock().unwrap();
                        state.last_local_commit = commit.clone();
                        state.last_remote_commit = commit;
                    }
                }

                true
            }
            Ok((_, stderr, _)) => {
                println!("Failed to push changes: {}", stderr);
                false
            }
            Err(e) => {
                println!("Error executing git push: {}", e);
                false
            }
        }
    }
}

// Helper struct to identify actions
struct ActionWrapper {
    id: &'static str,
    action: Arc<dyn ActionFn>,
}

impl ActionWrapper {
    fn new(id: &'static str, action: Arc<dyn ActionFn>) -> Self {
        Self { id, action }
    }
}

// Planner
struct DotfilesPlanner;

#[async_trait::async_trait]
impl PlannerFn for DotfilesPlanner {
    async fn plan(
        &self,
        world_state: &HashMap<String, Fact>,
        goal: &HashMap<String, Fact>,
        available_actions: &[Arc<dyn ActionFn>],
    ) -> Vec<Arc<dyn ActionFn>> {
        println!("Planning dotfiles synchronization...");
        println!("World state keys: {:?}", world_state.keys());

        // Create wrapped actions for identification
        let action_wrappers = vec![
            ActionWrapper::new("CloneRepositoryAction", available_actions[0].clone()),
            ActionWrapper::new("PullRemoteChangesAction", available_actions[1].clone()),
            ActionWrapper::new("PushLocalChangesAction", available_actions[2].clone()),
        ];

        let mut plan = Vec::new();

        // Check if we need to sync dotfiles
        if goal.contains_key("dotfiles_in_sync") {
            // First check if repository is initialized
            let repo_initialized = world_state
                .get("repository_initialized")
                .map(|f| f.data() == "true")
                .unwrap_or(false);

            if !repo_initialized {
                // Repository needs to be cloned first
                if let Some(clone_wrapper) = action_wrappers
                    .iter()
                    .find(|a| a.id == "CloneRepositoryAction")
                {
                    plan.push(clone_wrapper.action.clone());
                    return plan; // Return early as this needs to be done first
                }
            }

            // Check for remote changes
            let remote_changes = world_state
                .get("remote_changes")
                .map(|f| f.data() == "true")
                .unwrap_or(false);

            let local_changes = world_state
                .get("local_changes")
                .map(|f| f.data() == "true")
                .unwrap_or(false);

            // If both local and remote have changes, prioritize pulling first
            if remote_changes {
                if let Some(pull_wrapper) = action_wrappers
                    .iter()
                    .find(|a| a.id == "PullRemoteChangesAction")
                {
                    plan.push(pull_wrapper.action.clone());
                }
            }

            // Then push any local changes
            if local_changes {
                if let Some(push_wrapper) = action_wrappers
                    .iter()
                    .find(|a| a.id == "PushLocalChangesAction")
                {
                    plan.push(push_wrapper.action.clone());
                }
            }
        }

        println!("Plan created with {} steps", plan.len());
        plan
    }
}

// Main test function
#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_dotfiles_manager() {
        println!("Starting Dotfiles Manager Automaton Test");

        // Create a temporary directory for the dotfiles
        let temp_dir = TempDir::new().expect("Failed to create temp directory");
        let dotfiles_path = temp_dir.path().join("dotfiles");

        // The repository URL
        let repo_url = "https://github.com/lepepe/dotfiles";

        // Create a local task set for actor-based tasks
        let local = tokio::task::LocalSet::new();

        local
            .run_until(async move {
                // Create shared world state
                let world_state = Arc::new(Mutex::new(DotfilesState::new(
                    dotfiles_path.clone(),
                    repo_url,
                )));

                // Create sensors
                let sensors = vec![
                    Arc::new(RepoInitializedSensor::new(Arc::clone(&world_state)))
                        as Arc<dyn SensorFn>,
                    Arc::new(LocalChangesSensor::new(Arc::clone(&world_state)))
                        as Arc<dyn SensorFn>,
                    Arc::new(RemoteChangesSensor::new(Arc::clone(&world_state)))
                        as Arc<dyn SensorFn>,
                ];

                // Create actions
                let actions = vec![
                    Arc::new(CloneRepositoryAction::new(Arc::clone(&world_state)))
                        as Arc<dyn ActionFn>,
                    Arc::new(PullRemoteChangesAction::new(Arc::clone(&world_state)))
                        as Arc<dyn ActionFn>,
                    Arc::new(PushLocalChangesAction::new(Arc::clone(&world_state)))
                        as Arc<dyn ActionFn>,
                ];

                // Create planner
                let planner = Arc::new(DotfilesPlanner) as Arc<dyn PlannerFn>;

                // Create controller
                let controller =
                    ActorAutomatonController::new("DotfilesManager", sensors, actions, planner);

                // Set a goal to keep dotfiles in sync
                let mut goal = HashMap::new();
                goal.insert(
                    "dotfiles_in_sync".to_string(),
                    Fact::new("dotfiles_in_sync", "true", "UserGoal"),
                );
                controller.set_goal(goal).await;

                // Start the automaton
                controller.start().await;

                // Run for a few cycles to let the automaton work
                for cycle in 1..=3 {
                    println!("\n--- Cycle {} ---", cycle);

                    // Get the current world state for display
                    let state = controller.world_state().await;
                    println!("World state keys: {:?}", state.keys());
                    for (key, fact) in state.iter() {
                        println!("{}: {}", key, fact.data());
                    }

                    // In cycle 2, simulate local changes if repo was initialized
                    if cycle == 2 {
                        let state = world_state.lock().unwrap();
                        if state.repository_initialized {
                            // Simulate a local change by creating a test file
                            let test_file_path = state.local_path.join("test_change.txt");
                            drop(state);

                            match fs::write(&test_file_path, "This is a test change") {
                                Ok(_) => println!("Created test file to simulate local changes"),
                                Err(e) => println!("Failed to create test file: {}", e),
                            }
                        }
                    }

                    // Wait before next cycle, give more time for operations to complete
                    sleep(Duration::from_secs(5)).await;
                }

                // Stop the automaton when done
                controller.stop().await;
                println!("Shutting down Dotfiles Manager Automaton");

                // Clean up
                temp_dir.close().expect("Failed to clean up temp directory");
            })
            .await;
    }
}
