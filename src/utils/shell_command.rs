use crate::error::{GoapError, Result};
use std::process::{Command, Output, Stdio};
use tokio::process::Command as AsyncCommand;

/// A utility for executing shell commands
pub struct ShellCommand {
    /// The command to execute
    command: String,
    /// The timeout in seconds
    timeout: u64,
    /// The last command output
    response: Option<(String, String, i32)>,
}

impl ShellCommand {
    /// Create a new shell command
    pub fn new(command: impl Into<String>, timeout: u64) -> Self {
        Self {
            command: command.into(),
            timeout,
            response: None,
        }
    }

    /// Execute the command synchronously
    ///
    /// # Returns
    ///
    /// A tuple containing (stdout, stderr, exit_status)
    ///
    /// # Errors
    ///
    /// Returns an error if the command fails to execute
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use goaprs::utils::shell_command::ShellCommand;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut cmd = ShellCommand::new("echo 'test'", 5);
    /// let (stdout, stderr, status) = cmd.run()?;
    /// assert_eq!(stdout.trim(), "test");
    /// assert!(stderr.is_empty());
    /// assert_eq!(status, 0);
    /// # Ok(())
    /// # }
    /// ```
    pub fn run(&mut self) -> Result<(String, String, i32)> {
        let command = self.command.clone();
        self.run_command(&command)
    }

    /// Execute a specific command synchronously
    pub fn run_command(&mut self, command: &str) -> Result<(String, String, i32)> {
        let output = Command::new("sh")
            .arg("-c")
            .arg(command)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .map_err(|e| {
                GoapError::CommandExecution(format!("Failed to execute command: {}", e))
            })?;

        let result = self.process_output(output);
        self.response = Some(result.clone());
        Ok(result)
    }

    /// Execute the command asynchronously
    pub async fn run_async(&mut self) -> Result<(String, String, i32)> {
        let command = self.command.clone();
        self.run_command_async(&command).await
    }

    /// Execute a specific command asynchronously
    pub async fn run_command_async(&mut self, command: &str) -> Result<(String, String, i32)> {
        let output = AsyncCommand::new("sh")
            .arg("-c")
            .arg(command)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .await
            .map_err(|e| {
                GoapError::CommandExecution(format!("Failed to execute command: {}", e))
            })?;

        let result = self.process_output(output);
        self.response = Some(result.clone());
        Ok(result)
    }

    /// Get the last response
    pub fn response(&self) -> Option<&(String, String, i32)> {
        self.response.as_ref()
    }

    /// Process the command output
    fn process_output(&self, output: Output) -> (String, String, i32) {
        let stdout = String::from_utf8_lossy(&output.stdout).to_string();
        let stderr = String::from_utf8_lossy(&output.stderr).to_string();
        let status = output.status.code().unwrap_or(-1);

        (stdout, stderr, status)
    }
}

impl Clone for ShellCommand {
    fn clone(&self) -> Self {
        Self {
            command: self.command.clone(),
            timeout: self.timeout,
            response: self.response.clone(),
        }
    }
}

impl std::fmt::Debug for ShellCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ShellCommand")
            .field("command", &self.command)
            .field("timeout", &self.timeout)
            .finish()
    }
}

/// Helper function for creating and executing a shell command in one step
pub async fn exec_shell_command(
    command: impl Into<String>,
    timeout: u64,
) -> Result<(String, String, i32)> {
    let mut cmd = ShellCommand::new(command, timeout);
    cmd.run_async().await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_shell_command_echo() {
        let mut cmd = ShellCommand::new("echo 'Hello World'", 5);
        let (stdout, stderr, status) = cmd.run_async().await.unwrap();

        assert_eq!(stdout.trim(), "Hello World");
        assert!(stderr.is_empty());
        assert_eq!(status, 0);
    }

    #[tokio::test]
    async fn test_shell_command_error() {
        let mut cmd = ShellCommand::new("command_that_does_not_exist", 5);
        let (stdout, stderr, status) = cmd.run_async().await.unwrap();

        assert!(stdout.is_empty());
        assert!(!stderr.is_empty());
        assert_ne!(status, 0);
    }

    #[test]
    fn test_sync_shell_command() {
        let mut cmd = ShellCommand::new("echo 'Sync Test'", 5);
        let (stdout, stderr, status) = cmd.run().unwrap();

        assert_eq!(stdout.trim(), "Sync Test");
        assert!(stderr.is_empty());
        assert_eq!(status, 0);
    }

    #[tokio::test]
    async fn test_helper_function() {
        let (stdout, stderr, status) = exec_shell_command("echo 'Helper Test'", 5).await.unwrap();

        assert_eq!(stdout.trim(), "Helper Test");
        assert!(stderr.is_empty());
        assert_eq!(status, 0);
    }
}
