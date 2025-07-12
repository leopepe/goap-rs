use std::process::{Command, Output, Stdio};
use tokio::process::Command as AsyncCommand;

use crate::error::{Error, Result};

/// A utility for executing shell commands
///
/// The `ShellCommand` struct provides a convenient way to execute shell commands
/// both synchronously and asynchronously.
///
/// # Examples
///
/// ```
/// use goaprs::utils::shell_command::ShellCommand;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// // Create a new shell command
/// let mut cmd = ShellCommand::new("echo 'Hello World'", 5);
///
/// // Run synchronously
/// let (stdout, stderr, status) = cmd.run()?;
/// assert_eq!(stdout.trim(), "Hello World");
/// assert_eq!(status, 0);
/// # Ok(())
/// # }
/// ```
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
    ///
    /// # Arguments
    ///
    /// * `command` - The command to execute
    /// * `timeout` - The timeout in seconds
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::utils::shell_command::ShellCommand;
    ///
    /// let cmd = ShellCommand::new("ls -la", 10);
    /// ```
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
    /// ```
    /// use goaprs::utils::shell_command::ShellCommand;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut cmd = ShellCommand::new("echo 'test'", 5);
    /// let (stdout, stderr, status) = cmd.run()?;
    /// assert_eq!(stdout.trim(), "test");
    /// # Ok(())
    /// # }
    /// ```
    pub fn run(&mut self) -> Result<(String, String, i32)> {
        let command = self.command.clone();
        self.run_command(&command)
    }

    /// Execute a specific command synchronously
    ///
    /// This method allows executing a different command than the one
    /// originally provided to the ShellCommand.
    ///
    /// # Arguments
    ///
    /// * `command` - The command to execute
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
    /// ```
    /// use goaprs::utils::shell_command::ShellCommand;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut cmd = ShellCommand::new("echo 'original'", 5);
    /// let (stdout, _, _) = cmd.run_command("echo 'override'")?;
    /// assert_eq!(stdout.trim(), "override");
    /// # Ok(())
    /// # }
    /// ```
    pub fn run_command(&mut self, command: &str) -> Result<(String, String, i32)> {
        let output = Command::new("sh")
            .arg("-c")
            .arg(command)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .map_err(|e| Error::CommandExecution(format!("Failed to execute command: {}", e)))?;

        let result = self.process_output(output);
        self.response = Some(result.clone());
        Ok(result)
    }

    /// Execute the command asynchronously
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
    /// ```
    /// use goaprs::utils::shell_command::ShellCommand;
    ///
    /// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut cmd = ShellCommand::new("echo 'async test'", 5);
    /// let (stdout, stderr, status) = cmd.run_async().await?;
    /// assert_eq!(stdout.trim(), "async test");
    /// # Ok(())
    /// # }
    /// ```
    pub async fn run_async(&mut self) -> Result<(String, String, i32)> {
        let command = self.command.clone();
        self.run_command_async(&command).await
    }

    /// Execute a specific command asynchronously
    ///
    /// This method allows executing a different command than the one
    /// originally provided to the ShellCommand.
    ///
    /// # Arguments
    ///
    /// * `command` - The command to execute
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
    /// ```
    /// use goaprs::utils::shell_command::ShellCommand;
    ///
    /// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut cmd = ShellCommand::new("echo 'original'", 5);
    /// let (stdout, _, _) = cmd.run_command_async("echo 'async override'").await?;
    /// assert_eq!(stdout.trim(), "async override");
    /// # Ok(())
    /// # }
    /// ```
    pub async fn run_command_async(&mut self, command: &str) -> Result<(String, String, i32)> {
        let output = AsyncCommand::new("sh")
            .arg("-c")
            .arg(command)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .await
            .map_err(|e| Error::CommandExecution(format!("Failed to execute command: {}", e)))?;

        let result = self.process_output(output);
        self.response = Some(result.clone());
        Ok(result)
    }

    /// Get the last response
    ///
    /// Returns the output from the last executed command, if any.
    ///
    /// # Returns
    ///
    /// An optional reference to a tuple containing (stdout, stderr, exit_status)
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::utils::shell_command::ShellCommand;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut cmd = ShellCommand::new("echo 'test response'", 5);
    /// cmd.run()?;
    ///
    /// if let Some((stdout, _, _)) = cmd.response() {
    ///     assert_eq!(stdout.trim(), "test response");
    /// }
    /// # Ok(())
    /// # }
    /// ```
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
///
/// This is a convenience function that creates a new `ShellCommand` and immediately
/// executes it asynchronously.
///
/// # Arguments
///
/// * `command` - The command to execute
/// * `timeout` - The timeout in seconds
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
/// ```
/// use goaprs::utils::shell_command::exec_shell_command;
///
/// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
/// let (stdout, stderr, status) = exec_shell_command("echo 'one step'", 5).await?;
/// assert_eq!(stdout.trim(), "one step");
/// # Ok(())
/// # }
/// ```
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
