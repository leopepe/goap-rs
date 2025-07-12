//! Sensor module for the GOAP (Goal-Oriented Action Planning) system.
//!
//! Sensors are components that gather information from the environment to update
//! the world state. They represent the perception system of an agent, allowing it to
//! observe changes in its environment and make decisions based on current conditions.
//!
//! This module provides:
//! - [`SensorResponse`]: Represents the data returned from a sensor execution
//! - [`SensorFn`]: Trait for implementing custom sensor functionality
//! - [`Sensor`]: Concrete sensor implementation that wraps a `SensorFn` implementation
//! - [`Sensors`]: Collection for managing multiple sensors
//! - [`FnSensor`]: Helper for creating sensors from closures
//!
//! # Example
//!
//! ```
//! use std::collections::HashMap;
//! use goaprs::sensor::{Sensors, SensorFn, SensorResponse};
//! use goaprs::error::Result;
//! use async_trait::async_trait;
//!
//! // Create a custom temperature sensor
//! struct TemperatureSensor;
//!
//! #[async_trait]
//! impl SensorFn for TemperatureSensor {
//!     async fn exec(&self) -> Result<SensorResponse> {
//!         // In a real application, this would read from a thermometer or API
//!         Ok(SensorResponse::new("72.5°F".to_string(), "".to_string(), 0))
//!     }
//! }
//!
//! // Create a collection of sensors
//! let mut sensors = Sensors::new();
//!
//! // Add our temperature sensor
//! sensors.add("temperature", "room_temp", TemperatureSensor).unwrap();
//! ```

use std::fmt;
use std::future::Future;
use std::sync::{Arc, Mutex};

use async_trait::async_trait;

use crate::error::{GoapError, Result};

/// Represents the response from a sensor execution.
///
/// A `SensorResponse` contains:
/// - The standard output (stdout) from the sensor
/// - The standard error (stderr) from the sensor (if any)
/// - A return code indicating success (0) or failure (non-zero)
///
/// This structure standardizes the format of sensor data regardless of the
/// underlying implementation.
///
/// # Examples
///
/// ```
/// use goaprs::sensor::SensorResponse;
///
/// // Create a response with standard output
/// let response = SensorResponse::new("Temperature: 72°F".to_string(), "".to_string(), 0);
/// assert_eq!(response.stdout(), "Temperature: 72°F");
/// assert_eq!(response.response(), "Temperature: 72°F");
/// assert!(response.is_success());
/// ```
#[derive(Debug, Clone)]
pub struct SensorResponse {
    stdout: String,
    stderr: String,
    return_code: i32,
}

impl SensorResponse {
    /// Creates a new sensor response with the given outputs and return code.
    ///
    /// Automatically trims any trailing newlines from stdout and stderr.
    ///
    /// # Arguments
    ///
    /// * `stdout` - The standard output from the sensor
    /// * `stderr` - The standard error from the sensor
    /// * `return_code` - The return code (0 for success, non-zero for failure)
    ///
    /// # Returns
    ///
    /// A new `SensorResponse` instance
    pub fn new(stdout: String, stderr: String, return_code: i32) -> Self {
        Self {
            stdout: Self::trim(&stdout),
            stderr: Self::trim(&stderr),
            return_code,
        }
    }

    /// Gets the standard output (stdout) from the sensor execution.
    ///
    /// # Returns
    ///
    /// A string slice containing the stdout data
    pub fn stdout(&self) -> &str {
        &self.stdout
    }

    /// Gets the standard error (stderr) output from the sensor execution.
    ///
    /// # Returns
    ///
    /// A string slice containing the stderr data
    pub fn stderr(&self) -> &str {
        &self.stderr
    }

    /// Gets the return code from the sensor execution.
    ///
    /// # Returns
    ///
    /// An integer representing the return code (0 for success, non-zero for failure)
    pub fn return_code(&self) -> i32 {
        self.return_code
    }

    /// Gets the response content, preferring stdout if available, falling back to stderr otherwise.
    ///
    /// This method provides a convenient way to access the most relevant output
    /// from the sensor without having to check stdout and stderr separately.
    ///
    /// # Returns
    ///
    /// A string slice containing the sensor's output
    pub fn response(&self) -> &str {
        if !self.stdout.is_empty() {
            &self.stdout
        } else {
            &self.stderr
        }
    }

    /// Checks if the response was successful by examining the return code.
    ///
    /// A return code of 0 indicates success, anything else indicates failure.
    ///
    /// # Returns
    ///
    /// `true` if the return code is 0, `false` otherwise
    pub fn is_success(&self) -> bool {
        self.return_code == 0
    }

    /// Trims the given string by removing trailing newlines.
    ///
    /// This helps standardize the output format by removing unnecessary line endings.
    ///
    /// # Arguments
    ///
    /// * `s` - The string to trim
    ///
    /// # Returns
    ///
    /// A new String with trailing newlines removed
    fn trim(s: &str) -> String {
        s.trim_end_matches("\r\n")
            .trim_end_matches('\n')
            .to_string()
    }
}

impl fmt::Display for SensorResponse {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Response: {}, ReturnCode: {}",
            self.response(),
            self.return_code
        )
    }
}

/// Trait for sensor implementations that can gather data from the environment.
///
/// Implementors of this trait provide the core functionality for sensors in the GOAP system.
/// A `SensorFn` must be able to execute asynchronously and return a `SensorResponse` or an error.
///
/// This trait uses `async_trait` to enable async functions in traits.
///
/// # Examples
///
/// ```
/// use goaprs::sensor::{SensorFn, SensorResponse};
/// use goaprs::error::Result;
/// use async_trait::async_trait;
///
/// struct WeatherSensor;
///
/// #[async_trait]
/// impl SensorFn for WeatherSensor {
///     async fn exec(&self) -> Result<SensorResponse> {
///         // In a real app, this would call a weather API
///         Ok(SensorResponse::new("Sunny, 75°F".to_string(), "".to_string(), 0))
///     }
/// }
/// ```
#[async_trait]
pub trait SensorFn: Send + Sync {
    /// Execute the sensor and return the response
    async fn exec(&self) -> Result<SensorResponse>;
}

/// A sensor that gathers information about the world for use in GOAP planning.
///
/// A `Sensor` wraps a `SensorFn` implementation and provides additional metadata
/// such as name and binding key. The binding key is used to map the sensor's output
/// to a specific key in the world state.
///
/// Sensors can be executed to gather data, and their responses are cached for later access.
///
/// # Examples
///
/// ```
/// use goaprs::sensor::{Sensor, SensorFn, SensorResponse};
/// use goaprs::error::Result;
/// use async_trait::async_trait;
///
/// struct LightSensor;
///
/// #[async_trait]
/// impl SensorFn for LightSensor {
///     async fn exec(&self) -> Result<SensorResponse> {
///         Ok(SensorResponse::new("on".to_string(), "".to_string(), 0))
///     }
/// }
///
/// # #[tokio::main]
/// # async fn main() -> Result<()> {
/// // Create a sensor
/// let sensor = Sensor::new("living_room_light", "light_status", LightSensor);
///
/// // Execute the sensor to gather data
/// let response = sensor.exec().await?;
/// assert_eq!(response.stdout(), "on");
///
/// // The sensor name and binding can be used to identify it
/// assert_eq!(sensor.name(), "living_room_light");
/// assert_eq!(sensor.binding(), "light_status");
/// # Ok(())
/// # }
/// ```
#[derive(Clone)]
pub struct Sensor {
    /// The name of the sensor
    name: String,
    /// The binding key used to store the sensor's result in the world state
    binding: String,
    /// The function that executes the sensor
    func: Arc<dyn SensorFn>,
    /// The last response from the sensor
    response: Arc<Mutex<Option<SensorResponse>>>,
}

impl Sensor {
    /// Creates a new sensor with the specified name, binding key, and sensor function.
    ///
    /// # Arguments
    ///
    /// * `name` - A unique name for the sensor
    /// * `binding` - The key that will be used to store the sensor's data in the world state
    /// * `func` - The implementation of the sensor functionality
    ///
    /// # Returns
    ///
    /// A new `Sensor` instance
    pub fn new<F>(name: impl Into<String>, binding: impl Into<String>, func: F) -> Self
    where
        F: SensorFn + 'static,
    {
        Self {
            name: name.into(),
            binding: binding.into(),
            func: Arc::new(func),
            response: Arc::new(Mutex::new(None)),
        }
    }

    /// Gets the name of the sensor.
    ///
    /// # Returns
    ///
    /// A string slice containing the sensor's name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Gets the binding key of the sensor.
    ///
    /// The binding key is used to map the sensor's output to a specific key in the world state.
    ///
    /// # Returns
    ///
    /// A string slice containing the sensor's binding key
    pub fn binding(&self) -> &str {
        &self.binding
    }

    /// Executes the sensor, stores the response, and returns it.
    ///
    /// This method invokes the underlying sensor function, caches the response
    /// for later retrieval, and returns a copy of the response.
    ///
    /// # Returns
    ///
    /// A `Result` containing either a `SensorResponse` or an `Error`
    ///
    /// # Errors
    ///
    /// This method will return an error if:
    /// - The sensor function execution fails
    /// - The response mutex is poisoned
    pub async fn exec(&self) -> Result<SensorResponse> {
        let response = self.func.exec().await?;
        let mut resp_lock = self
            .response
            .lock()
            .map_err(|_| GoapError::Other("Lock poisoned".into()))?;
        *resp_lock = Some(response.clone());

        // Return a clone of the response
        Ok(response)
    }

    /// Gets the last response from the sensor, if any.
    ///
    /// This method returns the cached response from the most recent execution,
    /// or `None` if the sensor has not been executed yet.
    ///
    /// # Returns
    ///
    /// A `Result` containing either an `Option<SensorResponse>` or an `Error`
    ///
    /// # Errors
    ///
    /// This method will return an error if the response mutex is poisoned
    pub fn response(&self) -> Result<Option<SensorResponse>> {
        let resp_lock = self
            .response
            .lock()
            .map_err(|_| GoapError::Other("Lock poisoned".into()))?;
        Ok(resp_lock.clone())
    }
}

impl fmt::Debug for Sensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Sensor")
            .field("name", &self.name)
            .field("binding", &self.binding)
            .finish()
    }
}

impl fmt::Display for Sensor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.name)
    }
}

/// A collection of sensors that can be managed and executed as a group.
///
/// The `Sensors` collection provides methods for adding, retrieving, and removing
/// sensors, as well as executing all sensors in the collection simultaneously.
///
/// # Examples
///
/// ```
/// use goaprs::sensor::{Sensors, SensorFn, SensorResponse};
/// use goaprs::error::Result;
/// use async_trait::async_trait;
///
/// struct TemperatureSensor;
///
/// #[async_trait]
/// impl SensorFn for TemperatureSensor {
///     async fn exec(&self) -> Result<SensorResponse> {
///         Ok(SensorResponse::new("72°F".to_string(), "".to_string(), 0))
///     }
/// }
///
/// struct HumiditySensor;
///
/// #[async_trait]
/// impl SensorFn for HumiditySensor {
///     async fn exec(&self) -> Result<SensorResponse> {
///         Ok(SensorResponse::new("45%".to_string(), "".to_string(), 0))
///     }
/// }
///
/// # #[tokio::main]
/// # async fn main() -> Result<()> {
/// // Create a collection of sensors
/// let mut sensors = Sensors::new();
///
/// // Add sensors to the collection
/// sensors.add("temperature", "room_temp", TemperatureSensor)?;
/// sensors.add("humidity", "room_humidity", HumiditySensor)?;
///
/// // Execute all sensors at once
/// let responses = sensors.run_all().await?;
/// assert_eq!(responses.len(), 2);
///
/// // Access individual sensors
/// let temp_sensor = sensors.get("temperature").unwrap();
/// let response = temp_sensor.exec().await?;
/// assert_eq!(response.stdout(), "72°F");
/// # Ok(())
/// # }
/// ```
#[derive(Clone, Default)]
pub struct Sensors {
    sensors: Vec<Sensor>,
}

impl Sensors {
    /// Creates a new empty sensor collection.
    ///
    /// # Returns
    ///
    /// A new empty `Sensors` collection
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
        }
    }

    /// Creates a sensor collection from a vector of sensors.
    ///
    /// # Arguments
    ///
    /// * `sensors` - A vector of `Sensor` instances
    ///
    /// # Returns
    ///
    /// A new `Sensors` collection initialized with the provided sensors
    pub fn from_vec(sensors: Vec<Sensor>) -> Self {
        Self { sensors }
    }

    /// Adds a new sensor to the collection.
    ///
    /// Creates a new sensor with the provided name, binding, and function,
    /// and adds it to the collection. If a sensor with the same name already
    /// exists, an error is returned.
    ///
    /// # Arguments
    ///
    /// * `name` - A unique name for the sensor
    /// * `binding` - The key that will be used to store the sensor's data in the world state
    /// * `func` - The implementation of the sensor functionality
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// This method will return an error if a sensor with the same name already exists
    pub fn add<F>(
        &mut self,
        name: impl Into<String>,
        binding: impl Into<String>,
        func: F,
    ) -> Result<()>
    where
        F: SensorFn + 'static,
    {
        let name_str = name.into();

        // Check if a sensor with this name already exists
        if self.get(&name_str).is_some() {
            return Err(GoapError::SensorAlreadyInCollection(name_str));
        }

        // Add the new sensor
        self.sensors.push(Sensor::new(name_str, binding, func));
        Ok(())
    }

    /// Gets a sensor by name.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the sensor to retrieve
    ///
    /// # Returns
    ///
    /// An `Option` containing a reference to the sensor if found, or `None` if not found
    pub fn get(&self, name: &str) -> Option<&Sensor> {
        self.sensors.iter().find(|s| s.name() == name)
    }

    /// Removes a sensor by name.
    ///
    /// # Arguments
    ///
    /// * `name` - The name of the sensor to remove
    ///
    /// # Returns
    ///
    /// `true` if a sensor was removed, `false` if no sensor with the given name was found
    pub fn remove(&mut self, name: &str) -> bool {
        let initial_len = self.sensors.len();
        self.sensors.retain(|s| s.name() != name);
        self.sensors.len() != initial_len
    }

    /// Executes all sensors in the collection and returns their responses.
    ///
    /// This method executes each sensor in the collection sequentially and
    /// collects their responses. If any sensor execution fails, the method
    /// returns an error immediately without executing the remaining sensors.
    ///
    /// # Returns
    ///
    /// A `Result` containing either a vector of `SensorResponse`s or an `Error`
    ///
    /// # Errors
    ///
    /// This method will return an error if any sensor execution fails
    pub async fn run_all(&self) -> Result<Vec<SensorResponse>> {
        let mut responses = Vec::new();

        for sensor in &self.sensors {
            let response = sensor.exec().await?;
            responses.push(response);
        }

        Ok(responses)
    }

    /// Gets the number of sensors in the collection.
    ///
    /// # Returns
    ///
    /// The number of sensors in the collection
    pub fn len(&self) -> usize {
        self.sensors.len()
    }

    /// Checks if the collection is empty.
    ///
    /// # Returns
    ///
    /// `true` if the collection contains no sensors, `false` otherwise
    pub fn is_empty(&self) -> bool {
        self.sensors.is_empty()
    }

    /// Gets an iterator over the sensors in the collection.
    ///
    /// # Returns
    ///
    /// An iterator yielding references to the sensors
    pub fn iter(&self) -> impl Iterator<Item = &Sensor> {
        self.sensors.iter()
    }
}

impl fmt::Debug for Sensors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.sensors.iter()).finish()
    }
}

impl fmt::Display for Sensors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let names: Vec<_> = self.sensors.iter().map(|s| s.name()).collect();
        write!(f, "{:?}", names)
    }
}

/// Implementation of a sensor function using a closure.
///
/// `FnSensor` provides a convenient way to create sensors using closures
/// instead of implementing the `SensorFn` trait manually.
///
/// # Examples
///
/// ```
/// use goaprs::sensor::{Sensors, FnSensor};
///
/// # #[tokio::main]
/// # async fn main() {
/// let mut sensors = Sensors::new();
///
/// // Create a sensor using a closure
/// sensors.add("time", "current_time", FnSensor::new(|| async {
///     // In a real app, this would get the actual time
///     Ok(("12:30 PM".to_string(), "".to_string(), 0))
/// })).unwrap();
/// # }
/// ```
pub struct FnSensor<F> {
    func: F,
}

impl<F, Fut> FnSensor<F>
where
    F: Fn() -> Fut + Send + Sync,
    Fut: Future<Output = Result<(String, String, i32)>> + Send,
{
    /// Creates a new function sensor from a closure.
    ///
    /// # Arguments
    ///
    /// * `func` - A closure that returns a future resolving to a tuple of (stdout, stderr, return_code)
    ///
    /// # Returns
    ///
    /// A new `FnSensor` instance
    ///
    /// # Examples
    ///
    /// ```
    /// use goaprs::sensor::FnSensor;
    ///
    /// let sensor = FnSensor::new(|| async {
    ///     Ok(("data".to_string(), "".to_string(), 0))
    /// });
    /// ```
    pub fn new(func: F) -> Self {
        Self { func }
    }
}

#[async_trait]
impl<F, Fut> SensorFn for FnSensor<F>
where
    F: Fn() -> Fut + Send + Sync,
    Fut: Future<Output = Result<(String, String, i32)>> + Send,
{
    async fn exec(&self) -> Result<SensorResponse> {
        let (stdout, stderr, return_code) = (self.func)().await?;
        Ok(SensorResponse::new(stdout, stderr, return_code))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestSensor;

    #[async_trait]
    impl SensorFn for TestSensor {
        async fn exec(&self) -> Result<SensorResponse> {
            Ok(SensorResponse::new(
                "test output".to_string(),
                "".to_string(),
                0,
            ))
        }
    }

    #[tokio::test]
    async fn test_sensor_exec() {
        let sensor = Sensor::new("test", "test_binding", TestSensor);
        let response = sensor.exec().await.unwrap();
        assert_eq!(response.stdout(), "test output");
        assert_eq!(response.return_code(), 0);
    }

    #[tokio::test]
    async fn test_sensors_collection() {
        let mut sensors = Sensors::new();

        // Add a sensor
        sensors.add("test1", "binding1", TestSensor).unwrap();

        // Try to add a sensor with the same name
        let result = sensors.add("test1", "binding2", TestSensor);
        assert!(result.is_err());

        // Add another sensor with a different name
        sensors.add("test2", "binding2", TestSensor).unwrap();

        // Check the collection
        assert_eq!(sensors.len(), 2);
        assert!(sensors.get("test1").is_some());
        assert!(sensors.get("test2").is_some());
        assert!(sensors.get("nonexistent").is_none());

        // Remove a sensor
        assert!(sensors.remove("test1"));
        assert_eq!(sensors.len(), 1);
        assert!(sensors.get("test1").is_none());

        // Try to remove a non-existent sensor
        assert!(!sensors.remove("nonexistent"));
    }

    #[tokio::test]
    async fn test_fn_sensor() {
        let sensor_fn = FnSensor::new(|| async { Ok(("output".to_string(), "".to_string(), 0)) });

        let response = sensor_fn.exec().await.unwrap();
        assert_eq!(response.stdout(), "output");
    }
}
