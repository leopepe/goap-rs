//! Automaton module implements the core GOAP (Goal-Oriented Action Planning) finite state machine.
//!
//! This module provides the Automaton and AutomatonController classes which handle:
//! - Sensing the environment using sensors
//! - Planning actions to achieve goals
//! - Executing actions based on the plan
//! - Maintaining world state and working memory
//!
//! The automaton follows a continuous cycle of sensing, planning, and acting to achieve its goals.

//! Automaton module implements the core GOAP (Goal-Oriented Action Planning) finite state machine.
//!
//! This module provides the Automaton and AutomatonController classes which handle:
//! - Sensing the environment using sensors
//! - Planning actions to achieve goals
//! - Executing actions based on the plan
//! - Maintaining world state and working memory
//!
//! The automaton follows a continuous cycle of sensing, planning, and acting to achieve its goals.

use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;
use std::time::{Duration, SystemTime};
use tokio::sync::Mutex;

use crate::action::Actions;
use crate::error::Result;
use crate::planner::Planner;
use crate::sensor::Sensors;
use crate::world_state::WorldState;

/// Represents a fact gathered from the environment.
///
/// Facts are produced by sensors and stored in the working memory of the automaton.
/// Each fact has a binding key, data value, timestamp, and reference to the parent sensor.
///
/// # Examples
///
/// ```
/// use goaprs::Fact;
///
/// let fact = Fact::new("temperature", "72.5", "temperature_sensor");
/// assert_eq!(fact.binding(), "temperature");
/// assert_eq!(fact.data(), "72.5");
/// assert_eq!(fact.parent_sensor(), "temperature_sensor");
/// ```
///
/// Facts are produced by sensors and stored in the working memory of the automaton.
/// Each fact has a binding key, data value, timestamp, and reference to the parent sensor.
///
/// # Examples
///
/// ```
/// use goaprs::Fact;
///
/// let fact = Fact::new("temperature", "72.5", "temperature_sensor");
/// assert_eq!(fact.binding(), "temperature");
/// assert_eq!(fact.data(), "72.5");
/// assert_eq!(fact.parent_sensor(), "temperature_sensor");
/// ```
#[derive(Debug, Clone)]
pub struct Fact {
    /// The binding key for the fact
    binding: String,
    /// The data of the fact
    data: String,
    /// When this fact was observed
    timestamp: SystemTime,
    /// The parent sensor that produced this fact
    parent_sensor: String,
}

impl Fact {
    /// Creates a new fact with the given binding, data, and parent sensor.
    ///
    /// The fact is automatically timestamped with the current system time.
    ///
    /// # Arguments
    ///
    /// * `binding` - The key that identifies what the fact represents
    /// * `data` - The actual value or information of the fact
    /// * `parent_sensor` - The name of the sensor that produced this fact
    ///
    /// # Returns
    ///
    /// A new `Fact` instance
    pub fn new(
        binding: impl Into<String>,
        data: impl Into<String>,
        parent_sensor: impl Into<String>,
    ) -> Self {
        Self {
            binding: binding.into(),
            data: data.into(),
            timestamp: SystemTime::now(),
            parent_sensor: parent_sensor.into(),
        }
    }

    /// Gets the binding key of this fact.
    ///
    /// The binding key identifies what the fact represents in the world state.
    ///
    /// # Returns
    ///
    /// A string slice containing the binding key
    pub fn binding(&self) -> &str {
        &self.binding
    }

    /// Gets the data value of this fact.
    ///
    /// # Returns
    ///
    /// A string slice containing the fact's data
    pub fn data(&self) -> &str {
        &self.data
    }

    /// Gets the timestamp when this fact was created.
    ///
    /// # Returns
    ///
    /// The `SystemTime` when this fact was created
    pub fn timestamp(&self) -> SystemTime {
        self.timestamp
    }

    /// Gets the name of the parent sensor that produced this fact.
    ///
    /// # Returns
    ///
    /// A string slice containing the parent sensor's name
    pub fn parent_sensor(&self) -> &str {
        &self.parent_sensor
    }
}

impl fmt::Display for Fact {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.binding, self.data)
    }
}

/// States of the automaton's finite state machine.
///
/// The automaton cycles through these states as it operates:
/// 1. `WaitingOrders`: Idle state, waiting for a goal to be set
/// 2. `Sensing`: Gathering information from the environment
/// 3. `Planning`: Generating an action plan to achieve the goal
/// 4. `Acting`: Executing the planned actions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Waiting for orders - idle state before receiving goals
    WaitingOrders,
    /// Sensing the environment - gathering data from sensors
    Sensing,
    /// Planning actions - determining sequence of actions to reach goal
    Planning,
    /// Acting on the plan - executing planned actions
    Acting,
}

/// A finite state machine automaton that manages the GOAP (Goal-Oriented Action Planning) process.
///
/// The automaton is the core component of the GOAP system. It:
/// - Maintains the current world state and goal state
/// - Collects facts from sensors into working memory
/// - Plans actions to transition from the current state to the goal state
/// - Executes actions according to the plan
///
/// The automaton follows a cycle of sensing, planning, and acting to achieve its goals.
///
/// # Thread Safety
///
/// The automaton is designed to be thread-safe with internal state protected by `Arc<Mutex<>>`.
pub struct Automaton {
    /// Gets the name of the automaton.
    ///
    /// # Returns
    ///
    /// A string slice containing the automaton's name
    name: String,
    /// The current state of the automaton's finite state machine
    state: Arc<Mutex<State>>,
    /// The current world state (key-value pairs representing the environment)
    world_state: Arc<Mutex<WorldState>>,
    /// The goal state the automaton is trying to achieve
    goal: Arc<Mutex<WorldState>>,
    /// The working memory containing facts gathered from sensors
    working_memory: Arc<Mutex<Vec<Fact>>>,
    /// Collection of sensors used to gather information from the environment
    sensors: Sensors,
    /// Collection of available actions the automaton can perform
    #[allow(dead_code)]
    actions: Actions,
    /// The planner used to generate action sequences
    planner: Planner,
    /// The current action plan being executed
    action_plan: Arc<Mutex<Vec<crate::action::Action>>>,
}

impl Automaton {
    /// Creates a new automaton with the given name, sensors, actions, and initial world state.
    ///
    /// # Arguments
    ///
    /// * `name` - A name for the automaton
    /// * `sensors` - The collection of sensors the automaton will use to gather information
    /// * `actions` - The collection of actions the automaton can perform
    /// * `world_state_facts` - Initial key-value pairs representing the starting world state
    ///
    /// # Returns
    ///
    /// A new `Automaton` instance initialized with the provided components and in the `WaitingOrders` state
    ///
    /// # Examples
    ///
    /// ```
    /// use std::collections::HashMap;
    /// use goaprs::{Automaton, Sensors, Actions};
    ///
    /// let sensors = Sensors::new();
    /// let actions = Actions::new();
    /// let mut initial_state = HashMap::new();
    /// initial_state.insert("location".to_string(), "home".to_string());
    ///
    /// let automaton = Automaton::new("home_assistant", sensors, actions, initial_state);
    /// ```
    pub fn new(
        name: impl Into<String>,
        sensors: Sensors,
        actions: Actions,
        world_state_facts: HashMap<String, String>,
    ) -> Self {
        Self {
            name: name.into(),
            state: Arc::new(Mutex::new(State::WaitingOrders)),
            world_state: Arc::new(Mutex::new(WorldState::from_hashmap(world_state_facts))),
            goal: Arc::new(Mutex::new(WorldState::new())),
            working_memory: Arc::new(Mutex::new(Vec::new())),
            sensors,
            actions: actions.clone(),
            planner: Planner::new(actions),
            action_plan: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Gets the name of the automaton.
    ///
    /// # Returns
    ///
    /// A string slice containing the automaton's name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Gets the current state of the automaton's finite state machine.
    ///
    /// # Returns
    ///
    /// A `Result` containing the current `State` of the automaton
    ///
    /// # Errors
    ///
    /// Returns an error if the state mutex is poisoned
    pub async fn state(&self) -> Result<State> {
        let state_lock = self.state.lock().await;
        Ok(*state_lock)
    }

    /// Sets the goal state for the automaton.
    ///
    /// When a new goal is set, the automaton transitions to the `WaitingOrders` state,
    /// preparing for a new sense-plan-act cycle.
    ///
    /// # Arguments
    ///
    /// * `goal` - A HashMap of key-value pairs representing the desired goal state
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if either the goal or state mutex is poisoned
    pub async fn set_goal(&self, goal: HashMap<String, String>) -> Result<()> {
        // Update goal
        {
            let mut goal_lock = self.goal.lock().await;
            *goal_lock = WorldState::from_hashmap(goal);
        }

        // When setting a goal, transition to waiting orders
        {
            let mut state_lock = self.state.lock().await;
            *state_lock = State::WaitingOrders;
        }

        Ok(())
    }

    /// Gets the current goal state from the managed automaton.
    ///
    /// # Returns
    ///
    /// A `Result` containing the current goal `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if retrieving the goal state fails
    ///
    /// # Returns
    ///
    /// A `Result` containing a clone of the current `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if the world state mutex is poisoned
    ///
    /// # Returns
    ///
    /// A `Result` containing a clone of the current goal `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if the goal mutex is poisoned
    ///
    /// # Returns
    ///
    /// A `Result` containing a clone of the current goal `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if the goal mutex is poisoned
    pub async fn goal(&self) -> Result<WorldState> {
        let goal_lock = self.goal.lock().await;
        Ok(goal_lock.clone())
    }

    /// Gets the current world state from the managed automaton.
    ///
    /// # Returns
    ///
    /// A `Result` containing the current `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if retrieving the world state fails
    ///
    /// # Returns
    ///
    /// A `Result` containing a clone of the current `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if the world state mutex is poisoned
    pub async fn world_state(&self) -> Result<WorldState> {
        let world_state_lock = self.world_state.lock().await;
        Ok(world_state_lock.clone())
    }

    /// Transitions to the sensing state and gathers data from all sensors.
    ///
    /// This method:
    /// 1. Changes the automaton state to `Sensing`
    /// 2. Clears the current working memory
    /// 3. Executes each sensor and stores results as facts in working memory
    /// 4. Updates the world state with the gathered facts
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Any mutex is poisoned
    /// - Any sensor execution fails
    ///
    /// # Examples
    ///
    /// ```
    /// # async fn example(automaton: &goaprs::Automaton) -> Result<(), Box<dyn std::error::Error>> {
    /// // Gather information from all sensors
    /// automaton.sense().await?;
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// This method:
    /// 1. Changes the automaton state to `Sensing`
    /// 2. Clears the current working memory
    /// 3. Executes each sensor and stores results as facts in working memory
    /// 4. Updates the world state with the gathered facts
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Any mutex is poisoned
    /// - Any sensor execution fails
    ///
    /// # Examples
    ///
    /// ```
    /// # async fn example(automaton: &goaprs::Automaton) -> Result<(), Box<dyn std::error::Error>> {
    /// // Gather information from all sensors
    /// automaton.sense().await?;
    /// # Ok(())
    /// # }
    /// ```
    pub async fn sense(&self) -> Result<()> {
        // Change state to sensing
        {
            let mut state_lock = self.state.lock().await;
            *state_lock = State::Sensing;
        }

        // Clear the working memory
        {
            let mut working_memory = self.working_memory.lock().await;
            working_memory.clear();
        }

        // Run each sensor and update the working memory
        for sensor in self.sensors.iter() {
            let response = sensor.exec().await?;

            // Add the fact to working memory
            {
                let mut working_memory = self.working_memory.lock().await;
                working_memory.push(Fact::new(
                    sensor.binding(),
                    response.response(),
                    sensor.name(),
                ));
            }
        }

        // Update the world state with the facts
        {
            // Get facts from working memory first
            let facts: Vec<Fact> = {
                let working_memory = self.working_memory.lock().await;
                working_memory.clone()
            };

            // Then update world state with collected facts
            let mut world_state = self.world_state.lock().await;
            for fact in facts.iter() {
                world_state.insert(fact.binding().to_string(), fact.data().to_string());
            }
        }

        Ok(())
    }

    /// Transitions to the planning state and generates an action plan to reach the goal.
    ///
    /// This method:
    /// 1. Changes the automaton state to `Planning`
    /// 2. Uses the planner to find a sequence of actions that will transition from
    ///    the current world state to the goal state
    /// 3. Stores the generated plan for later execution
    ///
    /// # Returns
    ///
    /// A `Result` containing the generated action plan as a vector of `Action`s
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Any mutex is poisoned
    /// - The planner fails to generate a valid plan
    ///
    /// # Examples
    ///
    /// ```
    /// # async fn example(automaton: &goaprs::Automaton) -> Result<(), Box<dyn std::error::Error>> {
    /// // Generate a plan to achieve the goal
    /// let action_plan = automaton.plan().await?;
    /// println!("Generated plan with {} actions", action_plan.len());
    /// # Ok(())
    /// # }
    /// ```
    pub async fn plan(&self) -> Result<Vec<crate::action::Action>> {
        // Set state to Planning
        {
            let mut state_lock = self.state.lock().await;
            *state_lock = State::Planning;
        }

        // Get world state and goal state
        let world_hash: HashMap<String, String>;
        let goal_hash: HashMap<String, String>;

        {
            let world_state = self.world_state.lock().await;
            world_hash = world_state
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect();
        }

        {
            let goal = self.goal.lock().await;
            goal_hash = goal.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        }

        // Generate plan
        let mut planner = self.planner.clone();
        let plan = planner.plan(&world_hash, &goal_hash)?;

        // Store the action plan
        {
            let mut action_plan = self.action_plan.lock().await;
            *action_plan = plan.clone();
        }

        Ok(plan)
    }

    /// Transitions to the acting state and executes the current action plan.
    ///
    /// This method:
    /// 1. Changes the automaton state to `Acting`
    /// 2. Executes each action in the previously generated plan in sequence
    /// 3. Collects and returns the responses from each action
    ///
    /// # Returns
    ///
    /// A `Result` containing a vector of `ActionResponse`s from executed actions
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Any mutex is poisoned
    /// - Any action execution fails
    ///
    /// # Examples
    ///
    /// ```
    /// # async fn example(automaton: &goaprs::Automaton) -> Result<(), Box<dyn std::error::Error>> {
    /// // Execute the current action plan
    /// let responses = automaton.act().await?;
    /// for (i, response) in responses.iter().enumerate() {
    ///     println!("Action {}: {}", i, response);
    /// }
    /// # Ok(())
    /// # }
    /// ```
    ///
    /// This method:
    /// 1. Changes the automaton state to `Acting`
    /// 2. Executes each action in the previously generated plan in sequence
    /// 3. Collects and returns the responses from each action
    ///
    /// # Returns
    ///
    /// A `Result` containing a vector of `ActionResponse`s from executed actions
    ///
    /// # Errors
    ///
    /// Returns an error if:
    /// - Any mutex is poisoned
    /// - Any action execution fails
    ///
    /// # Examples
    ///
    /// ```
    /// # async fn example(automaton: &goaprs::Automaton) -> Result<(), Box<dyn std::error::Error>> {
    /// // Execute the current action plan
    /// let responses = automaton.act().await?;
    /// for (i, response) in responses.iter().enumerate() {
    ///     println!("Action {}: {}", i, response);
    /// }
    /// # Ok(())
    /// # }
    /// ```
    pub async fn act(&self) -> Result<Vec<crate::action::ActionResponse>> {
        // Set state to Acting
        {
            let mut state_lock = self.state.lock().await;
            *state_lock = State::Acting;
        }

        // Get action plan
        let actions_to_execute = {
            let action_plan = self.action_plan.lock().await;
            action_plan.clone()
        };

        let mut responses = Vec::new();

        // Execute each action in the plan
        for action in actions_to_execute.iter() {
            let response = action.exec().await?;
            responses.push(response.clone());
        }

        Ok(responses)
    }

    /// Transitions to the waiting orders state.
    ///
    /// This method:
    /// 1. Changes the automaton state to `WaitingOrders`
    /// 2. Clears the working memory
    ///
    /// This state indicates the automaton has either completed its goal or
    /// is waiting for a new goal to be set.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if any mutex is poisoned
    ///
    /// This method:
    /// 1. Changes the automaton state to `WaitingOrders`
    /// 2. Clears the working memory
    ///
    /// This state indicates the automaton has either completed its goal or
    /// is waiting for a new goal to be set.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if any mutex is poisoned
    pub async fn wait(&self) -> Result<()> {
        // Set state to WaitingOrders
        {
            let mut state_lock = self.state.lock().await;
            *state_lock = State::WaitingOrders;
        }

        // Clear the working memory
        {
            let mut working_memory = self.working_memory.lock().await;
            working_memory.clear();
        }

        Ok(())
    }
}

/// A controller that runs the automaton in a continuous loop.
///
/// The AutomatonController manages an Automaton instance, running its sense-plan-act
/// cycle in a separate thread. It provides methods to start and stop the automaton's
/// execution, as well as to set goals and access state information.
///
/// # Examples
///
/// ```
/// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
/// use std::collections::HashMap;
/// use goaprs::{AutomatonController, Sensors, Actions};
///
/// let sensors = Sensors::new();
/// let actions = Actions::new();
/// let mut world_state = HashMap::new();
/// world_state.insert("location".to_string(), "home".to_string());
///
/// let controller = AutomatonController::new(actions, sensors, "home_assistant", world_state);
///
/// // Set a goal
/// let mut goal = HashMap::new();
/// goal.insert("lights".to_string(), "on".to_string());
/// controller.set_goal(goal).await?;
///
/// // Start the automaton
/// controller.start().await?;
///
/// // Later, stop the automaton
/// controller.stop().await?;
/// # Ok(())
/// # }
/// ```
///
/// The AutomatonController manages an Automaton instance, running its sense-plan-act
/// cycle in a separate thread. It provides methods to start and stop the automaton's
/// execution, as well as to set goals and access state information.
///
/// # Examples
///
/// ```
/// # async fn example() -> Result<(), Box<dyn std::error::Error>> {
/// use std::collections::HashMap;
/// use goaprs::{AutomatonController, Sensors, Actions};
///
/// let sensors = Sensors::new();
/// let actions = Actions::new();
/// let mut world_state = HashMap::new();
/// world_state.insert("location".to_string(), "home".to_string());
///
/// let controller = AutomatonController::new(actions, sensors, "home_assistant", world_state);
///
/// // Set a goal
/// let mut goal = HashMap::new();
/// goal.insert("lights".to_string(), "on".to_string());
/// controller.set_goal(goal).await?;
///
/// // Start the automaton
/// controller.start().await?;
///
/// // Later, stop the automaton
/// controller.stop().await?;
/// # Ok(())
/// # }
/// ```
pub struct AutomatonController {
    /// The automaton to control
    automaton: Arc<Automaton>,
    /// Whether the controller is running
    running: Arc<Mutex<bool>>,
}

impl AutomatonController {
    /// Creates a new automaton controller with the given components.
    ///
    /// # Arguments
    ///
    /// * `actions` - Collection of actions the automaton can perform
    /// * `sensors` - Collection of sensors the automaton will use
    /// * `name` - Name for the automaton
    /// * `world_state` - Initial world state as key-value pairs
    ///
    /// # Returns
    ///
    /// A new `AutomatonController` instance that manages an Automaton with the provided components
    pub fn new(
        actions: Actions,
        sensors: Sensors,
        name: impl Into<String>,
        world_state: HashMap<String, String>,
    ) -> Self {
        Self {
            automaton: Arc::new(Automaton::new(name, sensors, actions, world_state)),
            running: Arc::new(Mutex::new(false)),
        }
    }

    /// Gets a reference to the managed automaton.
    ///
    /// # Returns
    ///
    /// A reference to the `Automaton` instance
    pub fn automaton(&self) -> &Automaton {
        &self.automaton
    }

    /// Sets the goal state for the automaton.
    ///
    /// # Arguments
    ///
    /// * `goal` - A HashMap of key-value pairs representing the desired goal state
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if setting the goal fails
    pub async fn set_goal(&self, goal: HashMap<String, String>) -> Result<()> {
        self.automaton.set_goal(goal).await
    }

    /// Gets the current world state from the managed automaton.
    ///
    /// # Returns
    ///
    /// A `Result` containing the current `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if retrieving the world state fails
    pub async fn world_state(&self) -> Result<WorldState> {
        self.automaton.world_state().await
    }

    /// Gets the current goal state from the managed automaton.
    ///
    /// # Returns
    ///
    /// A `Result` containing the current goal `WorldState`
    ///
    /// # Errors
    ///
    /// Returns an error if retrieving the goal state fails
    pub async fn goal(&self) -> Result<WorldState> {
        self.automaton.goal().await
    }

    /// Starts the automaton controller in a new thread.
    ///
    /// This method:
    /// 1. Sets the running flag to true
    /// 2. Spawns a new thread that runs the automaton's sense-plan-act cycle
    /// 3. The thread will continue until the controller is stopped
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if the running mutex is poisoned
    ///
    /// This method:
    /// 1. Sets the running flag to true
    /// 2. Spawns a new thread that runs the automaton's sense-plan-act cycle
    /// 3. The thread will continue until the controller is stopped
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if the running mutex is poisoned
    pub async fn start(&self) -> Result<()> {
        {
            let mut running = self.running.lock().await;
            *running = true;
        }

        let automaton = self.automaton.clone();
        let running = self.running.clone();

        // Spawn a task that runs the automaton loop
        tokio::spawn(async move {
            loop {
                let is_running = {
                    let running_guard = running.lock().await;
                    *running_guard
                };

                if !is_running {
                    break;
                }

                // Check if we've reached the goal
                let world_state = automaton.world_state().await.unwrap();
                let goal = automaton.goal().await.unwrap();

                if world_state.satisfies(&goal) {
                    println!("World state satisfies goal: {:?}", goal);
                    automaton.wait().await.unwrap();
                } else {
                    println!(
                        "World state differs from goal: \nState: {:?}\nGoal: {:?}",
                        world_state, goal
                    );
                    println!("Need to find an action plan");

                    // Generate a plan
                    let plan = automaton.plan().await.unwrap();
                    println!("Plan found. Will execute the action plan: {:?}", plan);

                    // Execute the plan
                    automaton.act().await.unwrap();
                }

                // Sense the environment
                automaton.sense().await.unwrap();

                // Wait before next cycle
                tokio::time::sleep(Duration::from_secs(5)).await;
            }
        });

        Ok(())
    }

    /// Stops the automaton controller.
    ///
    /// This method sets the running flag to false, which will cause the
    /// controller's thread to exit after its current cycle completes.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if the running mutex is poisoned
    ///
    /// This method sets the running flag to false, which will cause the
    /// controller's thread to exit after its current cycle completes.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure
    ///
    /// # Errors
    ///
    /// Returns an error if the running mutex is poisoned
    pub async fn stop(&self) -> Result<()> {
        let mut running = self.running.lock().await;
        *running = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::{ActionFn, ActionResponse};
    use crate::sensor::{SensorFn, SensorResponse};
    use async_trait::async_trait;

    struct TestSensor;

    #[async_trait]
    impl SensorFn for TestSensor {
        async fn exec(&self) -> Result<SensorResponse> {
            Ok(SensorResponse::new(
                "test value".to_string(),
                "".to_string(),
                0,
            ))
        }
    }

    struct TestAction;

    #[async_trait]
    impl ActionFn for TestAction {
        async fn exec(&self) -> Result<ActionResponse> {
            Ok(ActionResponse::new(
                "action executed".to_string(),
                "".to_string(),
                0,
            ))
        }
    }

    #[tokio::test]
    async fn test_automaton_basic_cycle() {
        // Create sensors and actions
        let mut sensors = Sensors::new();
        sensors.add("test_sensor", "test_key", TestSensor).unwrap();

        let mut actions = Actions::new();

        let mut conditions = HashMap::new();
        conditions.insert("test_key".to_string(), "test value".to_string());

        let mut effects = HashMap::new();
        effects.insert("goal_key".to_string(), "goal value".to_string());

        actions
            .add("test_action", conditions, effects, TestAction, 1.0)
            .unwrap();

        // Creates a new automaton with the given name, sensors, actions, and initial world state.
        //
        // # Arguments
        //
        // * `name` - A name for the automaton
        // * `sensors` - The collection of sensors the automaton will use to gather information
        // * `actions` - The collection of actions the automaton can perform
        // * `world_state_facts` - Initial key-value pairs representing the starting world state
        //
        // # Returns
        //
        // A new `Automaton` instance initialized with the provided components and in the `WaitingOrders` state
        //
        // # Examples
        //
        // ```
        // use std::collections::HashMap;
        // use goaprs::{Automaton, Sensors, Actions};
        //
        // let sensors = Sensors::new();
        // let actions = Actions::new();
        // let mut initial_state = HashMap::new();
        // initial_state.insert("location".to_string(), "home".to_string());
        //
        // let automaton = Automaton::new("home_assistant", sensors, actions, initial_state);
        // ```
        let mut world_state = HashMap::new();
        world_state.insert("initial_key".to_string(), "initial value".to_string());

        let automaton = Automaton::new("test", sensors, actions, world_state);

        // Set a goal
        let mut goal = HashMap::new();
        goal.insert("goal_key".to_string(), "goal value".to_string());
        automaton.set_goal(goal).await.unwrap();

        // Run through the cycle
        assert_eq!(automaton.state().await.unwrap(), State::WaitingOrders);

        // Sense
        automaton.sense().await.unwrap();
        assert_eq!(automaton.state().await.unwrap(), State::Sensing);

        // Check that the world state was updated
        let ws = automaton.world_state().await.unwrap();
        assert_eq!(ws.get("test_key"), Some(&"test value".to_string()));

        // Plan
        let plan = automaton.plan().await.unwrap();
        assert_eq!(automaton.state().await.unwrap(), State::Planning);
        assert_eq!(plan.len(), 1);
        assert_eq!(plan[0].name(), "test_action");

        // Act
        let responses = automaton.act().await.unwrap();
        assert_eq!(automaton.state().await.unwrap(), State::Acting);
        assert_eq!(responses.len(), 1);
        assert_eq!(responses[0].stdout(), "action executed");
    }
}
