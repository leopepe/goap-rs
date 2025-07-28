//! ActorAutomaton is a Finite State Machine (FSM) automaton using the Actor Model to change behavior and transit between states.
//!
//! The ActorAutomaton is designed to be used in a concurrent environment, where each state is represented by an actor. The actor receives messages from other actors, which trigger state transitions and behavior changes based on the information provided.
//!
//! The automaton has 3 states: Sensing, Thinking, Planning and Acting
//! The sensing state is responsible for gathering information about the environment using the automaton's sensors and updating the actor's world state accordingly.
//! The sensors execute a predefined routine to gather information about the environment. The information returned from the sensors are called world facts. The world facts are then processed by the thinking state.
//! The sensors can be executed in parallel to improve performance. The world_state is updated with the information gathered by the sensors.
//!
//!
//! The thinking state is responsible for processing the information gathered in the sensing state and making decisions based on it.
//! The basic decision-making process involves evaluating the current world state, compare it with the automaton's goals, if the world state is not in accordance with the goals the automaton then triggers the planning state to elaborate the correct sequence of actions it must take to make the world state in accordance with the goals.
//! If the world state is in accordance with the goals the automaton then triggers the sensing state again to monitor the environment.
//! If the world state matches the automaton's goals for several consecutive times and for a long period of time (by default 1 hour), the automaton may sleep for a predefined period of time (default 5 minute and increments by 1 minute each time) to avoid overloading the system.
//! The automaton can be waken up by the user or by external events.
//!
//!
//! The planning state is responsible for generating a plan which will transform the current world state into the desired world state (the automaton's goals).
//! The action plan constitutes of a ordered sequence of actions to be executed by the automaton when it is in the acting state.
//! If the automaton cannot generate an action plan it returns an error to the user informing that no plan is achievable given the current world state and the available actions.
//!
//! The acting state is responsible for executing the plan generated in the planning state.
//! Each action is represented by a struct containing the action's name, pre-conditions, effects and a function to execute the action. (linux os shell based command for instance)
//! The action preconditions are a set of
//! When the action execution succeeds, the automaton proceeds to execute the next action in the plan. If the action execution fails, the automaton reports the failure and triggers the sensing state to gather new information about the environment.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::{mpsc, RwLock};
use tokio::time::sleep;

use futures::future;

use crate::action::{Action, ActionResponse};
use crate::error::{GoapError, Result};
use crate::planner::Planner;
use crate::sensor::{Sensor, SensorResponse};
use crate::state::State;
use crate::world_state::WorldState;

/// The different states the automaton can be in
#[derive(Debug, Clone, PartialEq)]
pub enum AutomatonState {
    /// Gathering information from sensors
    Sensing,
    /// Processing information and making decisions
    Thinking,
    /// Generating action plans
    Planning,
    /// Executing actions
    Acting,
    /// Sleeping to avoid system overload
    Sleeping,
    /// Stopped state
    Stopped,
}

/// Messages that can be sent to the automaton
#[derive(Debug)]
pub enum AutomatonMessage {
    /// Start the automaton
    Start,
    /// Stop the automaton
    Stop,
    /// Wake up from sleep
    WakeUp,
    /// Set a new goal
    SetGoal(WorldState),
    /// Get current world state
    GetWorldState(tokio::sync::oneshot::Sender<WorldState>),
    /// Get current state
    GetState(tokio::sync::oneshot::Sender<AutomatonState>),
    /// Manual sense trigger
    Sense,
    /// Manual think trigger
    Think,
    /// Manual plan trigger
    Plan,
    /// Manual act trigger
    Act,
}

/// Response from automaton operations
#[derive(Debug)]
pub struct AutomatonResponse {
    pub success: bool,
    pub message: String,
    pub data: Option<AutomatonResponseData>,
}

#[derive(Debug)]
pub enum AutomatonResponseData {
    WorldState(WorldState),
    Plan(Vec<Action>),
    ActionResults(Vec<ActionResponse>),
    SensorResults(HashMap<String, SensorResponse>),
}

/// Configuration for the automaton
#[derive(Debug, Clone)]
pub struct AutomatonConfig {
    /// How long to wait before considering the goal achieved for sleep
    pub goal_achieved_duration: Duration,
    /// Initial sleep duration
    pub initial_sleep_duration: Duration,
    /// How much to increment sleep duration each time
    pub sleep_increment: Duration,
    /// Maximum sleep duration
    pub max_sleep_duration: Duration,
    /// How often to check for state transitions
    pub tick_interval: Duration,
}

impl Default for AutomatonConfig {
    fn default() -> Self {
        Self {
            goal_achieved_duration: Duration::from_secs(3600), // 1 hour
            initial_sleep_duration: Duration::from_secs(300),  // 5 minutes
            sleep_increment: Duration::from_secs(60),          // 1 minute
            max_sleep_duration: Duration::from_secs(1800),     // 30 minutes
            tick_interval: Duration::from_millis(100),
        }
    }
}

/// The main ActorAutomaton structure
pub struct ActorAutomaton {
    /// Current state of the automaton
    current_state: AutomatonState,
    /// Current world state
    world_state: Arc<RwLock<WorldState>>,
    /// Goal state to achieve
    goal_state: Arc<RwLock<Option<WorldState>>>,
    /// Available sensors
    sensors: Vec<Sensor>,
    /// Available actions
    actions: Vec<Action>,
    /// Current action plan
    current_plan: Vec<Action>,
    /// Configuration
    config: AutomatonConfig,
    /// Time when goal was first achieved
    goal_achieved_since: Option<Instant>,
    /// Current sleep duration
    current_sleep_duration: Duration,
    /// Message receiver
    message_receiver: mpsc::UnboundedReceiver<AutomatonMessage>,
    /// Running flag
    running: bool,
}

impl ActorAutomaton {
    /// Create a new ActorAutomaton
    pub fn new(
        sensors: Vec<Sensor>,
        actions: Vec<Action>,
        initial_world_state: WorldState,
    ) -> (Self, ActorAutomatonHandle) {
        let (sender, receiver) = mpsc::unbounded_channel();
        let handle = ActorAutomatonHandle {
            sender: sender.clone(),
        };

        let automaton = Self {
            current_state: AutomatonState::Sensing,
            world_state: Arc::new(RwLock::new(initial_world_state)),
            goal_state: Arc::new(RwLock::new(None)),
            sensors,
            actions,
            current_plan: Vec::new(),
            config: AutomatonConfig::default(),
            goal_achieved_since: None,
            current_sleep_duration: AutomatonConfig::default().initial_sleep_duration,
            message_receiver: receiver,
            running: false,
        };

        (automaton, handle)
    }

    /// Create with custom configuration
    pub fn with_config(
        sensors: Vec<Sensor>,
        actions: Vec<Action>,
        initial_world_state: WorldState,
        config: AutomatonConfig,
    ) -> (Self, ActorAutomatonHandle) {
        let (mut automaton, handle) = Self::new(sensors, actions, initial_world_state);
        automaton.current_sleep_duration = config.initial_sleep_duration;
        automaton.config = config;
        (automaton, handle)
    }

    /// Start the automaton actor loop
    pub async fn run(&mut self) -> Result<()> {
        self.running = true;
        let mut tick_interval = tokio::time::interval(self.config.tick_interval);

        while self.running {
            tokio::select! {
                // Handle incoming messages
                msg = self.message_receiver.recv() => {
                    if let Some(message) = msg {
                        self.handle_message(message).await?;
                    }
                }

                // Regular tick for state transitions
                _ = tick_interval.tick() => {
                    if self.running {
                        self.tick().await?;
                    }
                }
            }
        }

        Ok(())
    }

    /// Handle incoming messages
    async fn handle_message(&mut self, message: AutomatonMessage) -> Result<()> {
        match message {
            AutomatonMessage::Start => {
                self.running = true;
                self.current_state = AutomatonState::Sensing;
            }
            AutomatonMessage::Stop => {
                self.running = false;
                self.current_state = AutomatonState::Stopped;
            }
            AutomatonMessage::WakeUp => {
                if self.current_state == AutomatonState::Sleeping {
                    self.current_state = AutomatonState::Sensing;
                    self.goal_achieved_since = None;
                }
            }
            AutomatonMessage::SetGoal(goal) => {
                let mut goal_state = self.goal_state.write().await;
                *goal_state = Some(goal);
                // Reset sleep tracking when goal changes
                self.goal_achieved_since = None;
                self.current_sleep_duration = self.config.initial_sleep_duration;
                // If we're sleeping, wake up to work on new goal
                if self.current_state == AutomatonState::Sleeping {
                    self.current_state = AutomatonState::Sensing;
                }
            }
            AutomatonMessage::GetWorldState(sender) => {
                let world_state = self.world_state.read().await;
                let _ = sender.send(world_state.clone());
            }
            AutomatonMessage::GetState(sender) => {
                let _ = sender.send(self.current_state.clone());
            }
            AutomatonMessage::Sense => {
                self.sense().await?;
            }
            AutomatonMessage::Think => {
                self.think().await?;
            }
            AutomatonMessage::Plan => {
                self.plan().await?;
            }
            AutomatonMessage::Act => {
                self.act().await?;
            }
        }
        Ok(())
    }

    /// Regular tick for autonomous state transitions
    async fn tick(&mut self) -> Result<()> {
        match self.current_state {
            AutomatonState::Sensing => {
                self.sense().await?;
                self.current_state = AutomatonState::Thinking;
            }
            AutomatonState::Thinking => {
                self.think().await?;
            }
            AutomatonState::Planning => {
                self.plan().await?;
                self.current_state = AutomatonState::Acting;
            }
            AutomatonState::Acting => {
                self.act().await?;
                self.current_state = AutomatonState::Sensing;
            }
            AutomatonState::Sleeping => {
                // Stay in sleep state until woken up
            }
            AutomatonState::Stopped => {
                // Do nothing when stopped
            }
        }
        Ok(())
    }

    /// Sensing state implementation
    async fn sense(&mut self) -> Result<()> {
        log::info!("ActorAutomaton: Entering sensing state");

        // Execute all sensors in parallel
        let sensor_futures: Vec<_> = self
            .sensors
            .iter()
            .map(|sensor| async move {
                let result = sensor.exec().await;
                (sensor.name().to_string(), result)
            })
            .collect();

        let sensor_results = future::join_all(sensor_futures).await;

        // Update world state with sensor results
        let mut world_state = self.world_state.write().await;
        for (sensor_name, result) in sensor_results {
            match result {
                Ok(sensor_response) => {
                    // Convert sensor response to world state facts
                    let fact_key = format!("{}_status", sensor_name);
                    world_state.insert(fact_key.clone(), sensor_response.stdout().to_string());
                    log::debug!(
                        "Updated world state: {} = {}",
                        fact_key,
                        sensor_response.stdout()
                    );
                }
                Err(e) => {
                    log::error!("Sensor {} failed: {}", sensor_name, e);
                }
            }
        }

        Ok(())
    }

    /// Thinking state implementation
    async fn think(&mut self) -> Result<()> {
        log::info!("ActorAutomaton: Entering thinking state");

        let goal_state = self.goal_state.read().await;
        if let Some(goal) = goal_state.as_ref() {
            let world_state = self.world_state.read().await;

            // Check if current world state satisfies the goal
            if world_state.satisfies(goal) {
                log::info!("Goal is satisfied!");

                // Track how long the goal has been achieved
                let now = Instant::now();
                if let Some(achieved_since) = self.goal_achieved_since {
                    if now.duration_since(achieved_since) >= self.config.goal_achieved_duration {
                        log::info!("Goal achieved for extended period, entering sleep state");
                        self.current_state = AutomatonState::Sleeping;

                        // Sleep for the current sleep duration
                        let sleep_duration = self.current_sleep_duration;
                        tokio::spawn(async move {
                            sleep(sleep_duration).await;
                            // Note: In a real implementation, we'd send a WakeUp message here
                        });

                        // Increment sleep duration for next time
                        self.current_sleep_duration = std::cmp::min(
                            self.current_sleep_duration + self.config.sleep_increment,
                            self.config.max_sleep_duration,
                        );

                        return Ok(());
                    }
                } else {
                    self.goal_achieved_since = Some(now);
                }

                // Goal is satisfied but not for long enough, continue sensing
                self.current_state = AutomatonState::Sensing;
            } else {
                log::info!("Goal is not satisfied, need to plan actions");
                self.goal_achieved_since = None;
                self.current_state = AutomatonState::Planning;
            }
        } else {
            log::info!("No goal set, continuing to sense");
            self.current_state = AutomatonState::Sensing;
        }

        Ok(())
    }

    /// Planning state implementation
    async fn plan(&mut self) -> Result<()> {
        log::info!("ActorAutomaton: Entering planning state");

        let goal_state = self.goal_state.read().await;
        if let Some(goal) = goal_state.as_ref() {
            let world_state = self.world_state.read().await;

            // Convert WorldState to State for planner
            let current_state = self.world_state_to_state(&world_state);
            let goal_state_converted = self.world_state_to_state(goal);

            // Create planner dynamically to avoid Send + Sync issues
            let planner = Planner::new(self.actions.clone());
            match planner.plan(&current_state, &goal_state_converted) {
                Ok(plan) => {
                    log::info!("Generated plan with {} actions", plan.len());
                    for (i, action) in plan.iter().enumerate() {
                        log::info!("  Step {}: {}", i + 1, action.name);
                    }
                    self.current_plan = plan;
                }
                Err(e) => {
                    log::error!("Failed to generate plan: {}", e);
                    self.current_plan.clear();
                    // Return to sensing if planning fails
                    self.current_state = AutomatonState::Sensing;
                    return Ok(());
                }
            }
        } else {
            log::warn!("No goal set during planning");
            self.current_state = AutomatonState::Sensing;
        }

        Ok(())
    }

    /// Acting state implementation
    async fn act(&mut self) -> Result<()> {
        log::info!("ActorAutomaton: Entering acting state");

        if self.current_plan.is_empty() {
            log::warn!("No plan to execute");
            return Ok(());
        }

        // Execute actions sequentially
        let mut action_results = Vec::new();
        for action in &self.current_plan {
            log::info!("Executing action: {}", action.name);

            // Check if action preconditions are still satisfied
            let world_state = self.world_state.read().await;
            let current_state = self.world_state_to_state(&world_state);

            if !action.can_perform(&current_state) {
                log::error!("Action {} preconditions no longer satisfied", action.name);
                // Clear plan and return to sensing
                self.current_plan.clear();
                return Ok(());
            }

            // Execute the action
            match action.exec().await {
                Ok(response) => {
                    if response.is_success() {
                        log::info!("Action {} succeeded", action.name);

                        // Apply action effects to world state
                        let mut world_state = self.world_state.write().await;
                        let effects_state = self.state_to_world_state(&action.effects);
                        *world_state = world_state.apply(&effects_state);

                        action_results.push(response);
                    } else {
                        log::error!("Action {} failed: {}", action.name, response.stderr());
                        // Clear plan and return to sensing on failure
                        self.current_plan.clear();
                        return Ok(());
                    }
                }
                Err(e) => {
                    log::error!("Action {} execution error: {}", action.name, e);
                    // Clear plan and return to sensing on error
                    self.current_plan.clear();
                    return Ok(());
                }
            }
        }

        // Clear the executed plan
        self.current_plan.clear();
        log::info!("Plan execution completed successfully");

        Ok(())
    }

    /// Convert WorldState to State for planner compatibility
    fn world_state_to_state(&self, world_state: &WorldState) -> State {
        let mut state = State::new();
        for (key, value) in world_state.inner().iter() {
            // Pass string values directly to State
            state.set(key, value);
        }
        state
    }

    /// Convert State to WorldState
    fn state_to_world_state(&self, state: &State) -> WorldState {
        let mut world_state = WorldState::new();
        for (key, value) in state.values().iter() {
            world_state.insert(key.clone(), value.clone());
        }
        world_state
    }
}

/// Handle for communicating with the ActorAutomaton
#[derive(Clone)]
pub struct ActorAutomatonHandle {
    sender: mpsc::UnboundedSender<AutomatonMessage>,
}

impl ActorAutomatonHandle {
    /// Start the automaton
    pub async fn start(&self) -> Result<()> {
        self.sender
            .send(AutomatonMessage::Start)
            .map_err(|_| GoapError::Other("Failed to send start message".into()))?;
        Ok(())
    }

    /// Stop the automaton
    pub async fn stop(&self) -> Result<()> {
        self.sender
            .send(AutomatonMessage::Stop)
            .map_err(|_| GoapError::Other("Failed to send stop message".into()))?;
        Ok(())
    }

    /// Wake up the automaton from sleep
    pub async fn wake_up(&self) -> Result<()> {
        self.sender
            .send(AutomatonMessage::WakeUp)
            .map_err(|_| GoapError::Other("Failed to send wake up message".into()))?;
        Ok(())
    }

    /// Set a new goal for the automaton
    pub async fn set_goal(&self, goal: WorldState) -> Result<()> {
        self.sender
            .send(AutomatonMessage::SetGoal(goal))
            .map_err(|_| GoapError::Other("Failed to send set goal message".into()))?;
        Ok(())
    }

    /// Get the current world state
    pub async fn get_world_state(&self) -> Result<WorldState> {
        let (sender, receiver) = tokio::sync::oneshot::channel();
        self.sender
            .send(AutomatonMessage::GetWorldState(sender))
            .map_err(|_| GoapError::Other("Failed to send get world state message".into()))?;

        receiver
            .await
            .map_err(|_| GoapError::Other("Failed to receive world state response".into()))
    }

    /// Get the current automaton state
    pub async fn get_state(&self) -> Result<AutomatonState> {
        let (sender, receiver) = tokio::sync::oneshot::channel();
        self.sender
            .send(AutomatonMessage::GetState(sender))
            .map_err(|_| GoapError::Other("Failed to send get state message".into()))?;

        receiver
            .await
            .map_err(|_| GoapError::Other("Failed to receive state response".into()))
    }

    /// Manually trigger sensing
    pub async fn sense(&self) -> Result<()> {
        self.sender
            .send(AutomatonMessage::Sense)
            .map_err(|_| GoapError::Other("Failed to send sense message".into()))?;
        Ok(())
    }

    /// Manually trigger thinking
    pub async fn think(&self) -> Result<()> {
        self.sender
            .send(AutomatonMessage::Think)
            .map_err(|_| GoapError::Other("Failed to send think message".into()))?;
        Ok(())
    }

    /// Manually trigger planning
    pub async fn plan(&self) -> Result<()> {
        self.sender
            .send(AutomatonMessage::Plan)
            .map_err(|_| GoapError::Other("Failed to send plan message".into()))?;
        Ok(())
    }

    /// Manually trigger acting
    pub async fn act(&self) -> Result<()> {
        self.sender
            .send(AutomatonMessage::Act)
            .map_err(|_| GoapError::Other("Failed to send act message".into()))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sensor::{SensorFn, SensorResponse};
    use async_trait::async_trait;

    struct MockSensorFn {
        response: String,
    }

    impl MockSensorFn {
        fn new(response: &str) -> Self {
            Self {
                response: response.to_string(),
            }
        }
    }

    #[async_trait]
    impl SensorFn for MockSensorFn {
        async fn exec(&self) -> Result<SensorResponse> {
            Ok(SensorResponse::new(self.response.clone(), String::new(), 0))
        }
    }

    #[tokio::test]
    async fn test_automaton_creation() {
        let sensors = vec![Sensor::new(
            "test_sensor",
            "test_binding",
            MockSensorFn::new("active"),
        )];
        let actions = vec![];
        let world_state = WorldState::new();

        let (automaton, _handle) = ActorAutomaton::new(sensors, actions, world_state);
        assert_eq!(automaton.current_state, AutomatonState::Sensing);
    }

    #[tokio::test]
    async fn test_handle_messages() {
        let sensors = vec![];
        let actions = vec![];
        let world_state = WorldState::new();

        let (_automaton, handle) = ActorAutomaton::new(sensors, actions, world_state);

        // Test basic message sending (these would normally be handled by the running automaton)
        assert!(handle.start().await.is_ok());
        assert!(handle.stop().await.is_ok());
        assert!(handle.wake_up().await.is_ok());
    }

    #[tokio::test]
    async fn test_world_state_conversion() {
        let sensors = vec![];
        let actions = vec![];
        let mut world_state = WorldState::new();
        world_state.insert("test_key".to_string(), "true".to_string());
        world_state.insert("test_key2".to_string(), "false".to_string());

        let (automaton, _handle) = ActorAutomaton::new(sensors, actions, world_state.clone());

        let state = automaton.world_state_to_state(&world_state);
        assert_eq!(state.get("test_key"), Some("true"));
        assert_eq!(state.get("test_key2"), Some("false"));

        let converted_back = automaton.state_to_world_state(&state);
        assert_eq!(
            converted_back.inner().get("test_key"),
            Some(&"true".to_string())
        );
        assert_eq!(
            converted_back.inner().get("test_key2"),
            Some(&"false".to_string())
        );
    }
}
