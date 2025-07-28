//! ActorAutomaton Example
//!
//! This example demonstrates how to use the ActorAutomaton finite state machine
//! with the actor model. The automaton cycles through states: Sensing -> Thinking -> Planning -> Acting
//!
//! The example simulates a smart home agent that can control a lamp based on goals.

use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::sleep;

use goaprs::action::Action;
use goaprs::error::Result;
use goaprs::sensor::{Sensor, SensorFn, SensorResponse};
use goaprs::utils::actorautomaton::{ActorAutomaton, AutomatonConfig, AutomatonState};
use goaprs::world_state::WorldState;

use async_trait::async_trait;

/// Simulated lamp environment
struct LampEnvironment {
    is_on: Mutex<bool>,
}

impl LampEnvironment {
    fn new(initial_state: bool) -> Self {
        Self {
            is_on: Mutex::new(initial_state),
        }
    }

    async fn is_on(&self) -> bool {
        *self.is_on.lock().await
    }
}

/// Lamp status sensor implementation
struct LampSensor {
    env: Arc<LampEnvironment>,
}

impl LampSensor {
    fn new(env: Arc<LampEnvironment>) -> Self {
        Self { env }
    }
}

#[async_trait]
impl SensorFn for LampSensor {
    async fn exec(&self) -> Result<SensorResponse> {
        let is_on = self.env.is_on().await;
        let status = if is_on { "on" } else { "off" };
        println!("🔍 Sensor reading: lamp is {}", status);
        Ok(SensorResponse::new(status.to_string(), String::new(), 0))
    }
}

/// System status sensor implementation
struct SystemSensor;

#[async_trait]
impl SensorFn for SystemSensor {
    async fn exec(&self) -> Result<SensorResponse> {
        println!("🔍 Sensor reading: system is operational");
        Ok(SensorResponse::new(
            "operational".to_string(),
            String::new(),
            0,
        ))
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    env_logger::init();

    println!("🚀 Starting ActorAutomaton Smart Home Example");
    println!("==============================================\n");

    // Create shared lamp environment
    let lamp_env = Arc::new(LampEnvironment::new(false));

    // Create sensors
    let lamp_sensor = Sensor::new(
        "lamp_check",
        "lamp_status",
        LampSensor::new(lamp_env.clone()),
    );
    let system_sensor = Sensor::new("system_check", "system_status", SystemSensor);

    let sensors = vec![lamp_sensor, system_sensor];

    // Create actions
    let mut actions = Vec::new();

    // Turn lamp ON action
    let mut turn_on_action = Action::new("turn_lamp_on", 1.0)?;
    turn_on_action.preconditions.set("lamp_status", "false"); // lamp must be off
    turn_on_action.effects.set("lamp_status", "true"); // lamp will be on
    actions.push(turn_on_action);

    // Turn lamp OFF action
    let mut turn_off_action = Action::new("turn_lamp_off", 1.0)?;
    turn_off_action.preconditions.set("lamp_status", "true"); // lamp must be on
    turn_off_action.effects.set("lamp_status", "false"); // lamp will be off
    actions.push(turn_off_action);

    // Setup action (always available)
    let mut setup_action = Action::new("setup_system", 0.5)?;
    setup_action.effects.set("system_status", "true");
    actions.push(setup_action);

    // Create initial world state
    let mut initial_world_state = WorldState::new();
    initial_world_state.insert("lamp_status".to_string(), "off".to_string());
    initial_world_state.insert("system_status".to_string(), "unknown".to_string());

    // Configure the automaton with faster cycles for demonstration
    let config = AutomatonConfig {
        goal_achieved_duration: Duration::from_secs(5), // 5 seconds instead of 1 hour
        initial_sleep_duration: Duration::from_secs(2), // 2 seconds
        sleep_increment: Duration::from_secs(1),
        max_sleep_duration: Duration::from_secs(10),
        tick_interval: Duration::from_millis(500), // Faster ticks for demo
    };

    // Create the ActorAutomaton
    let (mut automaton, handle) =
        ActorAutomaton::with_config(sensors, actions, initial_world_state, config);

    // Start the automaton in a background task
    let automaton_task = tokio::spawn(async move {
        if let Err(e) = automaton.run().await {
            eprintln!("Automaton error: {}", e);
        }
    });

    // Start the automaton
    println!("🏁 Starting automaton...");
    handle.start().await?;

    // Wait a bit to let it start
    sleep(Duration::from_millis(500)).await;

    // Demonstrate manual state queries
    let current_state = handle.get_state().await?;
    println!("📊 Current automaton state: {:?}\n", current_state);

    // Set a goal: turn the lamp ON
    println!("🎯 Setting goal: Turn lamp ON");
    let mut goal = WorldState::new();
    goal.insert("lamp_status".to_string(), "on".to_string());
    goal.insert("system_status".to_string(), "operational".to_string());
    handle.set_goal(goal).await?;

    // Wait for the automaton to work on the goal
    println!("⏳ Waiting for automaton to achieve goal...\n");
    sleep(Duration::from_secs(3)).await;

    // Check the world state
    let world_state = handle.get_world_state().await?;
    println!("🌍 Current world state: {:?}", world_state.inner());

    // Change the goal: turn the lamp OFF
    println!("\n🎯 Changing goal: Turn lamp OFF");
    let mut new_goal = WorldState::new();
    new_goal.insert("lamp_status".to_string(), "off".to_string());
    new_goal.insert("system_status".to_string(), "operational".to_string());
    handle.set_goal(new_goal).await?;

    // Wait for the automaton to work on the new goal
    println!("⏳ Waiting for automaton to achieve new goal...\n");
    sleep(Duration::from_secs(3)).await;

    // Check the final world state
    let final_world_state = handle.get_world_state().await?;
    println!("🌍 Final world state: {:?}", final_world_state.inner());

    // Demonstrate manual operations
    println!("\n🔧 Demonstrating manual operations:");

    println!("  - Manual sense...");
    handle.sense().await?;
    sleep(Duration::from_millis(500)).await;

    println!("  - Manual think...");
    handle.think().await?;
    sleep(Duration::from_millis(500)).await;

    let current_state = handle.get_state().await?;
    println!(
        "  - Current state after manual operations: {:?}",
        current_state
    );

    // Test wake up functionality (set a goal that keeps being satisfied)
    println!("\n💤 Testing sleep/wake functionality...");
    println!("   Setting a goal that's already satisfied to trigger sleep mode");

    let satisfied_goal = final_world_state.clone();
    handle.set_goal(satisfied_goal).await?;

    println!("   Waiting for automaton to enter sleep mode...");
    sleep(Duration::from_secs(6)).await;

    let sleep_state = handle.get_state().await?;
    if sleep_state == AutomatonState::Sleeping {
        println!("   ✅ Automaton entered sleep mode!");

        println!("   Waking up automaton...");
        handle.wake_up().await?;
        sleep(Duration::from_millis(500)).await;

        let wake_state = handle.get_state().await?;
        println!("   State after wake up: {:?}", wake_state);
    } else {
        println!(
            "   ⏰ Automaton didn't sleep yet (state: {:?})",
            sleep_state
        );
    }

    // Stop the automaton
    println!("\n🛑 Stopping automaton...");
    handle.stop().await?;

    // Wait for the task to finish
    automaton_task.await.unwrap();

    println!("✅ ActorAutomaton example completed successfully!");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_lamp_sensor() {
        let env = Arc::new(LampEnvironment::new(false));
        let sensor = LampSensor::new(env.clone());

        // Test sensor when lamp is off
        let response = sensor.exec().await.unwrap();
        assert_eq!(response.stdout(), "off");

        // Turn lamp on and test again
        env.turn_on().await;
        let response = sensor.exec().await.unwrap();
        assert_eq!(response.stdout(), "on");
    }

    #[tokio::test]
    async fn test_system_sensor() {
        let sensor = SystemSensor;
        let response = sensor.exec().await.unwrap();
        assert_eq!(response.stdout(), "operational");
    }

    #[tokio::test]
    async fn test_lamp_environment() {
        let env = LampEnvironment::new(false);

        assert!(!env.is_on().await);

        env.turn_on().await;
        assert!(env.is_on().await);

        env.turn_off().await;
        assert!(!env.is_on().await);
    }

    #[tokio::test]
    async fn test_automaton_creation() {
        let env = Arc::new(LampEnvironment::new(false));
        let sensors = vec![
            Sensor::new("lamp_check", "lamp_status", LampSensor::new(env)),
            Sensor::new("system_check", "system_status", SystemSensor),
        ];

        let actions = vec![];
        let world_state = WorldState::new();

        let (automaton, _handle) = ActorAutomaton::new(sensors, actions, world_state);

        // The automaton should be created successfully
        // This is mainly a compilation test
    }
}
