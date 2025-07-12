use goaprs::utils::actor::{ActionFn, Fact, PlannerFn, SensorFn};
use goaprs::utils::ActorAutomatonController;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::sleep;

/// A shared state to demonstrate communication between components
#[derive(Debug)]
struct WorldState {
    temperature: i32,
    heater_on: bool,
    target_temperature: i32,
}

impl WorldState {
    fn new() -> Self {
        Self {
            temperature: 18, // Starting room temperature
            heater_on: false,
            target_temperature: 22, // Target temperature
        }
    }
}

// Sensors
#[derive(Debug)]
struct TemperatureSensor {
    world_state: Arc<Mutex<WorldState>>,
}

impl TemperatureSensor {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self { world_state }
    }
}

#[async_trait::async_trait]
impl SensorFn for TemperatureSensor {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> Vec<Fact> {
        let state = self.world_state.lock().unwrap();

        vec![
            Fact::new(
                "temperature",
                &state.temperature.to_string(),
                "TemperatureSensor",
            ),
            Fact::new(
                "target_temperature",
                &state.target_temperature.to_string(),
                "TemperatureSensor",
            ),
        ]
    }
}

#[derive(Debug)]
struct HeaterStateSensor {
    world_state: Arc<Mutex<WorldState>>,
}

impl HeaterStateSensor {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self { world_state }
    }
}

#[async_trait::async_trait]
impl SensorFn for HeaterStateSensor {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> Vec<Fact> {
        let state = self.world_state.lock().unwrap();

        vec![Fact::new(
            "heater_on",
            &state.heater_on.to_string(),
            "HeaterStateSensor",
        )]
    }
}

// Actions
struct TurnHeaterOnAction {
    world_state: Arc<Mutex<WorldState>>,
    id: &'static str,
}

impl TurnHeaterOnAction {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self {
            world_state,
            id: "TurnHeaterOnAction",
        }
    }

    fn id(&self) -> &str {
        self.id
    }
}

#[async_trait::async_trait]
impl ActionFn for TurnHeaterOnAction {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> bool {
        println!("Executing: Turn heater ON");
        let mut state = self.world_state.lock().unwrap();
        state.heater_on = true;
        // Simulate the heater warming the room
        state.temperature += 1;
        true
    }
}

struct TurnHeaterOffAction {
    world_state: Arc<Mutex<WorldState>>,
    id: &'static str,
}

impl TurnHeaterOffAction {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self {
            world_state,
            id: "TurnHeaterOffAction",
        }
    }

    fn id(&self) -> &str {
        self.id
    }
}

#[async_trait::async_trait]
impl ActionFn for TurnHeaterOffAction {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> bool {
        println!("Executing: Turn heater OFF");
        let mut state = self.world_state.lock().unwrap();
        state.heater_on = false;
        true
    }
}

struct IncreaseTemperatureAction {
    world_state: Arc<Mutex<WorldState>>,
    id: &'static str,
}

impl IncreaseTemperatureAction {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self {
            world_state,
            id: "IncreaseTemperatureAction",
        }
    }

    fn id(&self) -> &str {
        self.id
    }
}

#[async_trait::async_trait]
impl ActionFn for IncreaseTemperatureAction {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> bool {
        println!("Executing: Increase temperature");
        let mut state = self.world_state.lock().unwrap();

        if state.heater_on {
            state.temperature += 2;
            println!("  Temperature increased to {}", state.temperature);
            true
        } else {
            println!("  Cannot increase temperature - heater is off!");
            false
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
struct TemperatureControlPlanner;

#[async_trait::async_trait]
impl PlannerFn for TemperatureControlPlanner {
    async fn plan(
        &self,
        world_state: &HashMap<String, Fact>,
        goal: &HashMap<String, Fact>,
        available_actions: &[Arc<dyn ActionFn>],
    ) -> Vec<Arc<dyn ActionFn>> {
        println!("Planning with world state: {:?}", world_state.keys());

        // Create wrapped actions for identification
        let action_wrappers = vec![
            ActionWrapper::new("TurnHeaterOnAction", available_actions[0].clone()),
            ActionWrapper::new("TurnHeaterOffAction", available_actions[1].clone()),
            ActionWrapper::new("IncreaseTemperatureAction", available_actions[2].clone()),
        ];

        // A simple planning strategy
        let mut plan = Vec::new();

        if goal.contains_key("target_temperature_reached") {
            if let (Some(temp_fact), Some(target_fact)) = (
                world_state.get("temperature"),
                world_state.get("target_temperature"),
            ) {
                let current_temp: i32 = temp_fact.data().parse().unwrap_or(0);
                let target_temp: i32 = target_fact.data().parse().unwrap_or(0);

                println!(
                    "Current temp: {}, Target temp: {}",
                    current_temp, target_temp
                );

                let heater_on = world_state
                    .get("heater_on")
                    .map(|f| f.data() == "true")
                    .unwrap_or(false);

                // Simple planning logic
                if current_temp < target_temp {
                    // Need to increase temperature
                    if !heater_on {
                        // First turn on heater if it's off
                        if let Some(turn_on_wrapper) = action_wrappers
                            .iter()
                            .find(|a| a.id == "TurnHeaterOnAction")
                        {
                            plan.push(turn_on_wrapper.action.clone());
                        }
                    }

                    // Then add temperature increase action
                    if let Some(increase_wrapper) = action_wrappers
                        .iter()
                        .find(|a| a.id == "IncreaseTemperatureAction")
                    {
                        plan.push(increase_wrapper.action.clone());
                    }
                } else if current_temp > target_temp && heater_on {
                    // Need to decrease temperature - turn off heater
                    if let Some(turn_off_wrapper) = action_wrappers
                        .iter()
                        .find(|a| a.id == "TurnHeaterOffAction")
                    {
                        plan.push(turn_off_wrapper.action.clone());
                    }
                }
            }
        }

        println!("Plan created with {} steps", plan.len());
        plan
    }
}

#[tokio::main]
async fn main() {
    println!("Starting Simplified Actor-based Temperature Control Agent");

    // Create a local task set for actor-based tasks
    let local = tokio::task::LocalSet::new();

    local
        .run_until(async move {
            // Create shared world state
            let world_state = Arc::new(Mutex::new(WorldState::new()));

            // Create sensors
            let sensors = vec![
                Arc::new(TemperatureSensor::new(Arc::clone(&world_state))) as Arc<dyn SensorFn>,
                Arc::new(HeaterStateSensor::new(Arc::clone(&world_state))) as Arc<dyn SensorFn>,
            ];

            // Create actions
            let actions = vec![
                Arc::new(TurnHeaterOnAction::new(Arc::clone(&world_state))) as Arc<dyn ActionFn>,
                Arc::new(TurnHeaterOffAction::new(Arc::clone(&world_state))) as Arc<dyn ActionFn>,
                Arc::new(IncreaseTemperatureAction::new(Arc::clone(&world_state)))
                    as Arc<dyn ActionFn>,
            ];

            // Create planner
            let planner = Arc::new(TemperatureControlPlanner) as Arc<dyn PlannerFn>;

            // Create controller
            let controller =
                ActorAutomatonController::new("TemperatureController", sensors, actions, planner);

            // Set a goal to reach the target temperature
            let mut goal = HashMap::new();
            goal.insert(
                "target_temperature_reached".to_string(),
                Fact::new("target_temperature_reached", "true", "UserGoal"),
            );
            controller.set_goal(goal).await;

            // Start the automaton
            controller.start().await;

            // Run for a while to let the automaton work
            for i in 1..=10 {
                println!("\n--- Cycle {} ---", i);

                // Print current state
                {
                    let state = world_state.lock().unwrap();
                    println!(
                        "Current temperature: {}, Target: {}, Heater: {}",
                        state.temperature,
                        state.target_temperature,
                        if state.heater_on { "ON" } else { "OFF" }
                    );

                    // If we've reached the target temperature, we're done
                    if state.temperature >= state.target_temperature {
                        println!("Target temperature reached!");

                        // Drop the lock before acquiring it again
                        drop(state);

                        if i < 5 {
                            // Change the target to demonstrate the system adapting
                            println!("Setting new target temperature to 26°C");
                            let mut state = world_state.lock().unwrap();
                            state.target_temperature = 26;
                        }
                    }
                }

                // Wait for the next cycle
                sleep(Duration::from_secs(1)).await;
            }

            // Stop the automaton when done
            controller.stop().await;
            println!("Shutting down Temperature Control Agent");
        })
        .await;
}
