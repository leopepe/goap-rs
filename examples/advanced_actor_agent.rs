use goaprs::utils::actor::{ActionFn, Fact, PlannerFn, SensorFn};
use goaprs::utils::ActorAutomatonController;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::sleep;

/// A shared state to demonstrate communication between components
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
    name: String, // Name to identify the action
}

impl TurnHeaterOnAction {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self {
            world_state,
            name: "TurnHeaterOnAction".to_string(),
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn target_state(&self) -> HashMap<String, Fact> {
        let mut facts = HashMap::new();
        facts.insert(
            "heater_on".to_string(),
            Fact::new("heater_on", "true", "TurnHeaterOnAction"),
        );
        facts
    }

    fn preconditions(&self) -> HashMap<String, Fact> {
        let mut facts = HashMap::new();
        facts.insert(
            "heater_on".to_string(),
            Fact::new("heater_on", "false", "TurnHeaterOnAction"),
        );
        facts
    }

    fn cost(&self) -> f32 {
        1.0
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
    name: String, // Name to identify the action
}

impl TurnHeaterOffAction {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self {
            world_state,
            name: "TurnHeaterOffAction".to_string(),
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn target_state(&self) -> HashMap<String, Fact> {
        let mut facts = HashMap::new();
        facts.insert(
            "heater_on".to_string(),
            Fact::new("heater_on", "false", "TurnHeaterOffAction"),
        );
        facts
    }

    fn preconditions(&self) -> HashMap<String, Fact> {
        let mut facts = HashMap::new();
        facts.insert(
            "heater_on".to_string(),
            Fact::new("heater_on", "true", "TurnHeaterOffAction"),
        );
        facts
    }

    fn cost(&self) -> f32 {
        0.5
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
    name: String, // Name to identify the action
}

impl IncreaseTemperatureAction {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self {
            world_state,
            name: "IncreaseTemperatureAction".to_string(),
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn target_state(&self) -> HashMap<String, Fact> {
        let mut facts = HashMap::new();
        facts.insert(
            "temperature_increased".to_string(),
            Fact::new("temperature_increased", "true", "IncreaseTemperatureAction"),
        );
        facts
    }

    fn preconditions(&self) -> HashMap<String, Fact> {
        let mut facts = HashMap::new();
        facts.insert(
            "heater_on".to_string(),
            Fact::new("heater_on", "true", "IncreaseTemperatureAction"),
        );
        facts
    }

    fn cost(&self) -> f32 {
        2.0
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

// A wrapper to help with action identification
struct IdentifiableAction {
    name: String,
    action: Arc<dyn ActionFn>,
}

impl IdentifiableAction {
    fn new(name: &str, action: Arc<dyn ActionFn>) -> Self {
        Self {
            name: name.to_string(),
            action,
        }
    }

    fn name(&self) -> &str {
        &self.name
    }

    fn action(&self) -> Arc<dyn ActionFn> {
        self.action.clone()
    }
}

// Planner
struct TemperatureControlPlanner;

#[async_trait::async_trait]
impl PlannerFn for TemperatureControlPlanner {
    async fn plan(
        &self,
        state: &HashMap<String, Fact>,
        goal: &HashMap<String, Fact>,
        actions: &[Arc<dyn ActionFn>],
    ) -> Vec<Arc<dyn ActionFn>> {
        println!("Planning with current state: {:?}", state);

        // Convert actions to identifiable wrappers for better planning
        // This is just a workaround for the example
        let mut identifiable_actions = Vec::new();
        for (i, action) in actions.iter().enumerate() {
            // We'll use index as part of identification
            let name = match i {
                0 => "TurnHeaterOnAction",
                1 => "TurnHeaterOffAction",
                2 => "IncreaseTemperatureAction",
                _ => "UnknownAction",
            };
            identifiable_actions.push(IdentifiableAction::new(name, action.clone()));
        }

        // A simple planning strategy
        let mut plan = Vec::new();

        if goal.contains_key("target_temperature_reached") {
            if let (Some(temp_fact), Some(target_fact)) =
                (state.get("temperature"), state.get("target_temperature"))
            {
                let current_temp: i32 = temp_fact.data().parse().unwrap_or(0);
                let target_temp: i32 = target_fact.data().parse().unwrap_or(0);

                println!(
                    "Current temp: {}, Target temp: {}",
                    current_temp, target_temp
                );

                if current_temp < target_temp {
                    // Need to increase temperature
                    let heater_on = state
                        .get("heater_on")
                        .map(|f| f.data() == "true")
                        .unwrap_or(false);

                    if !heater_on {
                        // First, find the action that turns on the heater
                        if let Some(turn_on_action) = identifiable_actions
                            .iter()
                            .find(|a| a.name() == "TurnHeaterOnAction")
                        {
                            plan.push(turn_on_action.action());
                        }
                    }

                    // Then add an action to increase temperature
                    if let Some(increase_action) = identifiable_actions
                        .iter()
                        .find(|a| a.name() == "IncreaseTemperatureAction")
                    {
                        plan.push(increase_action.action());
                    }
                } else if current_temp > target_temp {
                    // Need to decrease temperature - turn off heater
                    let heater_on = state
                        .get("heater_on")
                        .map(|f| f.data() == "true")
                        .unwrap_or(false);

                    if heater_on {
                        // Find action to turn heater off
                        if let Some(turn_off_action) = identifiable_actions
                            .iter()
                            .find(|a| a.name() == "TurnHeaterOffAction")
                        {
                            plan.push(turn_off_action.action());
                        }
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
    println!("Starting Advanced Actor-based Temperature Control Agent");

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
                sleep(Duration::from_secs(1)).await;

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
                        // We need to drop the current lock before acquiring it again
                        drop(state);
                        if i < 10 {
                            // Change the target to demonstrate the system adapting
                            println!("Setting new target temperature to 26°C");
                            let mut state = world_state.lock().unwrap();
                            state.target_temperature = 26;
                        }
                    }
                }
            }

            // Stop the automaton when done
            controller.stop().await;
            println!("Shutting down Temperature Control Agent");
        })
        .await;
}
