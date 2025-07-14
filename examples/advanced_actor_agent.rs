use goaprs::utils::actor::{ActionFn, Fact, PlannerFn, SensorFn};
use goaprs::utils::ActorAutomatonController;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::time::sleep;

// Shared world state
#[derive(Debug)]
struct WorldState {
    heater_on: bool,
    temperature_increased: bool,
}

impl WorldState {
    fn new() -> Self {
        Self {
            heater_on: false,
            temperature_increased: false,
        }
    }
}

// Sensors
#[derive(Debug)]
struct HeaterSensor {
    world_state: Arc<Mutex<WorldState>>,
}

impl HeaterSensor {
    fn new(world_state: Arc<Mutex<WorldState>>) -> Self {
        Self { world_state }
    }
}

#[async_trait::async_trait]
impl SensorFn for HeaterSensor {
    async fn exec(&self, _world_state: &HashMap<String, Fact>) -> Vec<Fact> {
        let state = self.world_state.lock().unwrap();

        vec![Fact::new(
            "heater_on",
            &state.heater_on.to_string(),
            "HeaterSensor",
        )]
    }
}

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

        vec![Fact::new(
            "temperature_increased",
            &state.temperature_increased.to_string(),
            "TemperatureSensor",
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
        // When we turn off the heater, we lose the temperature increase
        state.temperature_increased = false;
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
            state.temperature_increased = true;
            println!("  Temperature increased successfully!");
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

        let mut plan = Vec::new();

        // Check if we need to increase temperature
        if let Some(goal_temp_increased) = goal.get("temperature_increased") {
            if goal_temp_increased.data() == "true" {
                // Check current state
                let heater_on = world_state
                    .get("heater_on")
                    .map(|f| f.data() == "true")
                    .unwrap_or(false);

                let temp_increased = world_state
                    .get("temperature_increased")
                    .map(|f| f.data() == "true")
                    .unwrap_or(false);

                if !temp_increased {
                    // We need to increase temperature
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
                }
            }
        }

        // Check if we need to turn off heater
        if let Some(goal_heater) = goal.get("heater_on") {
            if goal_heater.data() == "false" {
                // Check current state
                let heater_on = world_state
                    .get("heater_on")
                    .map(|f| f.data() == "true")
                    .unwrap_or(false);

                if heater_on {
                    // Need to turn off heater
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

// Function to print the current state
fn print_state(world_state: &Arc<Mutex<WorldState>>) {
    let state = world_state.lock().unwrap();
    println!("Current state:");
    println!("  Heater: {}", if state.heater_on { "ON" } else { "OFF" });
    println!(
        "  Temperature increased: {}",
        if state.temperature_increased {
            "YES"
        } else {
            "NO"
        }
    );
}

#[tokio::main]
async fn main() {
    println!("Starting Advanced Temperature Control Agent");

    // Create a local task set for actor-based tasks
    let local = tokio::task::LocalSet::new();

    local
        .run_until(async move {
            // Create shared world state
            let world_state = Arc::new(Mutex::new(WorldState::new()));

            // Create sensors
            let sensors = vec![
                Arc::new(HeaterSensor::new(Arc::clone(&world_state))) as Arc<dyn SensorFn>,
                Arc::new(TemperatureSensor::new(Arc::clone(&world_state))) as Arc<dyn SensorFn>,
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

            // Initial goal: increase temperature
            let mut goal = HashMap::new();
            goal.insert(
                "temperature_increased".to_string(),
                Fact::new("temperature_increased", "true", "UserGoal"),
            );
            controller.set_goal(goal).await;

            // Start the automaton
            controller.start().await;

            // Simulation loop
            for cycle in 1..=5 {
                println!("\n--- Cycle {} ---", cycle);

                // Print current state
                print_state(&world_state);

                // Check if we've reached the goal
                {
                    let state = world_state.lock().unwrap();

                    // If we've reached our initial goal and it's cycle 3, change the goal
                    if state.temperature_increased && cycle == 3 {
                        println!("Temperature increase achieved!");
                        println!("New goal: Turn off heater to save energy");

                        // Drop the lock before setting a new goal
                        drop(state);

                        // Change goal to turning off the heater
                        let mut new_goal = HashMap::new();
                        new_goal.insert(
                            "heater_on".to_string(),
                            Fact::new("heater_on", "false", "UserGoal"),
                        );
                        controller.set_goal(new_goal).await;
                    }
                }

                // Wait before next cycle
                sleep(Duration::from_secs(1)).await;
            }

            // Stop the automaton when done
            controller.stop().await;

            // Print final state
            println!("\nFinal state:");
            print_state(&world_state);
            println!("Advanced temperature control simulation complete!");
        })
        .await;
}
