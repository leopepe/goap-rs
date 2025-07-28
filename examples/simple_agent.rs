use std::collections::HashMap;
use std::error::Error;
use std::sync::Arc;
use std::time::Duration;

use goaprs::action::Action;
use goaprs::error::Result;
use goaprs::sensor::{SensorFn, SensorResponse, Sensors};
// use goaprs::utils::actor::ActionFn;
use goaprs::utils::automaton::AutomatonController;

use async_trait::async_trait;
use tokio::sync::Mutex;

/// A simple environment to store lamp state
struct LampEnvironment {
    is_on: Mutex<bool>,
}

impl LampEnvironment {
    fn new(is_on: bool) -> Self {
        Self {
            is_on: Mutex::new(is_on),
        }
    }
}

/// Simple lamp sensor implementation
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
        let is_on = *self.env.is_on.lock().await;
        let status = if is_on { "on" } else { "off" };
        Ok(SensorResponse::new(status.to_string(), String::new(), 0))
    }
}

#[tokio::main]
async fn main() -> std::result::Result<(), Box<dyn Error>> {
    // Create shared environment
    let env = Arc::new(LampEnvironment::new(false));

    // Create sensors collection
    let mut sensors = Sensors::new();

    // Add lamp status sensor
    sensors.add("check_lamp", "lamp_status", LampSensor::new(env.clone()))?;

    // Add uptime sensor with simple closure
    sensors.add(
        "uptime",
        "system_uptime",
        goaprs::sensor::FnSensor::new(|| async {
            Ok(("12 days, 5 hours, 10 minutes".to_string(), String::new(), 0))
        }),
    )?;

    // Create actions
    let mut actions = Vec::new();

    // Create turn lamp on action
    let mut turn_on = Action::new("turn_lamp_on", 1.0)?;
    turn_on.preconditions.set("lamp_status", "false");
    turn_on.effects.set("lamp_status", "true");
    actions.push(turn_on);

    // Create turn lamp off action
    let mut turn_off = Action::new("turn_lamp_off", 1.0)?;
    turn_off.preconditions.set("lamp_status", "true");
    turn_off.effects.set("lamp_status", "false");
    actions.push(turn_off);

    // Create simple action
    let mut simple_action = Action::new("simple_action", 0.5)?;
    simple_action.effects.set("simple_task", "true");
    actions.push(simple_action);

    // Set initial world state
    let mut world_state = HashMap::new();
    world_state.insert("lamp_status".to_string(), "off".to_string());
    world_state.insert("simple_task".to_string(), "pending".to_string());

    // Create the automaton controller
    let controller = AutomatonController::new(
        actions,
        sensors.iter().cloned().collect(),
        "smart_home_agent",
        world_state,
    );

    // Set a goal to turn the lamp on
    let mut goal = HashMap::new();
    goal.insert("lamp_status".to_string(), "on".to_string());
    goal.insert("simple_task".to_string(), "done".to_string());

    controller.set_goal(goal).await?;

    // Run a manual GOAP cycle to see what happens
    println!("-- Starting manual GOAP cycle --");

    // Sense the environment
    println!("Sensing the environment...");
    controller.automaton().sense().await?;

    // Plan actions
    println!("Planning actions...");

    let plan = controller.automaton().plan().await?;
    println!("Generated plan with {} actions:", plan.len());
    for (i, action) in plan.iter().enumerate() {
        println!("  Step {}: {}", i + 1, action.name);
    }

    // Execute actions
    println!("Executing actions...");
    let responses = controller.automaton().act().await?;
    for (i, response) in responses.iter().enumerate() {
        println!("  Result {}: {}", i + 1, response.stdout());
    }

    // Sense again to see changes
    println!("Sensing again to see changes...");
    controller.automaton().sense().await?;

    let final_state = controller.automaton().world_state().await?;
    println!("Final world state: {:?}", final_state);

    println!("\n-- Now starting automated controller --");
    println!("Press Ctrl+C to exit");

    // Start the controller (will run in a background thread)
    controller.start().await?;

    // Change the goal after 10 seconds to turn the lamp off
    tokio::time::sleep(Duration::from_secs(10)).await;
    println!("\nChanging goal to turn lamp OFF");

    let mut new_goal = HashMap::new();
    new_goal.insert("lamp_status".to_string(), "off".to_string());
    new_goal.insert("simple_task".to_string(), "done".to_string());
    controller.set_goal(new_goal).await?;

    // Wait for a while to see the controller working
    tokio::time::sleep(Duration::from_secs(20)).await;

    // Stop the controller
    controller.stop().await?;
    println!("Controller stopped");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    // Mock lamp sensor for testing
    struct MockLampSensor {
        is_on: Arc<Mutex<bool>>,
        call_count: Arc<AtomicUsize>,
    }

    impl MockLampSensor {
        fn new(is_on: bool) -> Self {
            Self {
                is_on: Arc::new(Mutex::new(is_on)),
                call_count: Arc::new(AtomicUsize::new(0)),
            }
        }
    }

    #[async_trait]
    impl SensorFn for MockLampSensor {
        async fn exec(&self) -> Result<SensorResponse> {
            self.call_count.fetch_add(1, Ordering::SeqCst);
            let is_on = *self.is_on.lock().await;
            let status = if is_on { "on" } else { "off" };
            Ok(SensorResponse::new(status.to_string(), String::new(), 0))
        }
    }

    // Mock action for testing
    struct MockAction {
        name: String,
        executed: Arc<AtomicUsize>,
        response: String,
    }

    impl MockAction {
        fn new(name: &str, response: &str) -> Self {
            Self {
                name: name.to_string(),
                executed: Arc::new(AtomicUsize::new(0)),
                response: response.to_string(),
            }
        }

        fn execution_count(&self) -> usize {
            self.executed.load(Ordering::SeqCst)
        }
    }

    #[async_trait]
    impl ActionFn for MockAction {
        async fn exec(&self, _world_state: &HashMap<String, goaprs::utils::actor::Fact>) -> bool {
            self.executed.fetch_add(1, Ordering::SeqCst);
            true
        }
    }

    #[tokio::test]
    async fn test_lamp_sensor() {
        // Create sensor
        let sensor = MockLampSensor::new(true);
        let call_count = sensor.call_count.clone();

        // Test sensor
        let result = sensor.exec().await.unwrap();
        assert_eq!(result.stdout(), "on");
        assert_eq!(call_count.load(Ordering::SeqCst), 1);

        // Change state
        *sensor.is_on.lock().await = false;

        // Test again
        let result = sensor.exec().await.unwrap();
        assert_eq!(result.stdout(), "off");
        assert_eq!(call_count.load(Ordering::SeqCst), 2);
    }

    #[tokio::test]
    async fn test_lamp_actions() {
        // Create environment
        let env = Arc::new(LampEnvironment::new(false));

        // Create actions
        let turn_on = TurnLampOnAction::new(env.clone());
        let turn_off = TurnLampOffAction::new(env.clone());

        // Test turn on
        assert!(turn_on.exec(&HashMap::new()).await);
        assert!(*env.is_on.lock().await);

        // Test turn off
        assert!(turn_off.exec(&HashMap::new()).await);
        assert!(!*env.is_on.lock().await);
    }

    #[tokio::test]
    async fn test_simple_goap_cycle() {
        // Create sensors
        let mut sensors = Sensors::new();
        let lamp_sensor = MockLampSensor::new(false);
        sensors
            .add("check_lamp", "lamp_status", lamp_sensor)
            .unwrap();

        // Create actions
        let mut actions = Vec::new();

        // Turn lamp on action
        let mut turn_on = Action::new("turn_lamp_on", 1.0).unwrap();
        turn_on.preconditions.set("lamp_status", false); // lamp is off
        turn_on.effects.set("lamp_status", true); // lamp will be on
        actions.push(turn_on);

        // Initial world state
        let mut world_state = HashMap::new();
        world_state.insert("lamp_status".to_string(), "off".to_string());

        // Create the controller
        let controller = AutomatonController::new(
            actions,
            sensors.iter().cloned().collect(),
            "test_agent",
            world_state,
        );

        // Set goal (turn lamp on)
        let mut goal = HashMap::new();
        goal.insert("lamp_status".to_string(), "on".to_string());
        controller.set_goal(goal).await.unwrap();

        // Run through GOAP cycle
        controller.automaton().sense().await.unwrap();
        let plan = controller.automaton().plan().await.unwrap();

        // Verify plan contains our action
        assert_eq!(plan.len(), 1);
        assert_eq!(plan[0].name, "turn_lamp_on");

        // This test only validates planning, not execution
        // In a real system, we would also call act()
    }
}
