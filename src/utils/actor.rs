//! Actor model implementation for the GOAP automaton system
//! This file provides an alternative implementation of the automaton.rs functionality
//! using the Actor Model for concurrency, allowing sensors to run in parallel with actions
//! and the planner, with automatic plan validation when world state changes.

use actix::dev::MessageResponse;
use actix::{
    Actor, ActorFutureExt, Addr, AsyncContext, Context, Handler, Message, ResponseFuture,
    WrapFuture,
};
use async_trait::async_trait;
use futures::future::join_all;
use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Represents a piece of information with binding, data, timestamp, and parent sensor
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct Fact {
    /// The binding or key for this fact
    binding: String,
    /// The data or value for this fact
    data: String,
    /// When this fact was created or updated (timestamp in seconds since epoch)
    timestamp: u64,
    /// The sensor that provided this fact
    parent_sensor: String,
}

impl Fact {
    /// Creates a new fact with the given binding, data, and parent sensor
    pub fn new(binding: &str, data: &str, parent_sensor: &str) -> Self {
        Fact {
            binding: binding.to_string(),
            data: data.to_string(),
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            parent_sensor: parent_sensor.to_string(),
        }
    }

    /// Returns the binding for this fact
    pub fn binding(&self) -> &str {
        &self.binding
    }

    /// Returns the data for this fact
    pub fn data(&self) -> &str {
        &self.data
    }

    /// Returns the timestamp for this fact
    pub fn timestamp(&self) -> u64 {
        self.timestamp
    }

    /// Returns the parent sensor for this fact
    pub fn parent_sensor(&self) -> &str {
        &self.parent_sensor
    }
}

impl fmt::Display for Fact {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}={} [{}]", self.binding, self.data, self.parent_sensor)
    }
}

/// The possible states of the automaton
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Waiting for orders or goals
    WaitingOrders,
    /// Sensing the environment
    Sensing,
    /// Planning actions
    Planning,
    /// Executing actions
    Acting,
}

/// Trait for sensor functionality
#[async_trait]
pub trait SensorFn: Send + Sync {
    /// Executes the sensor and returns a vector of facts
    async fn exec(&self, world_state: &HashMap<String, Fact>) -> Vec<Fact>;
}

/// Trait for action functionality
#[async_trait]
pub trait ActionFn: Send + Sync {
    /// Executes the action and returns success or failure
    async fn exec(&self, world_state: &HashMap<String, Fact>) -> bool;
}

/// Trait for planner functionality
#[async_trait]
pub trait PlannerFn: Send {
    /// Plans actions to achieve the goal based on the world state
    async fn plan(
        &self,
        world_state: &HashMap<String, Fact>,
        goal: &HashMap<String, Fact>,
        available_actions: &[Arc<dyn ActionFn>],
    ) -> Vec<Arc<dyn ActionFn>>;
}

// Actor messages

/// Message to request sensing the environment
#[derive(Message)]
#[rtype(result = "SenseResult")]
struct SenseRequest {
    #[allow(dead_code)]
    current_world_state: HashMap<String, Fact>,
}

/// Message to notify of world state changes
#[derive(Message)]
#[rtype(result = "()")]
struct WorldStateChanged {
    new_state: HashMap<String, Fact>,
}

/// Message to request planning
#[derive(Message)]
#[rtype(result = "Vec<Arc<dyn ActionFn>>")]
struct PlanRequest {
    world_state: HashMap<String, Fact>,
    goal: HashMap<String, Fact>,
}

/// Message to request action execution
#[derive(Message)]
#[rtype(result = "bool")]
struct ActionRequest {
    action: Arc<dyn ActionFn>,
    world_state: HashMap<String, Fact>,
}

/// Message to check plan validity
#[derive(Message)]
#[rtype(result = "bool")]
struct CheckPlanValidity {
    current_plan: Vec<Arc<dyn ActionFn>>,
    world_state: HashMap<String, Fact>,
    goal: HashMap<String, Fact>,
}

/// Message to set a goal
#[derive(Message)]
#[rtype(result = "()")]
struct SetGoalRequest {
    goal: HashMap<String, Fact>,
}

/// Message to start the automaton
#[derive(Message)]
#[rtype(result = "()")]
struct StartRequest {}

/// Message to stop the automaton
#[derive(Message)]
#[rtype(result = "()")]
struct StopRequest {}

/// Actor that handles sensors
pub struct SensorActor {
    sensors: Vec<Arc<dyn SensorFn>>,
    automaton_addr: Option<Addr<AutomatonActor>>,
}

impl SensorActor {
    pub fn new(sensors: Vec<Arc<dyn SensorFn>>) -> Self {
        SensorActor {
            sensors,
            automaton_addr: None,
        }
    }

    pub fn set_automaton(&mut self, addr: Addr<AutomatonActor>) {
        self.automaton_addr = Some(addr);
    }
}

impl Actor for SensorActor {
    type Context = Context<Self>;

    fn started(&mut self, ctx: &mut Self::Context) {
        // Start periodic sensing when actor starts
        ctx.run_interval(Duration::from_millis(500), |act, ctx| {
            SensorActor::run_sensing_cycle(act, ctx);
        });
    }
}

// Implement methods separately from Actor trait
impl SensorActor {
    // Run the sensing cycle
    fn run_sensing_cycle(act: &mut Self, ctx: &mut Context<Self>) {
        let Some(automaton_addr) = &act.automaton_addr else {
            return; // No automaton address available
        };

        // Step 1: Request world state from automaton
        Self::fetch_world_state(automaton_addr.clone(), act, ctx);
    }

    // Fetch the current world state from the automaton
    fn fetch_world_state(
        automaton_addr: Addr<AutomatonActor>,
        act: &mut Self,
        ctx: &mut Context<Self>,
    ) {
        let fut = async move {
            match automaton_addr.send(GetWorldState {}).await {
                Ok(WorldStateResult(world_state)) => Some(world_state),
                Err(_) => None,
            }
        };

        let fut = fut.into_actor(act).then(move |world_state_opt, act, ctx| {
            if let Some(world_state) = world_state_opt {
                Self::process_sensors(world_state, act, ctx);
            }
            actix::fut::ready(())
        });

        ctx.spawn(fut);
    }

    // Process all sensors with the current world state
    fn process_sensors(
        world_state: HashMap<String, Fact>,
        act: &mut Self,
        ctx: &mut Context<Self>,
    ) {
        // Create clones for async execution
        let sensors = act.sensors.clone();
        let automaton = act.automaton_addr.clone();

        // Run sensors and notify of changes
        let fut = async move {
            let all_facts = Self::collect_sensor_data(&sensors, &world_state).await;
            Self::notify_if_changed(all_facts, world_state, automaton).await;
        };

        ctx.spawn(fut.into_actor(act));
    }

    // Collect data from all sensors in parallel
    async fn collect_sensor_data(
        sensors: &[Arc<dyn SensorFn>],
        world_state: &HashMap<String, Fact>,
    ) -> HashMap<String, Fact> {
        // Run all sensors in parallel
        let sensor_futures = sensors
            .iter()
            .map(|sensor| sensor.exec(world_state))
            .collect::<Vec<_>>();

        let results = join_all(sensor_futures).await;

        // Combine all sensor results
        let mut all_facts = HashMap::new();
        for facts in results {
            for fact in facts {
                all_facts.insert(fact.binding().to_string(), fact);
            }
        }

        all_facts
    }

    // Notify automaton if world state has changed
    async fn notify_if_changed(
        new_facts: HashMap<String, Fact>,
        old_state: HashMap<String, Fact>,
        automaton: Option<Addr<AutomatonActor>>,
    ) {
        // Only notify if there are changes and we have facts
        if new_facts != old_state && !new_facts.is_empty() {
            if let Some(addr) = automaton {
                let _ = addr
                    .send(WorldStateChanged {
                        new_state: new_facts,
                    })
                    .await;
            }
        }
    }
}

// Response types that implement MessageResponse
#[derive(Debug)]
#[allow(dead_code)]
struct SenseResult(HashMap<String, Fact>);

impl<A> MessageResponse<A, SenseRequest> for SenseResult
where
    A: Actor,
    A::Context: AsyncContext<A>,
{
    fn handle(self, _: &mut A::Context, tx: Option<actix::dev::OneshotSender<Self>>) {
        if let Some(tx) = tx {
            let _ = tx.send(self);
        }
    }
}

#[derive(Debug)]
struct WorldStateResult(HashMap<String, Fact>);

impl<A> MessageResponse<A, GetWorldState> for WorldStateResult
where
    A: Actor,
    A::Context: AsyncContext<A>,
{
    fn handle(self, _: &mut A::Context, tx: Option<actix::dev::OneshotSender<Self>>) {
        if let Some(tx) = tx {
            let _ = tx.send(self);
        }
    }
}

#[derive(Debug)]
struct GoalResult(HashMap<String, Fact>);

impl<A> MessageResponse<A, GetGoal> for GoalResult
where
    A: Actor,
    A::Context: AsyncContext<A>,
{
    fn handle(self, _: &mut A::Context, tx: Option<actix::dev::OneshotSender<Self>>) {
        if let Some(tx) = tx {
            let _ = tx.send(self);
        }
    }
}

#[derive(Message)]
#[rtype(result = "WorldStateResult")]
struct GetWorldState;

/// Actor that handles planning
pub struct PlannerActor {
    planner: Arc<dyn PlannerFn>,
    available_actions: Vec<Arc<dyn ActionFn>>,
}

impl PlannerActor {
    pub fn new(planner: Arc<dyn PlannerFn>, available_actions: Vec<Arc<dyn ActionFn>>) -> Self {
        PlannerActor {
            planner,
            available_actions,
        }
    }
}

impl Actor for PlannerActor {
    type Context = Context<Self>;
}

impl Handler<PlanRequest> for PlannerActor {
    type Result = ResponseFuture<Vec<Arc<dyn ActionFn>>>;

    fn handle(&mut self, msg: PlanRequest, _ctx: &mut Context<Self>) -> Self::Result {
        let planner = self.planner.clone();
        let actions = self.available_actions.clone();
        Box::pin(async move { planner.plan(&msg.world_state, &msg.goal, &actions).await })
    }
}

impl Handler<CheckPlanValidity> for PlannerActor {
    type Result = ResponseFuture<bool>;

    fn handle(&mut self, msg: CheckPlanValidity, _ctx: &mut Context<Self>) -> Self::Result {
        let planner = self.planner.clone();
        let actions = self.available_actions.clone();

        // Make a simple check based on length
        // In a real system, you'd want a more sophisticated comparison
        let current_plan_len = msg.current_plan.len();

        Box::pin(async move {
            let new_plan = planner.plan(&msg.world_state, &msg.goal, &actions).await;
            // If plans are different lengths, they're definitely different
            new_plan.len() == current_plan_len
        })
    }
}

/// Actor that handles executing actions
pub struct ActionActor;

impl Default for ActionActor {
    fn default() -> Self {
        Self::new()
    }
}

impl ActionActor {
    pub fn new() -> Self {
        ActionActor
    }
}

impl Actor for ActionActor {
    type Context = Context<Self>;
}

impl Handler<ActionRequest> for ActionActor {
    type Result = ResponseFuture<bool>;

    fn handle(&mut self, msg: ActionRequest, _ctx: &mut Context<Self>) -> Self::Result {
        let action = msg.action;
        let world_state = msg.world_state;
        Box::pin(async move { action.exec(&world_state).await })
    }
}

/// The main automaton actor that coordinates the other actors
pub struct AutomatonActor {
    #[allow(dead_code)]
    name: String,
    state: State,
    world_state: HashMap<String, Fact>,
    goal: HashMap<String, Fact>,
    #[allow(dead_code)]
    working_memory: HashMap<String, Fact>,
    action_plan: Vec<Arc<dyn ActionFn>>,
    sensor_actor: Option<Addr<SensorActor>>,
    planner_actor: Option<Addr<PlannerActor>>,
    action_actor: Option<Addr<ActionActor>>,
    running: bool,
}

impl AutomatonActor {
    pub fn new(name: &str) -> Self {
        AutomatonActor {
            name: name.to_string(),
            state: State::WaitingOrders,
            world_state: HashMap::new(),
            goal: HashMap::new(),
            working_memory: HashMap::new(),
            action_plan: Vec::new(),
            sensor_actor: None,
            planner_actor: None,
            action_actor: None,
            running: false,
        }
    }

    pub fn set_sensor_actor(&mut self, addr: Addr<SensorActor>) {
        self.sensor_actor = Some(addr);
    }

    pub fn set_planner_actor(&mut self, addr: Addr<PlannerActor>) {
        self.planner_actor = Some(addr);
    }

    pub fn set_action_actor(&mut self, addr: Addr<ActionActor>) {
        self.action_actor = Some(addr);
    }

    fn execute_next_action(&self, ctx: &mut Context<Self>) {
        if !self.action_plan.is_empty() && self.running {
            // Clone what we need for async execution
            let action = self.action_plan[0].clone();
            let world_state = self.world_state.clone();
            let action_actor = self.action_actor.clone();
            let remaining_plan = self.action_plan[1..].to_vec();
            let addr = ctx.address();

            ctx.spawn(
                async move {
                    if let Some(actor) = action_actor {
                        let success = actor
                            .send(ActionRequest {
                                action,
                                world_state,
                            })
                            .await;

                        if let Ok(true) = success {
                            // Action succeeded, continue with the next action
                            addr.send(ContinueWithPlan { remaining_plan }).await.ok();
                        } else {
                            // Action failed, replan
                            addr.send(NeedReplan {}).await.ok();
                        }
                    }
                }
                .into_actor(self),
            );
        } else if self.running {
            // No actions left, go back to waiting orders
            ctx.address().do_send(SetState {
                state: State::WaitingOrders,
            });
        }
    }
}

impl Actor for AutomatonActor {
    type Context = Context<Self>;
}

#[derive(Message)]
#[rtype(result = "()")]
struct SetState {
    state: State,
}

#[derive(Message)]
#[rtype(result = "()")]
struct ContinueWithPlan {
    remaining_plan: Vec<Arc<dyn ActionFn>>,
}

#[derive(Message)]
#[rtype(result = "()")]
struct NeedReplan;

impl Handler<SetState> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: SetState, _ctx: &mut Context<Self>) -> Self::Result {
        self.state = msg.state;
    }
}

impl Handler<GetWorldState> for AutomatonActor {
    type Result = WorldStateResult;

    fn handle(&mut self, _msg: GetWorldState, _ctx: &mut Context<Self>) -> Self::Result {
        WorldStateResult(self.world_state.clone())
    }
}

impl Handler<WorldStateChanged> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: WorldStateChanged, ctx: &mut Context<Self>) -> Self::Result {
        let _old_state = self.world_state.clone();
        self.world_state = msg.new_state;

        // If we have a plan and the world state changed, check if the plan is still valid
        if !self.action_plan.is_empty() && self.running {
            if let Some(planner) = &self.planner_actor {
                let plan = self.action_plan.clone();
                let world_state = self.world_state.clone();
                let goal = self.goal.clone();
                let addr = ctx.address();
                let planner_clone = planner.clone();

                // Use spawn to avoid borrowing self
                actix::spawn(async move {
                    let valid = planner_clone
                        .send(CheckPlanValidity {
                            current_plan: plan,
                            world_state,
                            goal,
                        })
                        .await;

                    if let Ok(false) = valid {
                        // Plan is no longer valid, need to replan
                        addr.send(NeedReplan {}).await.ok();
                    }
                });
            }
        }
    }
}

impl Handler<SetGoalRequest> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: SetGoalRequest, _ctx: &mut Context<Self>) -> Self::Result {
        self.goal = msg.goal;
    }
}

impl Handler<StartRequest> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, _msg: StartRequest, ctx: &mut Context<Self>) -> Self::Result {
        if !self.running {
            self.running = true;

            // Start the automaton cycle
            ctx.address().do_send(RunAutomatonCycle {});
        }
    }
}

impl Handler<StopRequest> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, _msg: StopRequest, _ctx: &mut Context<Self>) -> Self::Result {
        self.running = false;
    }
}

#[derive(Message)]
#[rtype(result = "()")]
struct RunAutomatonCycle;

impl Handler<RunAutomatonCycle> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, _msg: RunAutomatonCycle, ctx: &mut Context<Self>) -> Self::Result {
        if !self.running {
            return;
        }

        match self.state {
            State::WaitingOrders => {
                if !self.goal.is_empty() {
                    // If we have a goal, start planning
                    ctx.address().do_send(SetState {
                        state: State::Planning,
                    });
                    ctx.address().do_send(RunAutomatonCycle {});
                } else {
                    // No goal yet, check again later
                    ctx.run_later(Duration::from_millis(100), |_, ctx| {
                        ctx.address().do_send(RunAutomatonCycle {});
                    });
                }
            }
            State::Planning => {
                if let Some(planner) = &self.planner_actor {
                    let world_state = self.world_state.clone();
                    let goal = self.goal.clone();
                    let addr = ctx.address();
                    let planner_clone = planner.clone();

                    // Use actix::spawn to avoid borrowing self
                    actix::spawn(async move {
                        let plan_result =
                            planner_clone.send(PlanRequest { world_state, goal }).await;

                        if let Ok(plan) = plan_result {
                            addr.send(SetActionPlan { plan }).await.ok();
                        } else {
                            // Planning failed, go back to waiting orders
                            addr.send(SetState {
                                state: State::WaitingOrders,
                            })
                            .await
                            .ok();
                        }
                    });
                }
            }
            State::Acting => {
                self.execute_next_action(ctx);
            }
            State::Sensing => {
                // In this implementation, sensing happens continuously in the SensorActor
                // So we just need to transition to the next state
                ctx.address().do_send(SetState {
                    state: State::Planning,
                });
                ctx.address().do_send(RunAutomatonCycle {});
            }
        }
    }
}

#[derive(Message)]
#[rtype(result = "()")]
struct SetActionPlan {
    plan: Vec<Arc<dyn ActionFn>>,
}

impl Handler<SetActionPlan> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: SetActionPlan, ctx: &mut Context<Self>) -> Self::Result {
        self.action_plan = msg.plan;

        if !self.action_plan.is_empty() {
            // If we have a plan, start acting
            ctx.address().do_send(SetState {
                state: State::Acting,
            });
        } else {
            // No plan, go back to waiting orders
            ctx.address().do_send(SetState {
                state: State::WaitingOrders,
            });
        }

        ctx.address().do_send(RunAutomatonCycle {});
    }
}

impl Handler<ContinueWithPlan> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: ContinueWithPlan, ctx: &mut Context<Self>) -> Self::Result {
        self.action_plan = msg.remaining_plan;
        ctx.address().do_send(RunAutomatonCycle {});
    }
}

impl Handler<NeedReplan> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, _msg: NeedReplan, ctx: &mut Context<Self>) -> Self::Result {
        ctx.address().do_send(SetState {
            state: State::Planning,
        });
        ctx.address().do_send(RunAutomatonCycle {});
    }
}

/// Controller for the actor-based automaton
pub struct ActorAutomatonController {
    automaton: Addr<AutomatonActor>,
}

impl ActorAutomatonController {
    /// Creates a new automaton controller with the given components
    pub fn new(
        name: &str,
        sensors: Vec<Arc<dyn SensorFn>>,
        actions: Vec<Arc<dyn ActionFn>>,
        planner: Arc<dyn PlannerFn>,
    ) -> Self {
        // Create all the actors
        let automaton_actor = AutomatonActor::new(name);
        let sensor_actor = SensorActor::new(sensors);
        let planner_actor = PlannerActor::new(planner, actions);
        let action_actor = ActionActor::new();

        // Start actors
        let automaton_addr = automaton_actor.start();
        let sensor_addr = sensor_actor.start();
        let planner_addr = planner_actor.start();
        let action_addr = action_actor.start();

        // Configure connections between actors
        // Note: In a production system, you would need to ensure these
        // messages are properly handled. This is simplified for the example.
        let sensor_addr_clone = sensor_addr.clone();
        let automaton_addr_clone = automaton_addr.clone();

        actix::spawn(async move {
            automaton_addr_clone
                .send(SetSensorActor {
                    addr: sensor_addr_clone.clone(),
                })
                .await
                .ok();
            automaton_addr_clone
                .send(SetPlannerActor { addr: planner_addr })
                .await
                .ok();
            automaton_addr_clone
                .send(SetActionActor { addr: action_addr })
                .await
                .ok();
            sensor_addr_clone
                .send(SetAutomatonActor {
                    addr: automaton_addr_clone.clone(),
                })
                .await
                .ok();
        });

        ActorAutomatonController {
            automaton: automaton_addr,
        }
    }

    /// Returns a reference to the automaton
    pub fn automaton(&self) -> &Addr<AutomatonActor> {
        &self.automaton
    }

    /// Sets the goal for the automaton
    pub async fn set_goal(&self, goal: HashMap<String, Fact>) {
        let _ = self.automaton.send(SetGoalRequest { goal }).await;
    }

    /// Gets the current world state of the automaton
    pub async fn world_state(&self) -> HashMap<String, Fact> {
        match self.automaton.send(GetWorldState {}).await {
            Ok(WorldStateResult(state)) => state,
            Err(_) => HashMap::new(),
        }
    }

    /// Gets the current goal of the automaton
    pub async fn goal(&self) -> HashMap<String, Fact> {
        match self.automaton.send(GetGoal {}).await {
            Ok(GoalResult(goal)) => goal,
            Err(_) => HashMap::new(),
        }
    }

    /// Starts the automaton
    pub async fn start(&self) {
        let _ = self.automaton.send(StartRequest {}).await;
    }

    /// Stops the automaton
    pub async fn stop(&self) {
        let _ = self.automaton.send(StopRequest {}).await;
    }
}

#[derive(Message)]
#[rtype(result = "()")]
struct SetSensorActor {
    addr: Addr<SensorActor>,
}

#[derive(Message)]
#[rtype(result = "()")]
struct SetPlannerActor {
    addr: Addr<PlannerActor>,
}

#[derive(Message)]
#[rtype(result = "()")]
struct SetActionActor {
    addr: Addr<ActionActor>,
}

#[derive(Message)]
#[rtype(result = "()")]
struct SetAutomatonActor {
    addr: Addr<AutomatonActor>,
}

#[derive(Message)]
#[rtype(result = "GoalResult")]
struct GetGoal;

impl Handler<SetSensorActor> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: SetSensorActor, _ctx: &mut Context<Self>) -> Self::Result {
        self.sensor_actor = Some(msg.addr);
    }
}

impl Handler<SetPlannerActor> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: SetPlannerActor, _ctx: &mut Context<Self>) -> Self::Result {
        self.planner_actor = Some(msg.addr);
    }
}

impl Handler<SetActionActor> for AutomatonActor {
    type Result = ();

    fn handle(&mut self, msg: SetActionActor, _ctx: &mut Context<Self>) -> Self::Result {
        self.action_actor = Some(msg.addr);
    }
}

impl Handler<GetGoal> for AutomatonActor {
    type Result = GoalResult;

    fn handle(&mut self, _msg: GetGoal, _ctx: &mut Context<Self>) -> Self::Result {
        GoalResult(self.goal.clone())
    }
}

impl Handler<SetAutomatonActor> for SensorActor {
    type Result = ();

    fn handle(&mut self, msg: SetAutomatonActor, _ctx: &mut Context<Self>) -> Self::Result {
        self.automaton_addr = Some(msg.addr);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use actix::System;
    use std::sync::Arc;
    use tokio;

    struct TestSensor;

    #[async_trait]
    impl SensorFn for TestSensor {
        async fn exec(&self, _world_state: &HashMap<String, Fact>) -> Vec<Fact> {
            vec![Fact::new("test", "value", "TestSensor")]
        }
    }

    struct TestAction;

    #[async_trait]
    impl ActionFn for TestAction {
        async fn exec(&self, _world_state: &HashMap<String, Fact>) -> bool {
            true
        }
    }

    struct TestPlanner;

    #[async_trait]
    impl PlannerFn for TestPlanner {
        async fn plan(
            &self,
            _world_state: &HashMap<String, Fact>,
            _goal: &HashMap<String, Fact>,
            available_actions: &[Arc<dyn ActionFn>],
        ) -> Vec<Arc<dyn ActionFn>> {
            available_actions.to_vec()
        }
    }

    #[test]
    fn test_actor_automaton_basic_cycle() {
        // Create a test system for running our actors
        let system = System::new();

        system.block_on(async {
            let sensors = vec![Arc::new(TestSensor) as Arc<dyn SensorFn>];
            let actions = vec![Arc::new(TestAction) as Arc<dyn ActionFn>];
            let planner = Arc::new(TestPlanner) as Arc<dyn PlannerFn>;

            let controller =
                ActorAutomatonController::new("TestAutomaton", sensors, actions, planner);

            // Set a goal
            let mut goal = HashMap::new();
            goal.insert(
                "test_goal".to_string(),
                Fact::new("test_goal", "achieved", "TestGoal"),
            );
            controller.set_goal(goal).await;

            // Start the automaton
            controller.start().await;

            // Wait a bit for the automaton to cycle through its states
            tokio::time::sleep(Duration::from_millis(500)).await;

            // Stop the automaton
            controller.stop().await;

            // Signal the system to stop
            System::current().stop();
        });
    }
}
