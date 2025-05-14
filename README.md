# GOAP-RS

A Goal-Oriented Action Planning (GOAP) implementation in Rust. This library provides a flexible and efficient way to implement autonomous agents that can plan sequences of actions to achieve specific goals.

## Features

- Simple and intuitive API
- Efficient A* pathfinding algorithm
- Support for action costs and preconditions
- Flexible state representation
- Error handling with custom error types

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
goap-rs = "0.1.0"
```

### Example

Here's a simple example of how to use the library:

```rust
use goap_rs::{Action, State, Planner, Result};

fn main() -> Result<()> {
    // Create actions
    let mut gather_wood = Action::new("gather_wood", 1.0)?;
    gather_wood.preconditions.set("has_axe", true);
    gather_wood.effects.set("has_wood", true);

    let mut build_house = Action::new("build_house", 2.0)?;
    build_house.preconditions.set("has_wood", true);
    build_house.effects.set("has_house", true);

    // Create the planner with available actions (Default A* search)
    // For Dijkstra search algorithm use
    // let planner = Planner::with_search_algorithm(actions, Box::new(DijkstraSearch));
    let planner = Planner::new(vec![gather_wood, build_house]);

    // Define the current state
    let mut current_state = State::new();
    current_state.set("has_axe", true);
    current_state.set("has_wood", false);
    current_state.set("has_house", false);

    // Define the goal state
    let mut goal_state = State::new();
    goal_state.set("has_house", true);

    // Find a plan
    let plan = planner.plan(&current_state, &goal_state)?;

    // Execute the plan
    for action in plan {
        println!("Executing action: {}", action.name);
    }

    Ok(())
}
```

If you want to visualize the plan you can use the `GoapVizualizer::new()`:

```rust
let visualizer = GoapVisualizer::new();
visualizer.visualize_plan(&actions, &current_state, &goal_state, &plan, "output.dot")?;
```

## How it Works

GOAP (Goal-Oriented Action Planning) is an AI architecture that enables agents to plan sequences of actions to achieve specific goals. The system works by:

1. Defining actions with preconditions and effects
2. Specifying the current state of the world
3. Setting a goal state to achieve
4. Using A* pathfinding to find the optimal sequence of actions

The planner will find the most efficient path from the current state to the goal state, taking into account the cost of each action.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

