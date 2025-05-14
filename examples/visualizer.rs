use goap_rs::{Action, GoapVisualizer, Planner, Result, State};
fn main() -> Result<()> {
    // Create a complex set of actions with multiple preconditions and effects
    let mut actions = Vec::new();

    // Action 1: Gather Resources
    let mut gather_resources = Action::new("gather_resources", 1.0).unwrap();
    gather_resources.preconditions.set("has_tools", true);
    gather_resources.preconditions.set("has_energy", true);
    gather_resources.preconditions.set("has_permission", true);
    gather_resources.preconditions.set("weather_good", true);
    gather_resources.preconditions.set("area_safe", true);
    gather_resources.effects.set("has_wood", true);
    gather_resources.effects.set("has_stone", true);
    gather_resources.effects.set("has_metal", true);
    gather_resources.effects.set("has_weather_protection", true);
    actions.push(gather_resources);

    // Action 2: Process Materials
    let mut process_materials = Action::new("process_materials", 2.0).unwrap();
    process_materials.preconditions.set("has_wood", true);
    process_materials.preconditions.set("has_stone", true);
    process_materials.preconditions.set("has_metal", true);
    process_materials.preconditions.set("has_workshop", true);
    process_materials.preconditions.set("has_skills", true);
    process_materials.effects.set("has_processed_wood", true);
    process_materials.effects.set("has_processed_stone", true);
    process_materials.effects.set("has_processed_metal", true);
    actions.push(process_materials);

    // Action 3: Build Foundation
    let mut build_foundation = Action::new("build_foundation", 3.0).unwrap();
    build_foundation
        .preconditions
        .set("has_processed_stone", true);
    build_foundation
        .preconditions
        .set("has_processed_metal", true);
    build_foundation.preconditions.set("has_blueprint", true);
    build_foundation.preconditions.set("has_equipment", true);
    build_foundation.preconditions.set("has_workers", true);
    build_foundation.effects.set("has_foundation", true);
    build_foundation.effects.set("has_structure_started", true);
    actions.push(build_foundation);

    // Action 4: Build Walls
    let mut build_walls = Action::new("build_walls", 4.0).unwrap();
    build_walls.preconditions.set("has_foundation", true);
    build_walls.preconditions.set("has_processed_wood", true);
    build_walls.preconditions.set("has_processed_stone", true);
    build_walls.preconditions.set("has_structure_started", true);
    build_walls
        .preconditions
        .set("has_weather_protection", true);
    build_walls.effects.set("has_walls", true);
    build_walls.effects.set("has_basic_structure", true);
    actions.push(build_walls);

    // Action 5: Install Roof
    let mut install_roof = Action::new("install_roof", 5.0).unwrap();
    install_roof.preconditions.set("has_basic_structure", true);
    install_roof.preconditions.set("has_processed_wood", true);
    install_roof.preconditions.set("has_processed_metal", true);
    install_roof
        .preconditions
        .set("has_roofing_materials", true);
    install_roof.preconditions.set("has_safety_equipment", true);
    install_roof.effects.set("has_roof", true);
    actions.push(install_roof);

    // Action 6: Install Windows
    let mut install_windows = Action::new("install_windows", 2.0).unwrap();
    install_windows.preconditions.set("has_walls", true);
    install_windows.preconditions.set("has_roof", true);
    install_windows.preconditions.set("has_windows", true);
    install_windows.preconditions.set("has_tools", true);
    install_windows
        .preconditions
        .set("has_weather_protection", true);
    install_windows.effects.set("has_installed_windows", true);
    install_windows.effects.set("has_natural_light", true);
    actions.push(install_windows);

    // Action 7: Install Doors
    let mut install_doors = Action::new("install_doors", 2.0).unwrap();
    install_doors.preconditions.set("has_walls", true);
    install_doors.preconditions.set("has_roof", true);
    install_doors.preconditions.set("has_doors", true);
    install_doors.preconditions.set("has_tools", true);
    install_doors
        .preconditions
        .set("has_weather_protection", true);
    install_doors.effects.set("has_installed_doors", true);
    install_doors.effects.set("has_access_points", true);
    actions.push(install_doors);

    // Action 8: Install Utilities
    let mut install_utilities = Action::new("install_utilities", 6.0).unwrap();
    install_utilities
        .preconditions
        .set("has_basic_structure", true);
    install_utilities.preconditions.set("has_roof", true);
    install_utilities
        .preconditions
        .set("has_utility_plans", true);
    install_utilities
        .preconditions
        .set("has_utility_materials", true);
    install_utilities
        .preconditions
        .set("has_certified_electrician", true);
    install_utilities.effects.set("has_electricity", true);
    install_utilities.effects.set("has_plumbing", true);
    install_utilities.effects.set("has_utilities", true);
    actions.push(install_utilities);

    // Action 9: Interior Finishing
    let mut interior_finishing = Action::new("interior_finishing", 4.0).unwrap();
    interior_finishing.preconditions.set("has_utilities", true);
    interior_finishing
        .preconditions
        .set("has_installed_windows", true);
    interior_finishing
        .preconditions
        .set("has_installed_doors", true);
    interior_finishing
        .preconditions
        .set("has_finishing_materials", true);
    interior_finishing
        .preconditions
        .set("has_interior_design_plan", true);
    interior_finishing
        .effects
        .set("has_finished_interior", true);
    interior_finishing.effects.set("has_livable_space", true);
    actions.push(interior_finishing);

    // Action 10: Final Inspection
    let mut final_inspection = Action::new("final_inspection", 1.0).unwrap();
    final_inspection
        .preconditions
        .set("has_finished_interior", true);
    final_inspection.preconditions.set("has_utilities", true);
    final_inspection
        .preconditions
        .set("has_installed_windows", true);
    final_inspection
        .preconditions
        .set("has_installed_doors", true);
    final_inspection
        .preconditions
        .set("has_safety_checks", true);
    final_inspection.effects.set("has_passed_inspection", true);
    final_inspection.effects.set("has_completed_house", true);
    actions.push(final_inspection);

    // Create the planner with all actions
    let planner = Planner::new(actions.clone());

    // Define the initial state with all required starting conditions
    let mut current_state = State::new();
    current_state.set("has_tools", true);
    current_state.set("has_energy", true);
    current_state.set("has_permission", true);
    current_state.set("weather_good", true);
    current_state.set("area_safe", true);
    current_state.set("has_workshop", true);
    current_state.set("has_skills", true);
    current_state.set("has_blueprint", true);
    current_state.set("has_equipment", true);
    current_state.set("has_workers", true);
    current_state.set("has_roofing_materials", true);
    current_state.set("has_safety_equipment", true);
    current_state.set("has_windows", true);
    current_state.set("has_doors", true);
    current_state.set("has_utility_plans", true);
    current_state.set("has_utility_materials", true);
    current_state.set("has_certified_electrician", true);
    current_state.set("has_finishing_materials", true);
    current_state.set("has_interior_design_plan", true);
    current_state.set("has_safety_checks", true);

    // Define the goal state
    let mut goal_state = State::new();
    goal_state.set("has_completed_house", true);

    // Find a plan
    let plan = planner.plan(&current_state, &goal_state).unwrap();

    // Visualize the plan
    let visualizer = GoapVisualizer::new();
    visualizer.visualize_plan(
        &actions,
        &current_state,
        &goal_state,
        &plan,
        "./complex_planning_visualization.dot",
    )?;

    Ok(())
}
