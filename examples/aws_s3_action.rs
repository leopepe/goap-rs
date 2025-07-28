#![allow(clippy::result_large_err)]

use aws_config::{meta::region::RegionProviderChain, BehaviorVersion};
use aws_sdk_s3::{types::BucketLocationConstraint, Client};
use goaprs::action::ActionResponse;
use goaprs::{Action, State};
use std::error::Error;
use std::sync::Arc;

// Shows your buckets, or those just in the region.
async fn list_buckets(
    client: &Client,
    region: BucketLocationConstraint,
) -> Result<Vec<(BucketLocationConstraint, String)>, Box<dyn Error>> {
    let mut buckets = client.list_buckets().into_paginator().send();

    let mut _num_buckets = 0;
    let mut result: Vec<(BucketLocationConstraint, String)> = Vec::new();
    let mut bucket_names: Vec<String> = Vec::new();

    while let Some(Ok(output)) = buckets.next().await {
        for bucket in output.buckets() {
            _num_buckets += 1;
            let r = client
                .get_bucket_location()
                .bucket(bucket.name().unwrap_or_default())
                .send()
                .await?;

            if r.location_constraint() == Some(&region) {
                let bucket_name = bucket.name().unwrap_or_default().to_string();
                result.push((
                    r.location_constraint().unwrap().clone(),
                    bucket_name.clone(),
                ));
                bucket_names.push(bucket_name);
            }
        }
    }

    println!("Found {} buckets in the specified region.", result.len());

    Ok(result)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Set up AWS client
    let region_provider = RegionProviderChain::default_provider().or_else("us-east-1");
    let config = aws_config::defaults(BehaviorVersion::latest())
        .region(region_provider)
        .load()
        .await;
    let client = Client::new(&config);
    let client_arc = Arc::new(client);

    // Create initial state
    let mut state = State::new();
    state.set("aws_credentials_loaded", "true");
    state.set("region", "true");

    // Create preconditions
    let mut preconditions = State::new();
    preconditions.set("aws_credentials_loaded", "true");

    // Create effects
    let mut effects = State::new();
    effects.set("buckets_listed", "true");

    // Create the action
    let mut list_buckets_action = Action::new("list_buckets", 0.1).unwrap();
    list_buckets_action.preconditions = preconditions;
    list_buckets_action.effects = effects;

    // Define a custom execution function for our action
    let client_for_closure = Arc::clone(&client_arc);

    // Create the action
    let exec_fn = Box::new(move |_state| {
        let client = client_for_closure.clone();
        Box::pin(async move {
            // Use us-east-1 region
            let region = BucketLocationConstraint::from("us-east-1");
            match list_buckets(&client, region).await {
                Ok(buckets) => {
                    let bucket_list: Vec<String> =
                        buckets.iter().map(|(_, name)| name.clone()).collect();

                    // Create a successful response with correct parameter order
                    ActionResponse::new(
                        format!(
                            "Successfully listed {} buckets: {:?}",
                            buckets.len(),
                            bucket_list
                        ),
                        String::new(),
                        0,
                    )
                }
                Err(e) => {
                    // Create an error response with correct parameter order
                    ActionResponse::new(String::new(), format!("Failed to list buckets: {}", e), 1)
                }
            }
        })
    });

    // Check if we can perform the action
    if list_buckets_action.can_perform(&state) {
        println!("Can perform the list_buckets action");

        // Execute the action
        let response = exec_fn(&state).await;

        // Print the response
        println!("Action execution response: {}", response);

        // Apply effects to the state
        list_buckets_action.apply_effects(&mut state);

        // Verify the state was updated
        assert_eq!(state.get("buckets_listed"), Some("true"));
    } else {
        println!("Cannot perform the list_buckets action, preconditions not met");
    }

    Ok(())
}
