//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

mod estimator;
mod map;
mod scan_map;
mod static_map;

use crate::estimator::Estimator;
use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::OccupancyGrid;
use visualization_msgs::msg::MarkerArray;
use std::ops::Deref;
use rclrs::{
    Node, 
    Subscription, 
    Publisher, 
    Executor, 
    RclrsError, 
    Context, 
    SpinOptions, 
    CreateBasicExecutor, 
    RclrsErrorFilter
};

struct FlowEstimatorNode {
    node: Arc<Node>,
    _sub_scan: Mutex<Option<Arc<Subscription<LaserScan>>>>,
    data: Arc<Mutex<Option<LaserScan>>>,
    scan_map: Arc<Publisher<OccupancyGrid>>,
    static_obstacle_map: Arc<Publisher<OccupancyGrid>>,
    obstacle_motion: Arc<Publisher<MarkerArray>>,
}

impl FlowEstimatorNode {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("flow_estimator")?;
        
        let scan_map = node.create_publisher::<OccupancyGrid>("scan_map")?;
        let static_obstacle_map = node.create_publisher::<OccupancyGrid>("static_obstacle_map")?;
        let obstacle_motion = node.create_publisher::<MarkerArray>("estimation_array")?;
        
        let data = Arc::new(Mutex::new(None));
        
        let flow_estimator_node = Arc::new(FlowEstimatorNode {
            node: node.into(),
            _sub_scan: None.into(),
            data,
            scan_map: scan_map.into(),
            static_obstacle_map: static_obstacle_map.into(),
            obstacle_motion: obstacle_motion.into(),
        });
        
        let data_cb = Arc::clone(&flow_estimator_node.data);
        let subscription = flow_estimator_node.node
            .create_subscription::<LaserScan, _>("scan", move |msg: LaserScan| { 
                *data_cb.lock().unwrap() = Some(msg); 
            })?;
        
        *flow_estimator_node._sub_scan.lock().unwrap() = Some(subscription.into());
        
        Ok(flow_estimator_node)
    }
    
    fn publish_scan_map(&self, buffer: &mut Vec<OccupancyGrid>) -> Result<(), RclrsError> {
        let scan = match self.data.lock().unwrap().deref() {
            Some(s) => s.clone(),
            None => {
                eprintln!("waiting scan");
                return Ok(());
            },
        };
        
        if let Some(last_map) = buffer.last() {
            if last_map.info.map_load_time == scan.header.stamp {
                return Ok(());
            }
        }
        
        let map = scan_map::generate(120, 120, 0.1, &scan);
        buffer.push(map.clone());
        self.scan_map.publish(&map)?;
        
        if buffer.len() > 100 {
            buffer.remove(0);
        }
        
        Ok(())
    }
    
    fn publish_static_obstacle_map(&self, map: &OccupancyGrid) -> Result<(), RclrsError> {
        self.static_obstacle_map.publish(map)?;
        Ok(())
    }
    
    fn publish_obstacle_motion(&self, map: &MarkerArray) -> Result<(), RclrsError> {
        self.obstacle_motion.publish(map)?;
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let republisher = FlowEstimatorNode::new(&executor)?;
    
    let republisher_other_thread = Arc::clone(&republisher);
    let mut map_buffer = vec![];
    
    std::thread::spawn(move || -> Result<(), RclrsError> {
        let mut estimator = Estimator::new();
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(200));
            
            republisher_other_thread.publish_scan_map(&mut map_buffer)?;
            
            if let Some(static_map) = static_map::generate(&mut map_buffer) {
                republisher_other_thread.publish_static_obstacle_map(&static_map)?;
                
                match estimator.generate(&mut map_buffer, &static_map) {
                    Ok(Some(estimation)) => 
                        republisher_other_thread.publish_obstacle_motion(&estimation)?,
                    Ok(None) => eprintln!("waiting ..."),
                    Err(e)   => eprintln!("{:?}", &e),
                }
            }
        }
    });
    
    executor.spin(SpinOptions::default()).first_error()
}
