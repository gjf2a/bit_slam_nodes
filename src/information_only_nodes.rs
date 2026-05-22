use arg_vals::ArgDocs;
use r2r::irobot_create_msgs::msg::IrIntensityVector;

use crate::{PERIOD, node_struct::{NodeSpec, RunnableNode}, robot_name, util::ros2_node_name};

pub struct ShowIrNode {
    docs: ArgDocs
}

impl Default for ShowIrNode {
    fn default() -> Self {
        Self {
            docs: ArgDocs::new(
                "show_ir_node",
                &vec![("--robot", "str", "")],
            ),
        }
    }
}

impl RunnableNode for ShowIrNode {
    fn arg_docs(&self) -> &ArgDocs {
        &self.docs
    }

    fn arg_docs_mut(&mut self) -> &mut ArgDocs {
        &mut self.docs
    }

    fn publishing_topics(&self, _: &arg_vals::ArgVals) -> anyhow::Result<Vec<String>> {
        Ok(vec![])
    }

    fn subscribing_topics(&self, args: &arg_vals::ArgVals) -> anyhow::Result<Vec<String>> {
        let robot = robot_name!(args);
        Ok(vec![format!("{robot}/ir_intensity")])
    }

    fn spec(&self, args: &arg_vals::ArgVals) -> anyhow::Result<crate::node_struct::NodeSpec> {
        let robot = robot_name!(args);
        let mut spec = NodeSpec::new(&ros2_node_name(robot, "show_ir_node"), PERIOD)?;
        let subs = self.subscribing_topics(args)?;
        spec.subscribe(&subs[0], move |ir: IrIntensityVector, _| {
            for value in ir.readings.iter() {
                print!("{} ", value.value);
            }
            println!();
        })?;
        Ok(spec)
    }
}