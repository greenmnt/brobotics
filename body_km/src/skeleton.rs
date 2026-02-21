use std::collections::HashMap;
use std::fs;

use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use serde::Deserialize;

#[derive(Debug, Clone, Copy)]
pub struct AxisRom {
    pub min: f64,
    pub max: f64,
}

impl AxisRom {
    pub fn clamp(&self, value: f64) -> f64 {
        value.clamp(self.min, self.max)
    }
}

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub parent: Option<usize>,
    pub children: Vec<usize>,
    pub position: Vector3<f64>,
    pub orientation: Matrix3<f64>,
    pub dof: [Option<AxisRom>; 3],       // [x, y, z]; None = locked
    pub max_velocity: [Option<f64>; 3],  // rad/s per axis; None = no limit
}

#[derive(Debug)]
pub struct Skeleton {
    pub joints: Vec<Joint>,
}

// --- JSON deserialization types ---

#[derive(Deserialize)]
struct JsonSkeleton {
    joints: Vec<JsonJoint>,
}

#[derive(Deserialize)]
struct JsonJoint {
    name: String,
    parent: Option<String>,
    position: [f64; 3],
    orientation: [[f64; 3]; 3],
    dof: HashMap<String, [f64; 2]>,
    #[serde(default)]
    velocity_limit: HashMap<String, f64>,
}

impl Skeleton {
    pub fn from_json(path: &str) -> Result<Skeleton, String> {
        let data = fs::read_to_string(path).map_err(|e| format!("Failed to read {path}: {e}"))?;
        let raw: JsonSkeleton =
            serde_json::from_str(&data).map_err(|e| format!("Failed to parse JSON: {e}"))?;

        // Build name -> index map
        let name_to_idx: HashMap<&str, usize> = raw
            .joints
            .iter()
            .enumerate()
            .map(|(i, j)| (j.name.as_str(), i))
            .collect();

        // Convert to Joint structs
        let mut joints: Vec<Joint> = raw
            .joints
            .iter()
            .map(|j| {
                let parent = j.parent.as_ref().map(|p| {
                    *name_to_idx
                        .get(p.as_str())
                        .unwrap_or_else(|| panic!("Unknown parent joint: {p}"))
                });

                let position = Vector3::new(j.position[0], j.position[1], j.position[2]);

                let orientation = Matrix3::new(
                    j.orientation[0][0],
                    j.orientation[0][1],
                    j.orientation[0][2],
                    j.orientation[1][0],
                    j.orientation[1][1],
                    j.orientation[1][2],
                    j.orientation[2][0],
                    j.orientation[2][1],
                    j.orientation[2][2],
                );

                let parse_axis = |key: &str| -> Option<AxisRom> {
                    j.dof
                        .get(key)
                        .map(|&[min, max]| AxisRom { min, max })
                };

                let dof = [parse_axis("x"), parse_axis("y"), parse_axis("z")];

                let max_velocity = [
                    j.velocity_limit.get("x").copied(),
                    j.velocity_limit.get("y").copied(),
                    j.velocity_limit.get("z").copied(),
                ];

                Joint {
                    name: j.name.clone(),
                    parent,
                    children: Vec::new(),
                    position,
                    orientation,
                    dof,
                    max_velocity,
                }
            })
            .collect();

        // Populate children lists
        for i in 0..joints.len() {
            if let Some(parent_idx) = joints[i].parent {
                let child_idx = i;
                joints[parent_idx].children.push(child_idx);
            }
        }

        Ok(Skeleton { joints })
    }

    pub fn find_joint(&self, name: &str) -> Option<usize> {
        self.joints.iter().position(|j| j.name == name)
    }

    /// Clamp a rotation vector (x, y, z) to the joint's ROM limits.
    /// Locked axes are forced to 0.0.
    pub fn clamp_rotation(&self, joint_idx: usize, rotation: Vector3<f64>) -> Vector3<f64> {
        let joint = &self.joints[joint_idx];
        Vector3::new(
            match &joint.dof[0] {
                Some(rom) => rom.clamp(rotation.x),
                None => 0.0,
            },
            match &joint.dof[1] {
                Some(rom) => rom.clamp(rotation.y),
                None => 0.0,
            },
            match &joint.dof[2] {
                Some(rom) => rom.clamp(rotation.z),
                None => 0.0,
            },
        )
    }

    /// Clamp a new rotation based on velocity limits and then ROM limits.
    ///
    /// Given the previous rotation, the new sensor reading, and the time step,
    /// limits the per-axis angular velocity to `max_velocity` (rad/s) and then
    /// clamps the result to the joint's ROM bounds. Locked axes are forced to 0.0.
    pub fn clamp_velocity(
        &self,
        joint_idx: usize,
        prev: Vector3<f64>,
        new: Vector3<f64>,
        dt: f64,
    ) -> Vector3<f64> {
        let joint = &self.joints[joint_idx];

        let clamp_axis =
            |axis: usize, prev_val: f64, new_val: f64| -> f64 {
                match &joint.dof[axis] {
                    Some(rom) => {
                        let val = match joint.max_velocity[axis] {
                            Some(max_vel) => {
                                let max_delta = max_vel * dt;
                                let delta = (new_val - prev_val).clamp(-max_delta, max_delta);
                                prev_val + delta
                            }
                            None => new_val,
                        };
                        rom.clamp(val)
                    }
                    None => 0.0,
                }
            };

        Vector3::new(
            clamp_axis(0, prev.x, new.x),
            clamp_axis(1, prev.y, new.y),
            clamp_axis(2, prev.z, new.z),
        )
    }

    /// Compute world-space positions via forward kinematics.
    ///
    /// Takes one `UnitQuaternion` per joint (local rotation relative to rest pose).
    /// Use `UnitQuaternion::identity()` for joints without sensor input.
    /// Returns one `Point3` per joint in world coordinates.
    pub fn forward_kinematics(&self, local_rotations: &[UnitQuaternion<f64>]) -> Vec<Point3<f64>> {
        self.forward_kinematics_full(local_rotations).0
    }

    /// Compute world-space positions and world rotations via forward kinematics.
    ///
    /// Same as `forward_kinematics` but also returns the accumulated world
    /// rotation per joint (useful for drawing orientation axes and paddle bones).
    pub fn forward_kinematics_full(
        &self,
        local_rotations: &[UnitQuaternion<f64>],
    ) -> (Vec<Point3<f64>>, Vec<UnitQuaternion<f64>>) {
        assert_eq!(local_rotations.len(), self.joints.len());

        let mut positions = vec![Point3::origin(); self.joints.len()];
        let mut world_rots = vec![UnitQuaternion::identity(); self.joints.len()];

        for (i, joint) in self.joints.iter().enumerate() {
            match joint.parent {
                None => {
                    positions[i] =
                        Point3::new(joint.position.x, joint.position.y, joint.position.z);
                    world_rots[i] = local_rotations[i];
                }
                Some(pi) => {
                    let offset = joint.position - self.joints[pi].position;
                    positions[i] = positions[pi] + world_rots[pi] * offset;
                    world_rots[i] = world_rots[pi] * local_rotations[i];
                }
            }
        }

        (positions, world_rots)
    }

    /// Print the skeleton as an indented tree.
    pub fn print_tree(&self) {
        println!("Loaded skeleton: {} joints", self.joints.len());
        for (i, joint) in self.joints.iter().enumerate() {
            if joint.parent.is_none() {
                self.print_subtree(i, 1);
            }
        }
    }

    fn print_subtree(&self, idx: usize, depth: usize) {
        let joint = &self.joints[idx];
        let indent = "  ".repeat(depth);
        let rom_str = Self::format_rom(&joint.dof);
        let vel_str = Self::format_vel(&joint.max_velocity);

        if joint.parent.is_none() {
            println!("{indent}{} (root){rom_str}{vel_str}", joint.name);
        } else {
            println!("{indent}{}{rom_str}{vel_str}", joint.name);
        }

        for &child in &joint.children {
            self.print_subtree(child, depth + 1);
        }
    }

    fn format_rom(dof: &[Option<AxisRom>; 3]) -> String {
        let axes = ["x", "y", "z"];
        let parts: Vec<String> = dof
            .iter()
            .zip(axes.iter())
            .filter_map(|(rom, axis)| {
                rom.as_ref()
                    .map(|r| format!("{axis}: {:.2}..{:.2}", r.min, r.max))
            })
            .collect();

        if parts.is_empty() {
            " (locked)".to_string()
        } else {
            format!(" [{}]", parts.join(", "))
        }
    }

    fn format_vel(max_velocity: &[Option<f64>; 3]) -> String {
        let axes = ["x", "y", "z"];
        let parts: Vec<String> = max_velocity
            .iter()
            .zip(axes.iter())
            .filter_map(|(vel, axis)| vel.map(|v| format!("{axis}: {v:.0}")))
            .collect();

        if parts.is_empty() {
            String::new()
        } else {
            format!(" vel({})", parts.join(", "))
        }
    }
}
