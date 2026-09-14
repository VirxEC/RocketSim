use glam::{Affine3A, Vec3A};

use super::{NUM_WHEELS, raycaster::VehicleRaycasterResult};
use crate::{
    bullet::dynamics::{
        constraint_solver::contact_constraint::{
            resolve_single_bilateral_fake_ground, resolve_single_collision,
        },
        rigid_body::{Impulse, RigidBody},
    },
    consts::{BT_TO_UU, UU_TO_BT, bullet_vehicle, curves},
};

pub struct RaycastInfo {
    pub contact_normal: Vec3A,
    pub contact_point: Vec3A,
    pub ground_body_idx: usize,
    pub suspension_length: f32,
    pub impulse: Vec3A,
    pub ground_stick: Vec3A,
    pub is_in_contact_with_world: bool,
    pub clipped_inv_contact_dot_suspension: f32,
    pub suspension_relative_vel: f32,
}

#[derive(Clone, Copy)]
pub struct FrictionCurveInput {
    pub chassis_translation: Vec3A,
    pub contact_normal: Vec3A,
    pub handbrake_val: f32,
    pub real_throttle: f32,
    pub three_wheels: bool,
    pub is_dynamic_hit: bool,
}

pub struct WheelInfo {
    pub raycast_info: Option<RaycastInfo>,
    pub hard_point: Vec3A,
    pub axle_dir: Vec3A,
    pub chassis_connection_point_cs: Vec3A,
    pub suspension_rest_length_1: f32,
    pub wheels_radius: f32,
    pub engine_force: f32,
    pub brake: f32,
    pub steer_angle: f32,
    pub vel_at_contact_point: Vec3A,
    pub lat_friction: f32,
    pub long_friction: f32,
    pub suspension_force_scale: f32,
    pub extra_pushback: f32,
    pub real_ray_length: f32,
}

impl WheelInfo {
    pub const DEFAULT: Self = Self {
        raycast_info: None,
        hard_point: Vec3A::ZERO,
        axle_dir: Vec3A::ZERO,
        chassis_connection_point_cs: Vec3A::ZERO,
        suspension_rest_length_1: 0.0,
        wheels_radius: 0.0,
        engine_force: 0.0,
        brake: 0.0,
        steer_angle: 0.0,
        vel_at_contact_point: Vec3A::ZERO,
        lat_friction: 1.0,
        long_friction: 1.0,
        suspension_force_scale: 1.0,
        extra_pushback: 0.0,
        real_ray_length: 0.0,
    };

    pub const fn set_params(
        &mut self,
        chassis_connection_cs: Vec3A,
        suspension_rest_length: f32,
        wheel_radius: f32,
        suspsension_force_scale: f32,
    ) {
        self.chassis_connection_point_cs = chassis_connection_cs;
        self.suspension_rest_length_1 = suspension_rest_length;
        self.wheels_radius = wheel_radius;
        self.suspension_force_scale = suspsension_force_scale;

        let suspension_travel = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;
        self.real_ray_length =
            self.suspension_rest_length_1 + suspension_travel + self.wheels_radius;
    }

    pub fn prepare_for_raycast(&mut self, chassis_trans: &Affine3A) -> (Vec3A, Vec3A) {
        self.hard_point = chassis_trans.transform_point3a(self.chassis_connection_point_cs);
        let target = self.hard_point - (chassis_trans.matrix3.z_axis * self.real_ray_length);

        (self.hard_point, target)
    }

    pub fn reset_wheel_suspension(&mut self) {
        self.extra_pushback = 0.0;
        self.raycast_info = None;
    }

    pub fn apply_ray_cast(
        &mut self,
        chassis: &RigidBody,
        chassis_trans: &Affine3A,
        front_axle_dir: Vec3A,
        ray_results: VehicleRaycasterResult,
        time_step: f32,
        front: bool,
    ) {
        let contact_point = ray_results.hit_point_in_world;
        let contact_normal = ray_results.hit_normal_in_world;
        let is_in_contact_with_world = ray_results.rigid_body.is_static_obj();

        // `front_axle_dir` is the cached steered axle shared by front
        // wheels with equal steer angles; wheels with a different angle
        // get their own axle from the caller.
        self.axle_dir = if front {
            front_axle_dir
        } else {
            chassis_trans.matrix3.y_axis
        };

        let up = chassis_trans.matrix3.z_axis;
        let wheel_trace_len_sq = (self.hard_point - contact_point).dot(up);

        let suspension_travel = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;
        let max_suspension_len = self.suspension_rest_length_1 + suspension_travel;

        let rel_pos = contact_point - chassis_trans.translation;
        self.vel_at_contact_point = chassis.get_vel_in_local_point(rel_pos);

        let proj_vel = contact_normal.dot(self.vel_at_contact_point);
        let denom = contact_normal.dot(up);

        let (suspension_relative_vel, clipped_inv_contact_dot_suspension) = if denom > 0.1 {
            let inv = 1.0 / denom;
            (proj_vel * inv, inv)
        } else {
            (0.0, 10.0)
        };

        let suspension_length = (wheel_trace_len_sq - self.wheels_radius).min(max_suspension_len);

        // The pushback resolve is a per-raycast transient: refresh it on
        // every contact tick so a wheel above the penetration threshold
        // reports 0 instead of keeping the previous tick's value.
        self.extra_pushback = 0.0;
        if is_in_contact_with_world {
            let ray_pushback_thresh = self.suspension_rest_length_1 + self.wheels_radius
                - bullet_vehicle::SUSPENSION_SUBTRACTION;
            if wheel_trace_len_sq < ray_pushback_thresh {
                let wheel_trace_dist_delta = wheel_trace_len_sq - ray_pushback_thresh;

                let collision_result = resolve_single_collision(
                    chassis,
                    ray_results.rigid_body,
                    ray_results.hit_point_in_world,
                    ray_results.hit_normal_in_world,
                    time_step,
                    wheel_trace_dist_delta,
                );

                self.extra_pushback = collision_result / NUM_WHEELS as f32;
            }
        }

        // Dynamic ray hits apply stick to the hit body.
        let ground_stick =
            if !ray_results.rigid_body.is_static_obj() && ray_results.rigid_body.inv_mass != 0.0 {
                -contact_normal
            } else {
                Vec3A::ZERO
            };

        self.raycast_info = Some(RaycastInfo {
            contact_normal,
            contact_point,
            ground_body_idx: ray_results.rigid_body_idx,
            suspension_length,
            impulse: Vec3A::ZERO,
            ground_stick,
            is_in_contact_with_world,
            clipped_inv_contact_dot_suspension,
            suspension_relative_vel,
        });
    }

    pub fn refresh_friction_curves(&mut self, chassis: &RigidBody, input: FrictionCurveInput) {
        let lat_dir = self.axle_dir;
        let long_dir = lat_dir.cross(input.contact_normal);
        let wheel_delta = self.hard_point - input.chassis_translation;
        let cross_vec = (chassis.ang_vel.cross(wheel_delta) + chassis.lin_vel) * BT_TO_UU;
        let base_friction = cross_vec.dot(lat_dir).abs();
        let friction_curve_input = if base_friction > 5.0 {
            base_friction / (cross_vec.dot(long_dir).abs() + base_friction)
        } else {
            0.0
        };

        let mut lat_friction = if input.three_wheels {
            curves::LAT_FRICTION_THREEWHEEL
        } else {
            curves::LAT_FRICTION
        }
        .get_output(friction_curve_input);

        let mut long_friction = 1.0;
        if input.handbrake_val != 0.0 {
            lat_friction *= 1.0
                + (curves::HANDBRAKE_LAT_FRICTION_FACTOR.get_output(friction_curve_input) - 1.0)
                    * input.handbrake_val;
            long_friction *= 1.0
                + (curves::HANDBRAKE_LONG_FRICTION_FACTOR.get_output(friction_curve_input) - 1.0)
                    * input.handbrake_val;
        }

        // Wheels on a dynamic hit body cannot use sticky ground: the lateral
        // bilateral resolves against a fixed body (target GetFakeBulletObj),
        // so a ball hit grips like static ground in the vehicle layer. RL
        // wheel records keep the 0.1 non-sticky scale on ball contacts even
        // with throttle (e.g. cb_reset_fling i67 lat 0.049/long 0.081 vs a
        // 0.49/0.81 unscaled vehicle computation), while flat ground
        // (normal.z = 1) scales by 1.0 and is unaffected. Gate on the
        // static-vs-dynamic hit classifier, not on recording or body names.
        if input.real_throttle == 0.0 || input.is_dynamic_hit {
            let non_sticky_scale =
                curves::NON_STICKY_FRICTION_FACTOR.get_output(input.contact_normal.z);
            lat_friction *= non_sticky_scale;
            long_friction *= non_sticky_scale;
        }

        self.lat_friction = lat_friction;
        self.long_friction = long_friction;
    }

    pub fn calc_friction_impulses(
        &mut self,
        chassis: &RigidBody,
        contact_normal: Vec3A,
        contact_point: Vec3A,
        time_step: f32,
        friction_scale: f32,
    ) -> Vec3A {
        let axle_dir = self.axle_dir.normalize_or_zero();

        let forward_dir = contact_normal.cross(axle_dir).normalize_or_zero();

        // Lateral friction resolves against a fixed ground body, never the
        // landed rigid body (target GetFakeBulletObj; upstream getFixedBody).
        let side_impulse = resolve_single_bilateral_fake_ground(chassis, contact_point, axle_dir);

        let rolling_friction = if self.engine_force == 0.0 {
            if self.brake == 0.0 {
                0.0
            } else {
                const ROLLING_FRICTION_SCALE: f32 = 113.73963;

                let contact_vel = self.vel_at_contact_point;
                let mut rel_vel = contact_vel.dot(forward_dir);

                if time_step > 1.0 / 80.0 {
                    let threshold = 0.8 - (1.0 / (time_step * 150.0));
                    if rel_vel.abs() < threshold {
                        rel_vel = 0.0;
                    }
                }

                (-rel_vel * ROLLING_FRICTION_SCALE).clamp(-self.brake, self.brake)
            }
        } else {
            -self.engine_force / friction_scale
        };

        let total_friction_force = forward_dir * rolling_friction * self.long_friction
            + axle_dir * side_impulse * self.lat_friction;
        total_friction_force * friction_scale
    }

    pub fn update_friction_impulse(
        &mut self,
        chassis: &RigidBody,
        time_step: f32,
        friction_scale: f32,
    ) {
        let Some(raycast_info) = self.raycast_info.as_ref() else {
            return;
        };

        let contact_normal = raycast_info.contact_normal;
        let contact_point = raycast_info.contact_point;
        let impulse = self.calc_friction_impulses(
            chassis,
            contact_normal,
            contact_point,
            time_step,
            friction_scale,
        );

        if let Some(raycast_info) = self.raycast_info.as_mut() {
            raycast_info.impulse = impulse;
        }
    }

    pub fn update_suspension(
        &mut self,
        cb: &mut RigidBody,
        chassis_translation: Vec3A,
        delta_time: f32,
    ) {
        let Some(raycast_info) = self.raycast_info.as_ref() else {
            return;
        };

        let force = (self.suspension_rest_length_1 - raycast_info.suspension_length)
            * bullet_vehicle::SUSPENSION_STIFFNESS
            * raycast_info.clipped_inv_contact_dot_suspension;

        let damping_vel_scale = if raycast_info.suspension_relative_vel < 0.0 {
            bullet_vehicle::WHEELS_DAMPING_COMPRESSION
        } else {
            bullet_vehicle::WHEELS_DAMPING_RELAXATION
        };

        let mut wheels_suspension_force =
            force - (damping_vel_scale * raycast_info.suspension_relative_vel);
        wheels_suspension_force *= self.suspension_force_scale;
        if wheels_suspension_force <= 0.0 {
            return;
        }
        let base_force_scale = wheels_suspension_force * delta_time + self.extra_pushback;
        let contact_point_offset = raycast_info.contact_point - chassis_translation;

        let force = raycast_info.contact_normal * base_force_scale;
        cb.add_impulse(
            Some("WheelsSuspension"),
            Impulse::LinearRelPos(force, contact_point_offset),
            true,
            false,
        );
    }

    pub fn apply_friction_impulses(
        &self,
        cb: &mut RigidBody,
        chassis_trans: &Affine3A,
        time_step: f32,
    ) {
        let Some(raycast_info) = self.raycast_info.as_ref() else {
            return;
        };

        let wheel_contact_offset = raycast_info.contact_point - chassis_trans.translation;
        let contact_up_dot = chassis_trans.matrix3.z_axis.dot(wheel_contact_offset);
        let wheel_rel_pos = wheel_contact_offset - chassis_trans.matrix3.z_axis * contact_up_dot;
        cb.add_impulse(
            Some("WheelsFriction"),
            Impulse::LinearRelPos(raycast_info.impulse * time_step, wheel_rel_pos),
            true,
            false,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bullet::{
        collision::shapes::{
            collision_shape::CollisionShapes, static_plane_shape::StaticPlaneShape,
        },
        dynamics::rigid_body::RigidBodyConstructionInfo,
    };

    // Target I23 negates the denominator before the compare.
    // A negated value below -0.1 selects the divide path.
    // The same condition is a true denominator above +0.1.
    // These tests lock that polarity mapping.
    fn make_bodies(lin_vel: Vec3A) -> (RigidBody, RigidBody) {
        let chassis_shape =
            CollisionShapes::StaticPlane(StaticPlaneShape::new(Affine3A::IDENTITY, Vec3A::Z));
        let mut chassis = RigidBody::new(RigidBodyConstructionInfo::new(180.0, chassis_shape));
        chassis.lin_vel = lin_vel;

        let ground_shape =
            CollisionShapes::StaticPlane(StaticPlaneShape::new(Affine3A::IDENTITY, Vec3A::Z));
        let ground = RigidBody::new(RigidBodyConstructionInfo::new(0.0, ground_shape));
        (chassis, ground)
    }

    fn make_wheel() -> WheelInfo {
        let mut wheel = WheelInfo::DEFAULT;
        wheel.set_params(Vec3A::ZERO, 1.0, 0.5, 1.0);
        wheel
    }

    fn cast_wheel(
        wheel: &mut WheelInfo,
        chassis: &RigidBody,
        ground: &RigidBody,
        normal: Vec3A,
        chassis_trans: &Affine3A,
    ) {
        // Put the contact far below the hard point.
        // This keeps the trace above the pushback gate.
        let (hard_point, _) = wheel.prepare_for_raycast(chassis_trans);
        let contact_point = hard_point - Vec3A::Z * 2.0;
        wheel.apply_ray_cast(
            chassis,
            chassis_trans,
            Vec3A::Y,
            VehicleRaycasterResult {
                hit_point_in_world: contact_point,
                hit_normal_in_world: normal,
                rigid_body_idx: 0,
                rigid_body: ground,
            },
            1.0 / 120.0,
            false,
        );
    }

    #[test]
    fn flat_ground_divides_by_denominator() {
        // Flat ground gives denom 1.0.
        // Expect projection scaled by 1.0.
        let (chassis, ground) = make_bodies(Vec3A::new(0.0, 0.0, -5.0));
        let mut wheel = make_wheel();
        let trans = Affine3A::IDENTITY;
        cast_wheel(&mut wheel, &chassis, &ground, Vec3A::Z, &trans);
        let info = wheel.raycast_info.as_ref().unwrap();
        assert_eq!(info.clipped_inv_contact_dot_suspension, 1.0);
        assert_eq!(info.suspension_relative_vel, -5.0);
    }

    #[test]
    fn threshold_boundary_uses_fallback() {
        // Denom 0.1 does not pass a strict greater-than test.
        // Expect the 10.0 fallback pair.
        let (chassis, ground) = make_bodies(Vec3A::new(0.0, 0.0, -5.0));
        let mut wheel = make_wheel();
        let trans = Affine3A::IDENTITY;
        let y = (1.0f32 - 0.1 * 0.1).sqrt();
        cast_wheel(
            &mut wheel,
            &chassis,
            &ground,
            Vec3A::new(0.0, y, 0.1),
            &trans,
        );
        let info = wheel.raycast_info.as_ref().unwrap();
        assert_eq!(info.suspension_relative_vel, 0.0);
        assert_eq!(info.clipped_inv_contact_dot_suspension, 10.0);
    }

    #[test]
    fn wall_normal_uses_fallback() {
        // A wall normal gives denom 0.0.
        // Expect the 10.0 fallback pair.
        let (chassis, ground) = make_bodies(Vec3A::new(0.0, 0.0, -5.0));
        let mut wheel = make_wheel();
        let trans = Affine3A::IDENTITY;
        cast_wheel(&mut wheel, &chassis, &ground, Vec3A::Y, &trans);
        let info = wheel.raycast_info.as_ref().unwrap();
        assert_eq!(info.suspension_relative_vel, 0.0);
        assert_eq!(info.clipped_inv_contact_dot_suspension, 10.0);
    }

    #[test]
    fn just_above_threshold_divides() {
        // Denom 0.11 passes the strict test.
        // Expect projection scaled by 1/denom.
        let (chassis, ground) = make_bodies(Vec3A::new(0.0, 0.0, -5.0));
        let mut wheel = make_wheel();
        let trans = Affine3A::IDENTITY;
        let y = (1.0f32 - 0.11 * 0.11).sqrt();
        let normal = Vec3A::new(0.0, y, 0.11);
        cast_wheel(&mut wheel, &chassis, &ground, normal, &trans);
        let info = wheel.raycast_info.as_ref().unwrap();
        let inv = 1.0 / 0.11f32;
        let expected_vel = normal.dot(Vec3A::new(0.0, 0.0, -5.0)) * inv;
        assert!(
            (info.clipped_inv_contact_dot_suspension - inv).abs() < 1e-6,
            "clipped {}",
            info.clipped_inv_contact_dot_suspension
        );
        assert!(
            (info.suspension_relative_vel - expected_vel).abs() < 1e-4,
            "rel vel {}",
            info.suspension_relative_vel
        );
    }
}
