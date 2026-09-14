//! Boost armed-bit seeding from RLPR impulse history.
//!
//! Target: b737 boost machine (`rl-exam/analysis/boost-affordability-gate-decision.md`).
//! `is_boosting` is the armed bit; `boosting_time` is time since arm. A fired
//! arrival tick ends ARMED (unless it depleted to zero); an unfired arrival
//! tick ends DISARMED. Seeding applies only to recordings that carry impulse
//! data; impulse-less legacy captures keep live-bit evolution.

use rocketsim::consts::TICK_TIME;

use super::cpp_records::ImpulseRecordType;
use super::tick_record::TickRecord;

/// True when any car in any tick carries impulse records.
///
/// Gates history seeding: impulse-less legacy captures must not be forced
/// CLEAR, which would discard the live latch.
pub fn recording_has_impulse_data(ticks: &[TickRecord]) -> bool {
    ticks.iter().any(|tick| {
        tick.car_records
            .iter()
            .any(|car| !car.phys.impulse_records().is_empty())
    })
}

fn fired_in_tick(tick: &TickRecord, car_idx: usize) -> bool {
    tick.car_records.get(car_idx).is_some_and(|car| {
        car.phys
            .impulse_records()
            .iter()
            .any(|r| r.impulse_type == ImpulseRecordType::Boost)
    })
}

/// Infer `(is_boosting, boosting_time)` at one recorded state tick.
///
/// The bit comes from the source-tick Boost impulse; zero recorded fuel
/// forces CLEAR. Time rebuilds from the contiguous Boost run ending at the
/// state tick, capped at 12 ticks (`MIN_TIME`). Every sample is checked;
/// none are skipped. The walk stays inside the contiguous `run_start` so
/// teleport gaps never extend the count. `None` on out-of-range input.
pub fn reconstruct_boost_state(
    ticks: &[TickRecord],
    run_start: usize,
    state_index: usize,
    car_idx: usize,
) -> Option<(bool, f32)> {
    if state_index >= ticks.len() || run_start > state_index {
        return None;
    }
    let state_tick = &ticks[state_index];
    let car = state_tick.car_records.get(car_idx)?;
    if !fired_in_tick(state_tick, car_idx) {
        return Some((false, 0.0));
    }
    if car.boost_amount <= 0.0 {
        return Some((false, 0.0));
    }
    let first = run_start.max(state_index.saturating_sub(11));
    let mut count = 0usize;
    let mut idx = state_index + 1;
    while idx > first {
        idx -= 1;
        if !fired_in_tick(&ticks[idx], car_idx) {
            break;
        }
        count += 1;
        if count >= 12 {
            break;
        }
        if idx == 0 {
            break;
        }
    }
    if count == 0 {
        return Some((false, 0.0));
    }
    Some((true, count as f32 * TICK_TIME))
}

#[cfg(test)]
mod tests {
    use crate::rlpr::cpp_records::{
        CarRecord, ControlsRecord, ImpulseRecordType, PhysRecord, VecRecord, WheelRecord,
    };
    use crate::rlpr::tick_record::TickRecord;

    use super::*;

    fn blank_phys() -> PhysRecord {
        unsafe { std::mem::zeroed() }
    }

    fn car_with(boost_amount: f32, impulse_types: &[ImpulseRecordType]) -> CarRecord {
        let mut phys = blank_phys();
        phys.set_test_impulses(impulse_types);
        CarRecord {
            phys,
            is_on_ground: false,
            is_jumping: false,
            is_flipping: false,
            jump_time: 0.0,
            flip_time: 0.0,
            has_jumped: false,
            double_jumped_or_flipped: false,
            has_flip: false,
            flip_rel_torque: VecRecord::new(0.0, 0.0, 0.0),
            boost_amount,
            is_touching_ball: false,
            prev_controls: ControlsRecord {
                throttle: 0.0,
                steer: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
                jump: false,
                boost: false,
                handbrake: false,
            },
            wheels: [WheelRecord {
                susp_length: 0.0,
                susp_rel_vel: 0.0,
                has_contact: false,
                contact_normal: VecRecord::new(0.0, 0.0, 1.0),
                steer_amount: 0.0,
                engine_force: 0.0,
                brake: 0.0,
                lat_friction: 0.0,
                long_friction: 0.0,
                extra_pushback: 0.0,
            }; 4],
        }
    }

    fn tick(car: CarRecord) -> TickRecord {
        TickRecord {
            car_records: vec![car],
            ball_record: blank_phys(),
        }
    }

    fn boost_tick(boost_amount: f32) -> TickRecord {
        tick(car_with(boost_amount, &[ImpulseRecordType::Boost]))
    }

    fn quiet_tick() -> TickRecord {
        tick(car_with(1.0, &[]))
    }

    #[test]
    fn unfired_state_tick_seeds_disarmed() {
        let ticks = vec![quiet_tick(), quiet_tick()];
        assert_eq!(reconstruct_boost_state(&ticks, 0, 1, 0), Some((false, 0.0)));
    }

    #[test]
    fn fired_state_tick_seeds_armed_with_single_tick_time() {
        let ticks = vec![quiet_tick(), boost_tick(1.0)];
        assert_eq!(
            reconstruct_boost_state(&ticks, 0, 1, 0),
            Some((true, TICK_TIME))
        );
    }

    #[test]
    fn contiguous_run_rebuilds_time_and_caps_at_twelve() {
        let ticks: Vec<_> = (0..20).map(|_| boost_tick(1.0)).collect();
        assert_eq!(
            reconstruct_boost_state(&ticks, 0, 4, 0),
            Some((true, 5.0 * TICK_TIME))
        );
        let capped = reconstruct_boost_state(&ticks, 0, 19, 0).unwrap();
        assert!(capped.0);
        assert!((capped.1 - 12.0 * TICK_TIME).abs() < 1e-9);
    }

    #[test]
    fn gap_breaks_the_run_and_run_start_bounds_history() {
        let mut ticks: Vec<_> = (0..10).map(|_| boost_tick(1.0)).collect();
        ticks[5] = quiet_tick();
        assert_eq!(
            reconstruct_boost_state(&ticks, 0, 9, 0),
            Some((true, 4.0 * TICK_TIME))
        );
        assert_eq!(reconstruct_boost_state(&ticks, 0, 5, 0), Some((false, 0.0)));
        // History before the contiguous run start is out of reach.
        assert_eq!(
            reconstruct_boost_state(&ticks, 7, 9, 0),
            Some((true, 3.0 * TICK_TIME))
        );
    }

    #[test]
    fn zero_fuel_forces_disarmed_despite_impulse() {
        let ticks = vec![quiet_tick(), boost_tick(0.0)];
        assert_eq!(reconstruct_boost_state(&ticks, 0, 1, 0), Some((false, 0.0)));
    }

    #[test]
    fn out_of_range_input_returns_none() {
        let ticks = vec![boost_tick(1.0)];
        assert_eq!(reconstruct_boost_state(&ticks, 0, 1, 0), None);
        assert_eq!(reconstruct_boost_state(&ticks, 1, 0, 0), None);
        assert_eq!(reconstruct_boost_state(&ticks, 0, 0, 3), None);
    }

    #[test]
    fn impulse_data_gate_detects_legacy_captures() {
        let with_data = vec![quiet_tick(), boost_tick(1.0)];
        assert!(recording_has_impulse_data(&with_data));
        let legacy = vec![quiet_tick(), quiet_tick()];
        assert!(!recording_has_impulse_data(&legacy));
    }

    #[test]
    fn non_boost_impulses_do_not_arm_but_count_as_data() {
        let ticks = vec![
            tick(car_with(1.0, &[ImpulseRecordType::Jump])),
            tick(car_with(1.0, &[ImpulseRecordType::Jump])),
        ];
        assert!(recording_has_impulse_data(&ticks));
        assert_eq!(reconstruct_boost_state(&ticks, 0, 1, 0), Some((false, 0.0)));
    }
}
