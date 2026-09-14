//! Boost armed-bit state machine batteries.
//!
//! Target: b737 amount manager plus dispatcher plus latch batteries (see
//! `rl-exam/analysis/boost-affordability-gate-decision.md`). `is_boosting`
//! is the armed bit; `boosting_time` is time since arm. Amounts here use the
//! RocketSim `0`-`100` scale.

use glam::Vec3A;
use rocketsim::{Arena, CarBodyConfig, GameMode, Team, consts::TICK_TIME};

fn init_once() {
    static INIT: std::sync::Once = std::sync::Once::new();
    INIT.call_once(|| {
        rocketsim::init(
            concat!(env!("CARGO_MANIFEST_DIR"), "/collision_meshes"),
            true,
        )
        .unwrap();
    });
}

fn cost_per_tick() -> f32 {
    (100.0 / 3.0) * TICK_TIME
}

fn fresh_airborne(boost: f32, armed: bool, time: f32) -> (Arena, usize) {
    let mut arena = Arena::new(GameMode::Soccar);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let mut state = *arena.get_car_state(car);
    state.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    state.phys.vel = Vec3A::ZERO;
    state.phys.ang_vel = Vec3A::ZERO;
    state.phys.rot_mat = glam::Mat3A::IDENTITY;
    state.is_on_ground = false;
    state.wheels_with_contact = [false; 4];
    state.boost = boost;
    state.is_boosting = armed;
    state.boosting_time = time;
    state.time_since_boosted = 0.0;
    arena.set_car_state(car, state);
    (arena, car)
}

fn step_once(arena: &mut Arena, car: usize, held: bool) {
    let mut state = *arena.get_car_state(car);
    state.controls.boost = held;
    state.prev_controls.boost = held;
    arena.set_car_state(car, state);
    arena.set_car_controls(car, state.controls);
    arena.step_tick();
}

#[test]
fn disarmed_sliver_fires_free_without_consume() {
    init_once();
    let (mut arena, car) = fresh_airborne(0.1, false, 0.0);
    step_once(&mut arena, car, true);
    let state = *arena.get_car_state(car);
    assert!(state.is_boosting);
    assert!((state.boosting_time - TICK_TIME).abs() < 1e-9);
    assert!((state.boost - 0.1).abs() < 1e-6);
}

#[test]
fn disarmed_release_stays_silent() {
    init_once();
    for boost in [0.0, 0.1, 50.0] {
        let (mut arena, car) = fresh_airborne(boost, false, 0.0);
        step_once(&mut arena, car, false);
        let state = *arena.get_car_state(car);
        assert!(!state.is_boosting);
        assert_eq!(state.boosting_time, 0.0);
        assert!((state.boost - boost).abs() < 1e-6);
    }
}

#[test]
fn exact_cost_fires_once_then_depletes_silently() {
    init_once();
    let cost = cost_per_tick();
    let (mut arena, car) = fresh_airborne(cost, false, 0.0);
    step_once(&mut arena, car, true);
    assert!(arena.get_car_state(car).is_boosting);
    step_once(&mut arena, car, true);
    let state = *arena.get_car_state(car);
    assert!(!state.is_boosting);
    assert_eq!(state.boost, 0.0);
    assert_eq!(state.boosting_time, 0.0);
}

#[test]
fn cost_plus_one_ulp_fires_twice() {
    init_once();
    let cost = cost_per_tick();
    let plus = f32::from_bits(cost.to_bits() + 1);
    let (mut arena, car) = fresh_airborne(plus, false, 0.0);
    step_once(&mut arena, car, true);
    assert!(arena.get_car_state(car).is_boosting);
    step_once(&mut arena, car, true);
    assert!(
        arena.get_car_state(car).is_boosting,
        "cost-plus-1ulp fires a paid second tick"
    );
    step_once(&mut arena, car, true);
    assert!(!arena.get_car_state(car).is_boosting);
    assert_eq!(arena.get_car_state(car).boost, 0.0);
}

#[test]
fn armed_sliver_entry_depletes_silently() {
    init_once();
    let (mut arena, car) = fresh_airborne(0.1, true, TICK_TIME);
    step_once(&mut arena, car, true);
    let state = *arena.get_car_state(car);
    assert!(!state.is_boosting);
    assert_eq!(state.boost, 0.0);
    assert_eq!(state.boosting_time, 0.0);
}

#[test]
fn single_press_fires_twelve_then_disarms() {
    init_once();
    let (mut arena, car) = fresh_airborne(100.0, false, 0.0);
    step_once(&mut arena, car, true);
    assert!(arena.get_car_state(car).is_boosting);
    for _ in 2..=12 {
        step_once(&mut arena, car, false);
        assert!(arena.get_car_state(car).is_boosting);
    }
    step_once(&mut arena, car, false);
    let state = *arena.get_car_state(car);
    assert!(!state.is_boosting, "tick 13 consumes once and disarms");
    assert_eq!(state.boosting_time, 0.0);
    let frozen = state.boost;
    assert!(frozen > 0.0 && frozen < 100.0);
    step_once(&mut arena, car, false);
    let state = *arena.get_car_state(car);
    assert!(!state.is_boosting);
    assert!((state.boost - frozen).abs() < 1e-6);
}

#[test]
fn held_fires_beyond_twelve_without_reset() {
    init_once();
    let cost = cost_per_tick();
    let (mut arena, car) = fresh_airborne(100.0, false, 0.0);
    for _ in 0..30 {
        step_once(&mut arena, car, true);
        assert!(arena.get_car_state(car).is_boosting);
    }
    let state = *arena.get_car_state(car);
    assert!((state.boost - (100.0 - 29.0 * cost)).abs() < 1e-3);
    assert!((state.boosting_time - 30.0 * TICK_TIME).abs() < 1e-6);
}

#[test]
fn repress_before_disarm_sustains_without_reset() {
    init_once();
    let (mut arena, car) = fresh_airborne(100.0, false, 0.0);
    step_once(&mut arena, car, true);
    for _ in 0..5 {
        step_once(&mut arena, car, false);
    }
    let before = arena.get_car_state(car).boost;
    step_once(&mut arena, car, true);
    let state = *arena.get_car_state(car);
    assert!(state.is_boosting);
    assert!(state.boost < before, "re-press before expiry stays paid");
    assert!(state.boosting_time > 6.0 * TICK_TIME);
}

#[test]
fn repress_after_disarm_arms_free() {
    init_once();
    let (mut arena, car) = fresh_airborne(100.0, false, 0.0);
    step_once(&mut arena, car, true);
    for _ in 0..11 {
        step_once(&mut arena, car, false);
    }
    step_once(&mut arena, car, false);
    assert!(!arena.get_car_state(car).is_boosting);
    let before = arena.get_car_state(car).boost;
    step_once(&mut arena, car, true);
    let state = *arena.get_car_state(car);
    assert!(state.is_boosting);
    assert!((state.boost - before).abs() < 1e-6);
    assert!((state.boosting_time - TICK_TIME).abs() < 1e-9);
}
