//! Default boost consume-rate regression (decimal `0.333` default).
//!
//! The `0`-`100` default is `33.3` (bits `0x42053333`), not `100/3`.
//! Per-tick cost `USED_PER_SECOND * TICK_TIME` is `0.2775` (bits
//! `0x3E8E147B`). The native `0`-`1` scale recompute
//! `f32(dt) * f32(0.333)` is `0.0027750002` (bits `0x3B35DCC7`).

use rocketsim::{
    GameMode, MutatorConfig,
    consts::{TICK_TIME, car::boost::USED_PER_SECOND},
};

#[test]
fn default_rate_is_decimal_33_point_3() {
    assert_eq!(USED_PER_SECOND.to_bits(), 0x4205_3333);
    assert_eq!(USED_PER_SECOND, 33.3f32);
    assert_ne!(USED_PER_SECOND, 100.0f32 / 3.0);
}

#[test]
fn default_per_tick_cost_bits() {
    let cost = USED_PER_SECOND * TICK_TIME;
    assert_eq!(cost.to_bits(), 0x3E8E_147B);
    assert!((cost - 0.2775).abs() < 1e-6);

    // RL `0`-`1` scale recompute from the live decimal rate.
    let norm_cost = f32::from_bits(0x3C08_8889) * f32::from_bits(0x3EAA_7EFA);
    assert_eq!(norm_cost.to_bits(), 0x3B35_DCC7);

    // The old nominal rate differs (100/3 per-tick cost).
    let nominal = (100.0f32 / 3.0) * TICK_TIME;
    assert_eq!(nominal.to_bits(), 0x3E8E_38E4);
    assert!((nominal - cost - 0.0002777874).abs() < 1e-7);
}

#[test]
fn mutator_override_behavior_kept() {
    assert_eq!(
        MutatorConfig::new(GameMode::Soccar).boost_used_per_second,
        USED_PER_SECOND
    );
    assert_eq!(
        MutatorConfig::new(GameMode::Heatseeker).boost_used_per_second,
        0.0
    );
}

#[test]
fn true_rate_consumes_slower_than_nominal() {
    let cost = USED_PER_SECOND * TICK_TIME;
    let nominal = (100.0f32 / 3.0) * TICK_TIME;
    assert!(cost < nominal);
}
