extern crate alloc;

mod messages;
#[cfg(all(not(target_arch = "wasm32"), feature = "rc"))]
#[path = "sim/rc_joystick.rs"]
mod rc_joystick;
mod sim_support;
mod tasks;

include!("sim.rs");

fn main() {
    run_bevymon();
}
