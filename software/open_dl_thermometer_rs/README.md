# Programming
Install the programmer using
`cargo install --locked elf2uf2-rs`
Change the runner in `.config/cargo.toml` to `elf2uf2-rs`
Then build and program the executable with
`cargo run` 
as usual.

# SWD/debug builds
`cargo install probe-rs-tools --locked`
Change the runner in `.config/cargo.toml` to `probe-rs`
Follow any platform-specific requirements at https://probe.rs/docs/getting-started/probe-setup
