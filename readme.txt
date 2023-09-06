References:
- Embedeed rust book and repo: 
    https://docs.rust-embedded.org/discovery/f3discovery/
    https://github.com/rust-embedded/discovery
- Jajakub's Youtube series: 
    https://docs.rust-embedded.org/discovery/f3discovery/


Basic Rust STM32 blue pill setup:
1) Choose ARM target
    - rustup target list
    - rustup target add <target> 
        - rustup target add thumbv7m-none-eabi for stm32f1xx
2) Creating a new rust project:
    - for binary: cargo new hello_world --bin
    - for library:  cargo new hello_world --lib
    - (cargo init also works)
3) Specify dependencies in cargo.toml
    ```
    [dependencies]
    embedded-hal = "0.2.7"
    cortex-m = "0.7.6"
    cortex-m-rt = "0.7.1"
    cortex-m-semihosting = "0.5.0"

    [dependencies.stm32f1xx-hal]
    version = "0.10.0"
    features = ["rt", "stm32f103", "medium"]

    # default runner starts a GDB session, which requires OpenOCD to be running
    # eg. openocd -f interface/stlink-v2-1.cfg -f target/stm32f1x.cfg
    [target.thumbv7m-none-eabi]
    runner = "gdb-multiarch -q -x ./openocd.gdb"
    rustflags = [
        "-C", "link-arg=-Tlink.x",
    ]

    [build]
    target = "thumbv7m-none-eabi"
    ```
4) Add openocd file for GDB debugging as specified in dependencies above (see this project for an example)
5) Create a linker script (memory.x) to specify flash and RAM locations
    - For stm32f1xx
        - flash begins at 0x08000000
        - internal sram starts at 0x200000000, ends at 0x400000000 (20k)
6) cargo build
7) cargo run

Notes:
1) Semihosting using SWD protocol allows you to send stdio information to host for debugging on ARM devices. This replaces the JTAG debugger on other embedded devices.
2) Look at micro datasheet for memory layout to write linker script
3) Embedded programs have no std and no main
    - this makes it difficult to write unit test or using certain Rust functionality
    - this is why we use hprintln which prints to SWD instead of standard println to stdio
    - knurling library may have tools to get around this
4) OpenOCD is a GDB server (alternatives include JlinkServer, etc.). We connect to the server and send debug commands to micro using the server.

Steps to run:
- On one terminal:
    - openocd -f interface/stlink-v2-1.cfg -f target/stm32f1x.cfg
        - note that openocd.gdb will have extra instructions for gdb to execute (eg. monitor arm semihosting enable)
- In another terminal: 
    - cargo run

Journal:
- BME280 cannot be powered off USB stick alongside STM32F1 blue pill micro - ADC readings are wrong/inconsistent
    - When powered off separate 3.3V power supply, pressure reading is correct and stable
- If run out of memory since rust binaries can be big if compiler does not optimize for size:
    - Add this to cargo.toml:
        ```
        [profile.dev]
        codegen-units = 1
        debug = 0
        lto = true
        opt-level = "z"
        ```
- Testing:
    - https://www.reddit.com/r/rust/comments/12xcmru/how_to_perform_unit_tests_for_an_embedded/
    - Usual convention is to have library in its own crate and test business logic on the host where we can use std library for tests 
    - Then binary for micro is in its own crate
    - So have to split project into library and binary, not just all in one
    - Workspaces may be the solution:
        - https://doc.rust-lang.org/book/ch14-03-cargo-workspaces.html
        - it does work, but now binary is too big again
            - bme280 binary is not compiled with optimization it seems?
            - unless we put optimization instructions at root cargo.toml, but
              if we do that then cargo test doesn't work in bme280 directory
    - solution - have two profiles in root cargo.toml - one for dev and one for test
        - allows build to work inside exec folder
        - allows test to work inside bme280 folder
        - unfortunately allows neither cargo build nor cargo test to work When
          called inside root folder...
        - may need xtask to have custom build task when calling from root folder

To Do:
- implement custom build task so cargo build and cargo test can be run
  at top level (xtask)
- figure out why memory.x needs to be in root directory
- implement unit tests
- figure out why humidity sensor is reading 0
- implement error types
- implement pass by reference and lifetimes
- figure out why stm32 crashes on overflow?

Next month project:
- port over to embassy?? 
    - reasons: shared use of timers
        - better API
        - room to grow