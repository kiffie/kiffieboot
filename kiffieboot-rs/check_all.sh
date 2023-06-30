#!/usr/bin/sh
# check for all reasonable feature combinations

export RUSTFLAGS="$RUSTFLAGS -Dwarnings"

cargo_check() {
    local features=$1
    echo "cargo check --features $features"
    cargo check --features $features || exit
    cargo clippy --features $features || exit
    echo
}

cargo_check pic32mx2x0 --no-default-features
cargo_check pic32mx2x0

cargo_check pic32mx2x4 --no-default-features
cargo_check pic32mx2x4

cargo_check pic32mx470 --no-default-features
cargo_check pic32mx470

cargo test --doc --features pic32mx2x0

