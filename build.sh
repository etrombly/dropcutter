#!/bin/bash
cargo vendor
docker run --rm --user "$(id -u)":"$(id -g)" -v "$PWD":/usr/src/myapp -w /usr/src/myapp -t rust cargo build --release
