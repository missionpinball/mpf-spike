FROM messense/rust-musl-cross:arm-musleabi
RUN rustup target add armv5te-unknown-linux-musleabi
