.PHONY: all
all: build

lint:
	cargo clippy

fmt: lint
	cargo fmt

test: fmt
	cargo test

build: test
	cargo build

release: test
	cargo build --release

clean:
	cargo clean
