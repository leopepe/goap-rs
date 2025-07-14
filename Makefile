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

flamegraph:
	cargo flamegraph --example visualizer

release: test
	cargo build --release

.PHONY: docs
docs: test
	cargo doc --color always --no-deps

clean:
	cargo clean
