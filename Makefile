.PHONY: all

all: format release

lint:
	cargp fmt --check && cargo clippy

format:
	cargo fmt

test: lint
	cargo test

build:
	cargo build

flamegraph:
	cargo flamegraph --example visualizer

release: test
	cargo build --release

publish: release
	cargo publish --token ${CRATES_IO_TOKEN}

.PHONY: docs
docs: test
	cargo doc --color always --no-deps

clean:
	cargo clean
