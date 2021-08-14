{
  description = "sbvh";
  
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system: let
        pkgs = import nixpkgs { inherit system; overlays = [ rust-overlay.overlay ]; };
      in {
        devShell = pkgs.mkShell {
          buildInputs = let
            rust = pkgs.rust-bin.stable.latest.default; 
          in [
            (rust.override { extensions = [ "rust-src" ]; }) pkgs.linuxPackages.perf
          ];
        };
      }
    );
}
