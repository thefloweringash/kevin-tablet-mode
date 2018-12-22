{ nixpkgs ? <nixpkgs> }:

with (import nixpkgs {});

stdenv.mkDerivation {
  name = "kevin-tablet-mode";
  src = lib.cleanSource ./.;
  nativeBuildInputs = [ cmake ];
}
