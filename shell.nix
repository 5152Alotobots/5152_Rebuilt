{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  buildInputs = [ pkgs.jdk17 pkgs.gradle pkgs.jdt-language-server ];
  shellHook = ''
    export JAVA_HOME=${pkgs.jdk17}
  '';
}
