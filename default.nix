with import <nixpkgs> {};
stdenv.mkDerivation {
  name = "zephyr";
  buildInputs = [
    gcc6
    cmake
    ninja
    gcc-arm-embedded
    ncurses
    python36
    python36Packages.pyaml
    python36Packages.pyelftools
    ];
  shellHook = 
    ''
    export ZEPHYR_BASE=/home/tburdick/src/zephyr
    export GCCARMEMB_TOOLCHAIN_PATH="/nix/store/dbhm1b0h3v55p7jkv7nl88p7k145jqik-gcc-arm-embedded-6-2017-q2-update/"
    export ZEPHYR_TOOLCHAIN_VARIANT=gccarmemb
    '';
}
