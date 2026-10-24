{
  description = "mesh-sampling: a tool and library to convert meshes to pointclouds / convex files";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
  };

  outputs =
    inputs:
    inputs.mc-rtc-nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        extraPackages = [ "ninja" ];
        overrideAttrs.mesh-sampling =
          { drv-prev, pkgs-final, ... }:
          {
            src = lib.cleanSource ./.;
            buildInputs = [
              pkgs-final.cli11
              pkgs-final.jrl-cmakemodules
            ];
            nativeBuildInputs = drv-prev.nativeBuildInputs ++ [
              pkgs-final.gtest
              pkgs-final.ninja
            ];
            cmakeFlags = (drv-prev.cmakeFlags or [ ]) ++ [
              "-GNinja"
            ];
            doCheck = true;
            meta.mainProgram = "mesh_sampling";
          };
      }
    );
}
