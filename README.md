# EcoCAR CAV controls, MATLAB/Simulink

ACC and lane centering in simulation for EcoCAR Year 1 CAV controls, with two controllers on
the same plant and scenario: PID (Simulink PID blocks) and MPC (Model Predictive Control Toolbox
ACC and lane keeping blocks). `WRITEUP.md` has the results.

- `cav_params.m`: every number. MPC values are written into the model at build time.
- `build_cav_model.m`: builds `cav_pid.slx` or `cav_mpc.slx` from code.
- `run_cav_demo.m`: runs both, prints pass/fail checks, saves plots in `results/`.

    build_cav_model('pid'); build_cav_model('mpc')   % after editing the builder or MPC values
    run_cav_demo                                      % about 30 s
    makeVideo = true; run_cav_demo                    % plus results/*_demo.mp4 (needs ffmpeg)

Needs MATLAB R2026b with Simulink, Vehicle Dynamics Blockset, Automated Driving Toolbox and
Model Predictive Control Toolbox.
