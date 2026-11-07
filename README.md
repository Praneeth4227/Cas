# EV–HESS Energy Management

LSTM- and GRU-based supercapacitor current reference prediction for an electric vehicle hybrid energy storage system.

## Run in MATLAB

Use MATLAB R2025b Update 3 with Simulink. Extract the complete folder to a short writable location such as E:/EVHESS_Submission. Open run_EV_HESS_Project.m from that extracted folder and click Run. Alternatively, run the script by its full path from any MATLAB current folder. If an older same-named editor file is current, accept MATLAB's Change Folder prompt or close that duplicate first. Do not open the MATLAB file directly inside the compressed archive.

The script checks the environment, validates inputs and prediction provenance, opens EV_HESS_Energy_Management.slx, simulates eight cases, prints numerical comparisons and displays nine figures. Cache and code-generation folders are created beside the project, outside the submission. No builder, launcher or manual path configuration is required. An already loaded unchanged model from another location is closed; unsaved model edits are never discarded.

## Files and dependencies

- run_EV_HESS_Project.m: single MATLAB execution file with local functions for fixed parameters, validated inputs, prediction import, shared plant execution, full-precision CSV export, energy metrics and plotting.
- EV_HESS_Energy_Management.slx: single integrated model with drive-cycle inputs, vehicle/drivetrain, battery, hybrid storage controller, supercapacitor/converter, reference selection and measurement/energy accounting.
- LSTM_model.py and GRU_model.py: independent inference implementations, including architectures, ten-sample causal windows with repeat-first padding, saved standardization, immutable checkpoint loading and verification against saved predictions. Features are speed_mps followed by acceleration_mps2; outputs are signed amperes. Recurrent state is zero for each independent window.
- ML_Assets/{LSTM,GRU}: unchanged trained weights and scalers, with matching cleaned model metadata. LSTM uses 64 hidden units and 17,473 parameters; GRU uses 74 and 17,391 parameters. No retraining is required.
- Data: approved 1 s speed/acceleration data, original public EUDC/IM240 schedules under Original, printed Equation 18 Scenario A analytical labels under Analytical_Reference, and validated MAT/CSV predictions under ML_Predictions.
- requirements.txt: Python inference dependencies; MATLAB uses the saved predictions and does not require Python. A historical dependency lockfile was not available. Run either Python file to verify both cycles; optional --refresh saves review predictions separately and does not replace validated imports automatically.
- Results/Simulation_Data: EUDC/IM240 × BatteryOnly, AnalyticalHESS, LSTM_HESS and GRU_HESS, with trace.mat, trace.csv and case_checks.csv.
- Results/Metrics: numerical comparisons. Results/Figures: nine PNG plus nine editable FIG outputs. Figures appear automatically in MATLAB.
- Verification: full-precision reference metrics, verification checklist, dependency description, payload inventory and SHA-256 checksums. The checksum list covers every file except itself; the inventory excludes itself and the checksum list to avoid circular hashing.

## Modelling assumptions and interpretation

The plant is the approved reduced-order prescribed-speed system, not a detailed switching inverter/PMSM or physical vehicle test. It uses an ideal 350 V DC bus/battery, 20 kWh battery with initial SOC 0.80, conditional 32 F supercapacitor bank initially at 280.8 V, zero initial inductor current and PI integral state, a 1 ms plant step and 1 s reference hold. Converter parameters, PI gains and protection equations are fixed across cases. Printed Equation 18 remains the analytical training target. Missing gear ratio and wheel radius are not invented; detailed motor speed/torque feasibility is not claimed.

Positive battery current denotes discharge; negative denotes charging. RMS current is weighted by interval duration. Storage-adjusted energy equals battery-terminal energy plus initial-minus-final SC and inductor energy. HESS configurations demonstrate battery-current smoothing; net energy savings are not demonstrated after storage-state changes and converter losses are accounted for. Final networks were trained on both included cycles, so these plant comparisons use in-sample offline prediction replay.

The current execution results and remaining limitations are recorded in Verification/FINAL_CHECK.md.
