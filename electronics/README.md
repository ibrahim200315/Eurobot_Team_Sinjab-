# Electronics (Sinjab)

This folder contains the electronics architecture, BOM, schematics, and interface definitions for the SINJAB robot.

## Quick status (from weekly presentations)
- TF tree is complete: map → odom → base_link → (wheels, lidar_link, camera_link).  
- SLAM Toolbox integrated (async SLAM + loop closure) with stable mapping and localization.

## Folder map
- `bom/` : BOM spreadsheets and purchase notes
- `power/` : nominal voltage rails, power budgeting, fuses, safety
- `schematics/` : KiCad schematics (power distribution, sensors IO, power & motion)
- `interfaces/` : MCU ↔ Raspberry Pi protocol, connector pinouts (to be finalized)

## Key documents
- Nominal voltage rails and power estimation: see `power/Nominal_voltages_of_components.pdf`
