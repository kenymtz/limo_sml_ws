*** NO VÁLIDO PARA CONCLUSIONES ***

Barrido R4-G de progress_checker (required_movement_radius / movement_time_allowance) DETENIDO el 2026-10-03
a pedido del usuario antes de R4-G0.

Motivos:
1. Datos PARCIALES: PR005 S1-S3 completos, S4 incompleto; PR003/PT35/PT45 sin corridas.
2. Ejecutado con plan_mode=fixed_plan. R4-G0 demostró que fixed_plan NO es fiel para atascos cerca del goal
   / progress_checker (Nav2 real replanifica a 1 Hz y DWB::setPlan reinicia critics y latches).
   Ver results/navigation_offline_lab/DWB_R4G0_REPLAN_FIDELITY_REPORT.md

Se conserva solo como registro. No usar en tablas, comparaciones ni decisiones.
