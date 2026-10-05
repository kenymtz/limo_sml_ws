1. **R4-C (offline): ajuste de critics de DWB en copia experimental, con este mismo simulador y los mismos 6 escenarios**:
   - `RotateToGoal` activo (está comentado en la config real): ataca el ~26–30 % de corridas que llegan en xy y abortan por yaw.
   - ObstacleFootprint con `scale` que compense el factor de resolución (p. ej. 2.0 → peso efectivo 0.1) y/o BaseObstacle con `sum_scores`: buscar la escala que no bloquee N1 (mínimo local de A) sin volver a los contactos.
   - Métrica de aceptación: 0 contactos en S1–S5, 0 contactos con penetración > 1 cm en S6, llegada ≥ 60/64 en S1.
2. Paralelo, barato: repetir B en S4/S6 con varias semillas para estimar la tasa de contacto con latencia alta. Hoy es 2/48 con una semilla.
3. **No** llevar ObstacleFootprint solo a `navigation2_real.yaml` todavía. Reduce contactos en lazo cerrado (5 → 0 fuera del estrés de latencia) y desbloquea N1, pero no resuelve la llegada en yaw ni el mínimo local de `around_obstacle`, que son las fallas dominantes. Cualquier cambio de producción requiere tu confirmación y una prueba física supervisada.
4. Medir en el robot (solo lectura de bags, sin mover) la latencia real cmd → movimiento a varias velocidades, y el error de AMCL en la arena, para reemplazar los σ supuestos.
