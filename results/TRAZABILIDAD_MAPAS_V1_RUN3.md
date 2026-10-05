# Trazabilidad de mapas "v1" / "run3" — LIMO RoboCup SML

Fecha: 2026-09-17
Método: solo lectura, dirigida (sin repetir la búsqueda masiva de filesystem ya agotada en `AUDITORIA_MAPAS.md`). Fuentes nuevas consultadas: `~/.bash_history`, historial de git de `limoatwork` (solo lectura: `git log`, `git status`, `git show`, sin checkout ni fetch), ramas remotas (`origin/robocup-sim`, inspeccionadas sin checkout).

---

## 1. Búsqueda de los términos en fuentes no revisadas antes

**CONFIRMADO (ausencia total)**: se buscaron las cadenas `v1`, `run3`, `run 3`, `production`, `producción`, `full arena`, `map.pgm`, `map.yaml`, `pbstream`, `/tmp` en:
- `~/.bash_history` completo (2000 líneas) → **cero coincidencias** con `v1`/`run3`/`producción`/`production`, y **cero** menciones de `/tmp`.
- Mensajes de commit de `limoatwork` (`git log --all --grep`, todas las ramas) → **cero coincidencias**.
- Mensajes de commit de la rama remota `origin/robocup-sim` → **cero coincidencias**.

Esto es un hallazgo en sí mismo: ni siquiera en el historial de comandos ejecutados, ni en ningún mensaje de commit de todo el historial git de `limoatwork` (que incluye ~19 commits de múltiples autores desde el import inicial), aparecen los términos "v1" o "run3". **Refuerza con una fuente adicional la conclusión ya alcanzada en `AUDITORIA_MAPAS.md`: son, con alta probabilidad, apodos verbales/informales del equipo, no identificadores técnicos registrados en ningún artefacto del proyecto.**

## 2. Hallazgo nuevo: ningún mapa candidato está bajo control de versiones

**CONFIRMADO** (vía `git status --porcelain -- src/limo_ros2/limo_bringup/maps/` en `limoatwork`, operación de solo lectura):

```
 M src/limo_ros2/limo_bringup/maps/map.pgm         ← modificado, no confirmado (commiteado)
 M src/limo_ros2/limo_bringup/maps/map.yaml        ← modificado, no commiteado
?? src/limo_ros2/limo_bringup/maps/map1.pgm                        ← sin trackear
?? src/limo_ros2/limo_bringup/maps/map1.yaml                       ← sin trackear
?? src/limo_ros2/limo_bringup/maps/map_20260718_185606.pgm         ← sin trackear
?? src/limo_ros2/limo_bringup/maps/map_20260718_185606.yaml        ← sin trackear
?? src/limo_ros2/limo_bringup/maps/map_20260720_160000.pgm         ← sin trackear
?? src/limo_ros2/limo_bringup/maps/map_20260720_160000.yaml        ← sin trackear
?? src/limo_ros2/limo_bringup/maps/map_20260723_193027.*           ← sin trackear (.pbstream, .pgm, .yaml)
?? src/limo_ros2/limo_bringup/maps/map_20260723_195338.*           ← sin trackear
?? src/limo_ros2/limo_bringup/maps/map_20260723_201330.pbstream    ← sin trackear
```

**Ninguno de los mapas candidatos identificados en `AUDITORIA_MAPAS.md` está confirmado en git.** El único mapa que alguna vez estuvo bajo control de versiones es una versión **anterior y más pequeña** de `map.pgm`/`map.yaml`, committeada en:

```
commit 1aa72b3e363093c05f7585491bf5480c79323ac0
Author: IgorNunnes <IgorNunnes@users.noreply.github.com>
Date:   2026-07-18 18:27:33 -0300
Update limoatwork workspace
```

**CONFIRMADO por comparación de contenido**: el `map.yaml` de ese commit dice `resolution: 0.05`, `origin: [-1.11, -1.57, 0]` — **idéntico** al archivo hoy guardado por separado como `map_20260718_185606.yaml` (mismo origen, misma resolución). Esto confirma que **el `map.pgm` original commiteado por IgorNunnes el 18 de julio es, con alta confianza, el mismo archivo que hoy se conserva localmente bajo el nombre `map_20260718_185606.pgm`** — en algún momento posterior alguien lo copió a un nombre con timestamp antes de sobrescribir `map.pgm` con una versión nueva (la del 23 de julio, 0.03 m, que es la que existe hoy con ese nombre, **sin commitear**).

## 3. Hipótesis reformulada sobre "v1"

**HIPÓTESIS (nueva, con más respaldo que las anteriores, pero no confirmada)**: si "v1" se refiere a la **primera versión de mapa que el equipo consideró suficientemente estable como para compartir/registrar formalmente** (a diferencia de iteraciones locales de prueba), el candidato con más peso pasa a ser **el mapa commiteado por IgorNunnes el 2026-07-18** (hoy conservado localmente como `map_20260718_185606.pgm`) — es el único mapa de todo el proyecto que en algún momento fue tratado con la formalidad de un commit de git, mientras que todas las iteraciones posteriores (incluidas las de mayor resolución/escala) quedaron **únicamente como archivos locales no versionados**.

Esto **no contradice pero sí reencuadra** el candidato señalado en `AUDITORIA_MAPAS.md` (`map.pgm` actual, 0.03 m) como "el más fuerte por evidencia indirecta" — aquel análisis se basó en recencia de modificación y coincidencia de resolución con la nota de CLAUDE.md; este análisis aporta un criterio distinto (formalidad de versionado) que apunta a un archivo diferente. **Ambas hipótesis conviven sin resolverse; ninguna es más "correcta" sin confirmación del equipo.**

## 4. "run3" — sin nueva evidencia

**DESCONOCIDO, sin cambios respecto a `AUDITORIA_MAPAS.md` y `AUDITORIA_FULL_ARENA.md`.** No se encontró ninguna referencia textual nueva. El candidato por escala (metadata de `stage3_cartographer_static`, 63.75 m²) sigue siendo el más fuerte por tamaño, pero:
- Es anterior al inicio del historial de shell disponible (no se pudo reconstruir el comando que lo generó — ver `TRAZABILIDAD_SLAM_CONFIGS.md` §0).
- No tiene archivo persistido, ni commiteado ni local.

## 5. ¿Alguno estuvo solo en `/tmp`?

**DESCONOCIDO — sin evidencia ni a favor ni en contra.** No hay ninguna mención de `/tmp` en `~/.bash_history` (rango visible) relacionada con mapas. La búsqueda de filesystem ya realizada en `AUDITORIA_FULL_ARENA.md` confirmó que **hoy** no existe nada en `/tmp` relacionado — pero, tal como advierte la instrucción de esta fase, eso no descarta que algo haya existido ahí temporalmente y se haya perdido al limpiarse `/tmp` (comportamiento estándar de ese directorio entre reinicios o limpiezas periódicas del sistema). No se encontró evidencia positiva de que esto haya ocurrido; tampoco se puede descartar.

## 6. Resumen de certeza

| Pregunta | Estado |
|---|---|
| ¿"v1"/"run3" aparecen en algún commit, comando o mensaje registrado? | **CONFIRMADO: no aparecen en ninguna fuente consultada** |
| ¿Algún mapa candidato está commiteado en git? | **CONFIRMADO: solo una versión antigua y más pequeña (18 de julio), hoy conservada como `map_20260718_185606`** |
| ¿El `map.pgm` actual (0.03 m) es el mismo que el commiteado? | **CONFIRMADO: no** — el commiteado tiene resolución 0.05 m y coincide con `map_20260718_185606` |
| ¿Cuál es "v1"? | **DESCONOCIDO** — dos hipótesis no confirmadas y no excluyentes (recencia vs. formalidad de versionado) |
| ¿Cuál es "run3"/"full arena"? | **DESCONOCIDO** — candidato por escala sin archivo persistido |
| ¿Existió alguno solo en `/tmp`? | **DESCONOCIDO** — sin evidencia en ningún sentido |

---

*Documento generado por búsqueda dirigida de solo lectura en `~/.bash_history` y git de `limoatwork` (`log`, `status`, `show`, sin checkout ni fetch). No se modificó, copió ni movió ningún archivo. No se ejecutó ROS ni hardware.*
