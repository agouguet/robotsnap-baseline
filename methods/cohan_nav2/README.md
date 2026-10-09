# CoHAN-Nav2 avec RobotSNAP

Faire tourner [CoHAN-Nav2](https://github.com/sphanit/CoHAN-Nav2) - la version
ROS 2 / nav2 de CoHAN, dont HATEB est le planificateur local - contre
RobotSNAP.

## Pourquoi cette voie plutot que l'image ROS 1

Le dossier `../hateb/` fait tourner l'ancienne image ROS 1 de HATEB. Elle marche,
mais elle coute trois conteneurs, un pont ROS 1 <-> ROS 2, et deux problemes que
ce pont introduit : `/tf_static` ne le franchit pas, et les messages y arrivent
avec des reperes prefixes d'un `/` que tf2 refuse. CoHAN-Nav2 supprime tout cela :
c'est un graphe ROS 2, il parle directement a `robotsnap bridge --ros2`.

## Etat

- **Compile** : les 13 paquets se construisent sur la machine hote
  (3 min 19 s, avertissements seulement, aucune erreur).
- **Espace de travail** : `work/cohan_nav2_ws`, un `colcon` normal, source dans
  `work/cohan_nav2_ws/src`. Il est ignore par git : le depot porte les sources
  et les epingles, jamais les artefacts de build.
- **Pre-requis installe** : `libopenblas-dev` est present, le plugin HATEB se
  charge et s'initialise.

## Installation de l'espace de travail

```bash
robotsnap-baseline install cohan_nav2
robotsnap-baseline check   cohan_nav2
```

`install` clone l'amont epingle (`pins.env`), met les sous-modules a jour,
installe `libopenblas-dev` si besoin et construit l'espace de travail dans
`work/cohan_nav2_ws` — que git ignore. Si un espace de travail deja construit au
meme commit est present sur la machine, il l'adopte au lieu de reconstruire :
`--adopt PATH` le choisit explicitement.

## Pre-requis

- ROS 2 Humble avec `nav2` sur l'hote.
- `libopenblas-dev` : necessaire ; `install` s'en charge.

  Sans elle, `controller_server` refuse de charger le plugin HATEB :

  ```
  Failed to load library .../libhateb_local_planner_core.so
  dlopen error: libopenblas.so.0: cannot open shared object file
  ```

## Configuration

`cohan_robotsnap_params.yaml` est une copie de
`cohan2_params_pr2.yaml`, adaptee a RobotSNAP sur quatre points :

| parametre | valeur | pourquoi |
| --------- | ------ | -------- |
| `robot_base_frame`, `base_frame`, `base_frame_id`, `footprint_frame` | `chassis_link` | RobotSNAP nomme le corps `chassis_link` ; les configs d'origine attendent `base_link` / `base_footprint` |
| `robot.max_vel_y`, `robot.acc_lim_y` | `0.0` | CoHAN-Nav2 traite `max_vel_y == 0` comme non-holonome. Le robot de RobotSNAP est differentiel : `RobotInputController.OnRosCommandReceived` ne lit que `linear.x` et `angular.z`, et jette `linear.y`. En holonome le planificateur commande un lateral que le robot ne peut pas executer |
| `global_frame` | `map` | inchange, RobotSNAP publie deja `map` |
| observation laser | `/base_scan_filtered` | inchange : `cohan_layers/agent_filter.py` le fabrique a partir de `/scan`, que RobotSNAP publie |

## Lancement

```bash
robotsnap-baseline run cohan_nav2
tmux attach -t cohan
```

Six fenetres : `unity`, `bridge`, `cohan` (la pile nav2), `agents` (le pont des
agents, l'horloge et les nœuds CoHAN), `rviz`, `goal`.

`./run.sh --stop` arrete la session.

## Ce que fait chaque morceau

**`agents_bridge.py`** : CoHAN-Nav2 lit les pietons sur `/tracked_agents`
(`cohan_msgs/TrackedAgents`). Son propre `agents_bridge.py` les reconstruit a
partir d'une odometrie par humain (`/humanN/base_pose_ground_truth`), que
RobotSNAP ne publie pas : RobotSNAP envoie *tous* les agents d'un coup, en JSON,
dans le repere du robot. Notre nœud applique la pose de `/odom` - la meme
transformation que `agents_adapter.py` dans `../brne/` et `../hateb/` - et publie
un `TrackedAgent` de type `HUMAN` par agent, avec un segment `TORSO`, comme le
nœud d'origine.

**`clock_relay.py`** : la pile tourne en `use_sim_time`. RobotSNAP n'emet aucun
flux d'horloge ; son odometrie porte deja l'heure du simulateur, et le relais la
republie sur `/clock`. Identique a celui de `../hateb/`.

**Pas de `frames_alias.py`** ici, contrairement a `../hateb/` : en ROS 2 pur,
`/tf_static` circule normalement. Il suffit de nommer `chassis_link` dans les
parametres plutot que d'ajouter des reperes.

**Pas de `localization_launch.py`** : il demarre un `map_server` qui publierait
sa propre carte sur `/map` par-dessus celle de RobotSNAP, et un
`static_transform_publisher` qui publierait un `map -> odom` statique en
concurrence du vrai. `run.sh` ne lance que `navigation_launch.py`.

## Topics

| cote  | topic                | type                                |
| ----- | -------------------- | ----------------------------------- |
| CoHAN | `/tracked_agents`    | `cohan_msgs/TrackedAgents` (ecoute) |
| CoHAN | `/cmd_vel`           | `geometry_msgs/Twist` (publie)      |
| CoHAN | `navigate_to_pose`   | action nav2 (ecoute)                |
| ROBOT | `/odom`,`/scan`,`/map` | odometrie, laser, grille          |
| ROBOT | `/simulation/agents` | `std_msgs/String` JSON              |

Le but se donne par l'**action** `navigate_to_pose`, pas par un topic : c'est du
nav2. D'ou `send_goal.py` :

```bash
python send_goal.py X Y [YAW_DEG]
```

## Depannage

```bash
ros2 topic echo --once /tracked_agents
ros2 lifecycle get /controller_server      # active ?
ros2 topic hz /cmd_vel
ros2 action list | grep navigate
```

- une fenetre disparait : `remain-on-exit` la garde ouverte, mais si elle est
  fermee, relancez la commande a la main dans le dossier ;
- `controller_server` en `unconfigured` : l'erreur de chargement du plugin est
  dans la fenetre `cohan`.
