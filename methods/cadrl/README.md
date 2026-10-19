# CADRL avec RobotSNAP

Faire tourner [CADRL](https://github.com/mit-acl/cadrl_ros) - le reseau de
valeur pre-entraine pour la navigation de foule de *Collision Avoidance with
Deep Reinforcement Learning* (Chen et al., RA-L 2017) - contre RobotSNAP.

## Pourquoi passer par sa propre image

Le code publie de CADRL est ROS 1 (Kinetic) et attend un simulateur precis : il
lit une odometrie sur `/robot_odom`, la foule sur `/social_sim/agents`, le but
sur `/global_goal`, et publie `/cmd_vel`. Son *checkpoint* est dans son image.

RobotSNAP parle ROS 2. La greffe se fait donc en deux temps, sans toucher a
l'image :

1. `robotsnap bridge --ros2` expose RobotSNAP sur le graphe ROS 2 ;
2. `tools/ros1/compose.yaml` monte, autour de l'image, un maitre ROS 1
   (`roscore`), `ros1_bridge` entre les deux distributions, et une vue ROS 1 de
   RobotSNAP.

C'est le meme montage pour CADRL et SARL*, et il n'est ecrit qu'une fois : la
methode ne dit que son image, son environnement ROS 1 et sa commande de
lancement, dans `method.env`.

## Ce qui est adapte, et ou

| ce qui differe | qui le traduit |
| -------------- | -------------- |
| RobotSNAP publie `/odom`, CADRL attend `/robot_odom` | `tools/ros1/robotsnap_ros1_adapter.py` |
| la foule est en JSON, dans la frame du **robot** ; CADRL lit `agents_msgs/AgentArray` dans la frame du **monde** | `tools/ros1/robotsnap_ros1_adapter.py` |
| `ros1_bridge` ne connait pas `agents_msgs` | l'adaptateur tourne *dans le conteneur de la methode*, ou le message est construit, et ne lit que des types standard (`std_msgs/String`, `nav_msgs/Odometry`) que le pont sait faire passer |
| le robot ne lit `/cmd_vel` que sous ROS | `tools/set_ros_control.py`, appele par la fenetre `control` |

Rien de l'amont n'est modifie : ni les sources, ni l'image, ni le checkpoint
(`checkpoints/network_01900000`).

## Lancement

```bash
robotsnap-baseline install cadrl       # verifie les images (CADRL, maitre, pont)
robotsnap-baseline check   cadrl       # verifie sans lancer de simulation
robotsnap-baseline run     cadrl
tmux attach -t cadrl
```

Six fenetres : `unity`, `bridge`, `method` (toute la face ROS 1 : `roscore`, le
pont, CADRL et l'adaptateur), `control` (passe le robot sous ROS), `rviz` (la vue
partagee `../../tools/scene.rviz`), `goal`.

Le but s'envoie en ROS 2 ; `ros1_bridge` le fait passer en ROS 1 :

```bash
python tools/send_goal.py X Y [YAW_DEG]
```

`X` et `Y` sont dans le repere de la carte, en metres. Options : `--headless`
(application sans fenetre), `--robot NAME` (le robot conduit), `--kill`,
`--stop`.

## Topics

| cote | topic | type |
| ---- | ----- | ---- |
| RobotSNAP (ROS 2) | `/odom`, `/scan` | odometrie, laser |
| RobotSNAP (ROS 2) | `/simulation/agents` | `std_msgs/String`, JSON, frame robot |
| adaptateur (ROS 1) | `/robot_odom` | `nav_msgs/Odometry` (ecoute) |
| adaptateur (ROS 1) | `/social_sim/agents` | `agents_msgs/AgentArray` (ecoute) |
| CADRL | `/global_goal` | `geometry_msgs/PoseStamped` (ecoute) |
| CADRL | `/cmd_vel` | `geometry_msgs/Twist` (publie) |

## Arret

```bash
robotsnap-baseline stop cadrl
```

`--stop` tue la session `tmux` **et** fait `docker compose down` : tuer le client
compose ne suffit pas a arreter les quatre services, qui continueraient de tenir
les ports du prochain lancement.

## Depannage

- **Le robot ne bouge pas.** Le mode de controle doit etre `ROS` : la fenetre
  `control` s'en charge et reessaie jusqu'a ce que le simulateur accepte.
- **CADRL ne voit personne.** Verifiez `/social_sim/agents` dans le conteneur, et
  que le pont tourne bien.
- **Rien ne franchit le pont.** `ros1_bridge` ne passe que les types qu'il
  connait ; d'ou l'adaptateur, qui n'utilise que `std_msgs/String` et
  `nav_msgs/Odometry`.

## Licence

L'image et le code de CADRL gardent la licence de leur amont
([mit-acl/cadrl_ros](https://github.com/mit-acl/cadrl_ros)).
