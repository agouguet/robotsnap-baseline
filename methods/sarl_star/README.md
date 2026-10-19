# SARL* avec RobotSNAP

Faire tourner [SARL*](https://github.com/LeeKeyu/sarl_star) - le reseau a
attention sociale pre-entraine de *Socially Attentive Reinforcement Learning*
(Chen et al., RA-L 2019) - contre RobotSNAP.

## Pourquoi passer par sa propre image

Le code publie de SARL* est ROS 1 (Kinetic) et attend un simulateur precis : il
lit une odometrie sur `/robot_odom`, la foule sur `/social_sim/agents`, le but
sur `/global_goal`, et publie `/cmd_vel`. Son *checkpoint* est dans son image.

RobotSNAP parle ROS 2. La greffe se fait donc en deux temps, sans toucher a
l'image :

1. `robotsnap bridge --ros2` expose RobotSNAP sur le graphe ROS 2 ;
2. `tools/ros1/compose.yaml` monte, autour de l'image, un maitre ROS 1
   (`roscore`), `ros1_bridge` entre les deux distributions, et une vue ROS 1 de
   RobotSNAP (`/robot_odom`, `/social_sim/agents`, `/amcl_pose`, le costmap
   global et `/tf`).

C'est le meme montage que CADRL, et il n'est ecrit qu'une fois : la methode ne
dira que son image, son environnement ROS 1 et sa commande de lancement.

## La commande de lancement : `rosrun`, pas `navigation.launch`

`navigation.launch` amont ne convient pas telle quelle. A cote du planificateur
et de `agents_to_people.py`, il inclut
`turtlebot_navigation/launch/includes/hokuyo_move_base.launch.xml`, qui demarre
`map_server`, le costmap global, le costmap local, `move_base`, le lisseur de
vitesse, le controleur de securite et un laser. RobotSNAP n'a ni turtlebot, ni
laser ROS 1, ni `base_footprint` : cet include ne peut pas se configurer.

Et ce n'est pas necessaire. `sarl_star_node.py` **planifie sa propre
vitesse** : il construit un `JointState`, appelle `policy.predict(state)`, et
publie le `Twist` obtenu. Il ne demande aucun chemin a `move_base` et n'appelle
aucun service ni action de navigation. La commande retenue est donc le strict
necessaire, la meme que le launch amont pour le planificateur et son helper -
les deux dans un seul shell, pour que les deux heritent de l'environnement ROS
que le conteneur source avant lui :

```
rosrun sarl_star_ros agents_to_people.py agents:=social_sim/agents
    & rosrun sarl_star_ros sarl_star_node.py
        odom:=robot_odom local_goal:=global_goal cmd_vel_mux/input/teleop:=cmd_vel
```

Les trois remaps sont ceux de `navigation.launch`, passes en arguments `rosrun`
pour ne modifier aucun fichier de l'amont. `agents_to_people.py` est l'autre
noeud du launch : rien d'autre ne transforme `/social_sim/agents` en `/people`.

### Ce que le noeud attend, et qui le lui fournit

Preuve dans `scripts/sarl_star_node.py` (image `sarl_star`) : le noeud s'abonne
a

| topic | usage | qui le publie ici |
| ----- | ----- | ----------------- |
| `/amcl_pose` | sa propre pose `px, py, theta` | `tools/ros1/robotsnap_ros1_adapter.py`, a partir de `/odom` |
| `/odom` | sa vitesse `vx, vy` | RobotSNAP, par le pont |
| `/people` | la foule | `agents_to_people.py`, a partir de `/social_sim/agents` |
| `/local_goal` (remappe `/global_goal`) | le but | l'operateur, par le pont |
| `/move_base/global_costmap/costmap` | l'environnement statique | `tools/ros1/robotsnap_ros1_adapter.py`, a partir de `/map` |
| `/tf` (`map` -> `odom` -> `base_link`, `base_footprint`) | ses conversions de repere | `tools/ros1/robotsnap_ros1_adapter.py` |

et publie `/cmd_vel_mux/input/teleop` (remappe `/cmd_vel`).

La boucle principale ne planifie que lorsque `Is_lg_Received`, `IsAMCLReceived`,
`IsObReceived` et `Is_gc_Received` sont tous vrais. Les quatre le deviennent :
la pose vient de l'adaptateur, la foule de `agents_to_people.py`, le but du
pont, et le costmap global de la grille `/map` de RobotSNAP. `base_footprint`
est publie comme `base_link` sous son second nom, comme le fait deja
`tools/frames_alias.py` cote ROS 2.

Deux pieges a connaitre, tous les deux dans l'amont et non corriges ici :
`AMCL` est **commentee dans `navigation.launch`** - meme la commande d'origine
ne publie pas `/amcl_pose` - et `visualize_goal()` publie `self.gx` sans
verifier qu'il est defini, donc un but qui arrive avant le TF fait tomber le
noeud sur `position.x: None`. C'est precisement pourquoi la chaine de reperes
est publiee des le premier message d'odometrie.

## Ce qui est adapte, et ou

| ce qui differe | qui le traduit |
| -------------- | -------------- |
| RobotSNAP publie `/odom`, SARL* attend `/robot_odom` | `tools/ros1/robotsnap_ros1_adapter.py` |
| la foule est en JSON, dans la frame du **robot** ; SARL* lit `agents_msgs/AgentArray` dans la frame du **monde** | `tools/ros1/robotsnap_ros1_adapter.py` |
| `ros1_bridge` ne connait pas `agents_msgs` | l'adaptateur tourne *dans le conteneur de la methode*, ou le message est construit, et ne lit que des types standard (`std_msgs/String`, `nav_msgs/Odometry`) que le pont sait faire passer |
| le robot ne lit `/cmd_vel` que sous ROS | `tools/set_ros_control.py`, appele par la fenetre `control` |

Rien de l'amont n'est modifie : ni les sources, ni l'image, ni le *checkpoint*
(`CrowdNav/crowd_nav/data/output/rl_model.pth`).

## Lancement

```bash
robotsnap-baseline install sarl_star   # verifie les images (SARL*, maitre, pont)
robotsnap-baseline check   sarl_star   # verifie sans lancer de simulation
robotsnap-baseline run     sarl_star
tmux attach -t sarl_star
```

Six fenetres : `unity`, `bridge`, `method` (toute la face ROS 1 : `roscore`, le
pont, SARL* et l'adaptateur), `control` (passe le robot sous ROS), `rviz` (la vue
partagee `../../tools/scene.rviz`), `goal`.

Le but s'envoie en ROS 2 ; `ros1_bridge` le fait passer en ROS 1 :

```bash
python tools/send_goal.py X Y [YAW_DEG]
```

`X` et `Y` sont dans le repère de la carte, en metres. Options : `--headless`
(application sans fenetre), `--robot NAME` (le robot conduit), `--kill`,
`--stop`.

## Topics

| cote | topic | type |
| ---- | ----- | ---- |
| RobotSNAP (ROS 2) | `/odom`, `/scan`, `/map` | odometrie, laser, grille |
| RobotSNAP (ROS 2) | `/simulation/agents` | `std_msgs/String`, JSON, frame robot |
| adaptateur (ROS 1) | `/robot_odom`, `/amcl_pose`, `/tf` | odometrie, pose, reperes |
| adaptateur (ROS 1) | `/social_sim/agents`, `/move_base/global_costmap/costmap` | foule, grille |
| SARL* | `/global_goal` | `geometry_msgs/PoseStamped` (ecoute) |
| SARL* | `/cmd_vel` | `geometry_msgs/Twist` (publie) |

## Arret

```bash
robotsnap-baseline stop sarl_star
```

`--stop` tue la session `tmux` **et** fait `docker compose down` : tuer le
client compose ne suffit pas a arreter les quatre services, qui continueraient
de tenir les ports du prochain lancement.

## Depannage

- **Le robot ne bouge pas.** Le mode de controle doit etre `ROS` (la fenetre
  `control` s'en charge). Si le noeud tourne mais ne commande rien, verifiez que
  les quatre entrees de la porte sont bien la : `/amcl_pose`,
  `/move_base/global_costmap/costmap`, `/people` et `/global_goal`.
- **`TransformException: base_footprint does not exist`.** L'adaptateur ROS 1
  n'est pas lance : c'est lui qui publie `map -> odom -> base_link`, et
  `base_footprint` sous le meme nom.
- **SARL* ne voit personne.** Verifiez `/social_sim/agents` puis `/people` dans
  le conteneur, et que le pont tourne bien.
- **Rien ne franchit le pont.** `ros1_bridge` ne passe que les types qu'il
  connait ; d'ou l'adaptateur, qui n'utilise que `std_msgs/String` et
  `nav_msgs/Odometry`.

## Licence

L'image et le code de SARL* gardent la licence de leur amont
([LeeKeyu/sarl_star](https://github.com/LeeKeyu/sarl_star)).
