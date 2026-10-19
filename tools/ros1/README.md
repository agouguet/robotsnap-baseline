# tools/ros1

Ce qu'une méthode ROS 1 a besoin qu'on lui monte autour, et rien de plus.

Une méthode ROS 1 ne peut pas rejoindre le graphe ROS 2 de RobotSNAP seul : il
lui faut un maître ROS 1, un pont vers ROS 2, et une vue ROS 1 des topics que
RobotSNAP publie déjà. Ces trois choses sont les mêmes pour toutes les méthodes
ROS 1, donc elles vivent ici, une fois, et non recopiées dans chacune.

## `compose.yaml`

Quatre services, dans cet ordre :

| service | rôle |
| ------- | ---- |
| `ros-master` | le maître ROS 1 (`roscore`) |
| `ros-bridge` | `ros1_bridge` : ROS 1 et ROS 2 se voient |
| `method` | le conteneur de la méthode, lancé par sa propre commande |
| `method-adapters` | la vue ROS 1 de RobotSNAP : `/robot_odom` et la foule |

Tout est sur le réseau de l'hôte : le maître, le pont, la méthode et le pont
RobotSNAP partagent `127.0.0.1`, ce dont `ros1_bridge` a besoin. Le fichier ne
connaît aucune méthode : `method.env`, `METHOD_IMAGE`, `METHOD_LAUNCH`,
`METHOD_ROS_ENV` et `ROBOT` viennent de l'extérieur, posés par le `run.sh` de la
méthode à travers `ros1_up_command` (`lib/common.sh`). Les images du maître et
du pont sont épinglées dans `pins.env`.

`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` est posé sur le pont : Fast DDS préfère la
mémoire partagée, que des conteneurs séparés ne partagent pas, et le graphe se
découvre alors sans qu'aucun message ne passe. C'est la panne la plus trompeuse
de ce montage — `ros2 topic list` montre tout, `ros2 topic hz` ne montre rien.

## `robotsnap_ros1_adapter.py`

```bash
python robotsnap_ros1_adapter.py [--robot robot_1]
```

La vue ROS 1 de RobotSNAP, lancée dans le conteneur de la méthode — le seul
endroit où `agents_msgs` version ROS 1 est construit. Elle met sous la main d'une
méthode ce qu'un robot ROS 1 de l'époque du turtlebot aurait, et rien de plus :

| ce qu'une méthode ROS 1 attend | d'où il vient |
| ------------------------------ | ------------- |
| `/robot_odom` (`nav_msgs/Odometry`) | `/odom`, même message, même horodatage |
| `/social_sim/agents` (`agents_msgs/AgentArray`, frame `map`) | `/simulation/agents` (JSON, frame du robot), par `p_world = p_robot + R(yaw)·p_agent` — la transformation de `tools/agents_adapter.py` |
| `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`) | la pose de `/odom` sous le nom et le type d'AMCL |
| `/move_base/global_costmap/costmap` (`nav_msgs/OccupancyGrid`) | `/map`, même grille |
| `/tf` : `map -> odom -> base_link`, `base_footprint` | l'identité et la pose de `/odom` |

`/tf` est reconstruit ici et non repris de ROS 2 : `tf2_msgs` ne fait pas partie
des types que ce pont connaît, donc `/tf` ne franchit pas. `base_footprint` est
`base_link` sous le second nom que les piles ROS 1 écrivent en dur — le même
office que `tools/frames_alias.py` côté ROS 2. Sans cette chaîne, un nœud qui
fait `waitForTransform('/map', '/base_footprint')` lève, et SARL\* tombe ensuite
sur `position.x: None` au premier but.

Elle ne lit que des types standard — `std_msgs/String`, `nav_msgs/Odometry` — et
c'est ce qui la fait passer par le pont : `agents_msgs/AgentArray` n'est pas
connu du pont générique, donc c'est **ici**, du côté ROS 1, que la conversion a
lieu, et non de l'autre côté.

Le fichier doit rester compatible **Python 2** : les images CADRL et SARL*
embarquent ROS Kinetic, dont `rospy` est un `rospy` Python 2, et un `import
yaml` échoue sous `python3`. Pas de f-strings, pas d'annotations.

Elle ne suppose rien de l'ordre d'arrivée : sans odométrie elle ne publie rien,
un JSON invalide ou une liste vide sont ignorés sans lever.
