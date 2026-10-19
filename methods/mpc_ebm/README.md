# MPC + EBM avec RobotSNAP

Faire tourner l'image `agouguet/benchmark-social-navigation:mpc_ebm` - le
controleur predictif (MPC) couple a une prediction de mouvement par energie
(EBM, *energy-based motion prediction*) - contre RobotSNAP.

C'est la borne "automatique" du banc : aucune politique apprise, aucun reseau de
valeur. Un solveur d'horizon glissant poursuit un chemin, en evitant les
obstacles et, quand le modele EBM est disponible, les humains tels qu'il les
prevoit. C'est ce qui la rend utile face aux baselines IA : elle ne partage ni
la forme d'observation ni la fonction de recompense des methodes apprises.

## Pourquoi la greffe est courte

L'image a ete construite pour ce banc. Elle contient deux familles de lancements :

* `zmr_demo/single_robot_mpc_rviz` et les `*_gazebo` - des demos **autonomes**,
  qui embarquent leur propre modele cinematique (`zmr_drive_model`) ou Gazebo.
  Branchees sur RobotSNAP, elles simuleraient le robot deux fois : inutilisables
  ici.
* `sbrn_demo/demo_warehouse_single_robot_gazebo_mpc_mmp.launch.py` - malgre son
  nom, **aucun Gazebo**. C'est la variante "pont" : elle lit une odometrie
  externe et la foule, et publie une vitesse. C'est elle qui est lancee.

Ce lancement demarre six nœuds. Trois interessent la greffe :

| nœud | lit | ecrit |
| ---- | --- | ----- |
| `bridge_mpc_trajectory_tracker_node` | `/robot_odom`, `/motion_prediction_result`, `/new_scenario`, services `/get_inflated_map` et `/get_robot_schedule` | `/cmd_vel`, `/robot_states`, `/robot_0_schedule_viz` |
| `bridge_unity_actor_traj_node` | `/social_sim/agents` | `/human_traj_array` |
| `bridge_geometric_map_node` et `bridge_global_path_coordinate_node` | `/social_sim/scene_info`, `/new_scenario` | la carte et le planning, servis par les deux services ci-dessus |

## Ce qui est adapte, et ou

| ce qui differe | qui le traduit |
| -------------- | -------------- |
| RobotSNAP publie `/odom`, le MPC attend `/robot_odom` | `tools/odom_relay.py` |
| la foule est en JSON frame **robot** ; le MPC lit `agents_msgs/AgentArray` frame **monde** | `tools/agents_adapter.py` |
| le MPC attend `/social_sim/scene_info` (nom de carte) et `/new_scenario`, que RobotSNAP ne publie pas | le pane `scene` de `run.sh` |
| le robot ne lit `/cmd_vel` que sous ROS | `tools/set_ros_control.py`, appele par la fenetre `control` |

Rien de l'amont n'est modifie : ni les sources, ni l'image, ni le modele.

## Le but n'est pas un topic

Point important, et different de CADRL/SARL* : le MPC **ne lit pas de but**. Il
suit le chemin decrit par `assets/maps/<scene>/schedule.csv`, resolu sur
`graph.json`, et le but est le **dernier nœud de ce chemin**. Pour changer de
but il faut mettre a jour la carte et le planning de l'image, ou choisir un
autre `--scene`. Changer de `--scene` ne deplace pas le but dans RobotSNAP.

L'autre consequence : le MPC planifie **dans les coordonnees de sa carte**, en
metres. Il n'a de sens que si le scenario joue par RobotSNAP est celui du banc
(meme geometrie, meme origine). La carte se choisit avec `--scene` :

```bash
robotsnap-baseline run mpc_ebm --scene intersection
```

Les six cartes livrees sont `corner`, `crowd`, `frontal`, `intersection`,
`narrow_passage`, `perpendicular_traffic` - les memes scenes que le jeu de
donnees de navigation sociale.

## Lancement

```bash
robotsnap-baseline install mpc_ebm       # verifie l'image
robotsnap-baseline check   mpc_ebm       # verifie sans lancer de simulation
robotsnap-baseline run     mpc_ebm       # carte 'crowd' par defaut
tmux attach -t mpc_ebm
```

Six fenetres : `unity`, `bridge`, `method` (le conteneur et les trois
adaptateurs), `control`, `rviz` (`../../tools/scene.rviz`), `goal`.

Options : `--headless` (application sans fenetre), `--scene NAME`,
`--robot NAME` (seul `robot_1` est conduit : l'image fixe les noms de topics
nus), `--kill`, `--stop`, `--print-method`.

Pour arreter proprement l'application et le conteneur :

```bash
robotsnap-baseline stop mpc_ebm
```

## Limites connues

* **L'EBM a besoin d'un GPU.** Au demarrage, le nœud de prediction
  (`motion_predict_node`) appelle `torch.cuda.is_available()` et se termine
  proprement s'il n'y a pas de CUDA. Sans GPU le MPC tourne toujours et publie
  `/cmd_vel`, mais sans la prediction EBM : il evite alors les humains comme des
  obstacles statiques. Avec GPU, le nœud reste vivant et publie
  `/motion_prediction_result`.
* **La carte doit correspondre.** Le MPC navigue dans la carte du banc ; un
  scenario RobotSNAP dont la geometrie differe ne donnera pas un comportement
  coherent, meme avec le bon `--scene`.
* **Un seul robot.** Le nœud fixe `/robot_odom` et `/cmd_vel`. Conduire
  `robot_2` demanderait un alias de topics que cette methode ne fournit pas.
* **L'horloge est systeme.** Ces nœuds ne lisent pas l'horloge simulee ; avec le
  time-scale de RobotSNAP, le MPC avancera plus lentement que le monde.
