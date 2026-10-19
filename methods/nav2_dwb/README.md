# nav2 + DWB

Le bout classique de la comparaison : la pile `nav2` de `nav2_bringup`, son
planificateur global et son planificateur local DWB, sans modele de personne.

```bash
./robotsnap-baseline run nav2_dwb
# dans la fenetre `goal` :
#   python tools/send_goal_nav2.py X Y [YAW_DEG]
```

## Ce qui a ete corrige

La methode a d'abord ete rapportee comme « ne fonctionne pas ». Le robot
atteignait la zone du but, puis l'action `navigate_to_pose` ne revenait jamais
et `controller_server` repetait `Failed to make progress`.

Trois defauts ont ete trouves puis corriges, dans cet ordre :

1. **Un second `/clock`.** La methode lancait `tools/clock_relay.py` alors que la
   passerelle RobotSNAP publie deja l'horloge simulee. Deux ecrivains sur le
   meme topic faisaient reculer le temps a chaque message, et chaque noeud de la
   pile vidait son tampon TF (`Detected jump back in time`). Le relais a ete
   retire ; la fenetre `nav2` n'a plus qu'un pane.
2. **La pile demarrait avant le robot.** `navigation_launch.py` etait lance sans
   qu'aucun robot existe : le *player* demarre en standby et n'applique le
   scenario que sur demande. Les noeuds s'activaient, ne trouvaient aucune
   transformation `map -> base_link` et rien ne le disait. Deux outils partages
   ont ete ajoutes - `tools/play_scenario.py` demarre l'episode, et
   `tools/wait_for_robot.py` retient la pile jusqu'a ce que les frames du robot
   se resolvent.
3. **L'arrivee exigeait un cap que la scene ne demande pas.** Voir ci-dessous.

## La limite connue : le robot tourne mal a basse vitesse

Le troisieme defaut n'etait pas dans la pile, il etait dans les parametres.
L'exemple de `nav2_bringup` fait d'un but une *pose* : il faut arriver sur le
point **et** sur le cap. Un scenario RobotSNAP ne donne pas de cap
(`robot_1_goal: yaw: null`), et l'arrivee du simulateur est purement
positionnelle (`Robot.WatchManualArrival`, seuil `0.2 m`). La pile demandait donc
au robot quelque chose que la mission ne demandait pas - et le robot ne pouvait
pas le lui donner : le châssis du Jackal, tel qu'il est modele, **tourne
beaucoup moins vite qu'on le lui demande en rotation sur place**.

Mesure sur le build 0.1.0, consigne `angular.z` tenue dix secondes, robot seul
sur un sol libre :

| consigne (rad/s) | rotation obtenue |
| ---------------- | ---------------- |
| 0.16 | aucune |
| 0.18 | aucune |
| 0.21 | ~ 12 % |
| 0.25 | ~ 37 % |
| 0.30 | ~ 57 % |
| 0.60 | ~ 81 % |

Sous environ 0.2 rad/s, la consigne ne produit **rien**. Au-dessus, le robot
atteint une fraction croissante de ce qui est demande. La translation, elle, est
fidele : une consigne de 0.26 m/s deplace le robot a 0.26 m/s.

C'est un ecart du modele physique du Jackal, pas de cette methode : le Jackal est
le seul prefab en `driveModel: 2` (modele de pneus, `tyreRelaxationSpeed: 0.4`,
`tyreLongitudinalSlip: 0.08`) ; Freight, Kuri et Bibus sont en
`driveModel: 1` (base pilotee en vitesse). Le modele de pneus ne produit pas le
couple de lacet necessaire a l'equilibre en rotation sur place, ou la resistance
au ripage laterale n'est plus compensee. Toute politique qui commande moins de
0.2 rad/s - donc a peu pres toutes, a l'approche d'un but - verra le meme ecart.

Le corriger demande de reprendre `Assets/Scripts/RobotSNAP/Agent/Robot/
ArticulationWheelController.cs` (modele de pneus) ou de passer le prefab Jackal
en `driveModel: 1`, puis de reconstruire l'application et sa distribution.

## Ce que la methode fait de cette limite

En attendant, les parametres de la methode sont alignes sur ce que le robot sait
faire, et non sur l'inverse. Les quatre ecarts a l'exemple de `nav2_bringup` sont
commentes dans `nav2_dwb_params.yaml` :

| parametre | exemple nav2 | ici | pourquoi |
| --- | --- | --- | --- |
| `general_goal_checker.yaw_goal_tolerance` | `0.25` | `3.15` | le scenario ne donne pas de cap ; le simulateur compte l'arrivee sur la position seule |
| `general_goal_checker.xy_goal_tolerance` | `0.25` | `0.2` | la meme distance que `Robot.ArrivalDistanceMetres`, pour que le but atteint soit aussi celui que les metriques lisent |
| `progress_checker.movement_time_allowance` | `10.0` | `20.0` | la derniere partie d'un alignement de skid-steer ne couvre pas de terrain ; dix secondes la lisaient comme un robot bloque |
| `min_speed_xy` / `min_speed_theta` | `0.0` / `0.0` | `0.05` / `0.25` | lu par le validateur de nav2 comme une paire : DWB cesse d'echantillonner les rotations que le châssis ne peut pas executer |

Verifie de bout en bout sur le build 0.1.0 : le robot part de son point de depart
(`x = 0, y = 7` en ROS), rejoint `(2.0, 0.0)` en 25 s environ, s'arrete a 0.16 m
du but - dans le seuil d'arrivee de l'application - et l'action
`navigate_to_pose` revient `SUCCEEDED`.
