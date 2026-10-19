# DRL-VO avec RobotSNAP

Faire tourner
[DRL-VO](https://github.com/TempleRAIL/drl_vo_nav) - la politique de navigation
sociale du papier *DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes
Using Velocity Obstacles* (IEEE T-RO 2023) - contre RobotSNAP.

## Pourquoi une version ROS 2

Le code publie de DRL-VO est ROS 1. On l'utilisait jusqu'ici par son image
Docker, derriere un `ros1_bridge` : trois conteneurs, un pont entre deux
distributions, et les problemes que ce pont introduit (`/tf_static` qui ne
franchit pas la frontiere, des reperes prefixes d'un `/` que tf2 refuse).

Ici, la politique et le constructeur d'observation sont repris tels quels - meme
checkpoint, meme arithmetique - mais cables en noeuds ROS 2 sur la machine hote.
Le robot est conduit par `robotsnap bridge --ros2` : pas de conteneur, pas de
pont, un seul graphe.

## Ce qui est repris, et ce qui change

Le reseau, les entrees et la commande sont ceux du papier :

| element | valeur |
| ------- | ------ |
| modele | `model/drl_vo.zip`, le PPO publie (stable-baselines3 0.10) |
| observation | `[carte pietons 12800][historique laser 7200][sous-but 2]` |
| carte pietons | 2 x 80 x 80 cases de 0,25 m, x jusqu'a 20 m, y sur +-10 m |
| laser | 720 rayons sur -90..+90 degres, historique de 10 trames a 20 Hz |
| marge de but | 0,9 m (le robot s'arrete) |
| marge d'obstacle | 0,4 m sur les +-45 degres avant, puis pivot a 0,7 rad/s |
| sorties | avant 0..0,5 m/s, lacet -2..2 rad/s |

Quatre points sont adaptes, chacun note dans le code :

- **Le lidar est remis a l'echelle.** Le modele a vu un capteur de 1080 rayons
  sur 270 degres, a 0,25 degre. `drl_vo_lidar.py` reechantillonne ce que
  RobotSNAP publie sur cette meme grille, donc n'importe quel profil de robot
  convient.
- **Un rayon sans echo vaut 30 m.** Le code ROS 1 transformait un `inf` en 0, ce
  qui se lit comme un obstacle colle au pare-chocs. Avec un capteur plus court
  que les 30 m du modele, cela figerait le robot ; ici un non-echo est le bout
  lointain de l'echelle, ce que le reseau attend d'un espace libre.
- **Le sous-but vient d'une ligne droite.** Le code publie le prend sur le plan
  global de `move_base`, par `pure_pursuit.py`. Les missions de RobotSNAP ont un
  seul but, dont le plan global est la ligne droite : `drl_vo_subgoal.py`
  calcule directement le point a 2 m devant, qui est le meme point.
- **La BatchNorm tourne comme dans la publication.** `stable_baselines3` 1.1 ne
  passait jamais la politique en mode inference : le deploiement publie
  normalisait donc avec les statistiques de l'echantillon courant. Le defaut
  (`--batchnorm reference`) reproduit exactement ce comportement, verifie
  chiffre par chiffre contre l'image d'origine ; `--batchnorm running` utilise
  les statistiques accumulees, ce que la couche etait censee faire.

## Verification

`robotsnap-baseline check drl_vo` verifie chaque piece sans lancer de simulation,
et compare la reponse du checkpoint a celle de l'implementation publiee. Cette
reference a ete obtenue en faisant tourner `drl_vo_inference.py` dans l'image
Docker sur la meme entree : les deux reponses coincident au chiffre pres.

Une session reelle a aussi ete deroulee : application lancee, scenario `default`
(`robot_1`, Jackal, 5 pietons), nœuds deposes sur le graphe, puis un but envoye
depuis Python. Le robot a parcouru la distance jusqu'a 0,8 m du but et s'y est
arrete - la marge de 0,9 m du papier.

## Lancement

```bash
robotsnap-baseline install drl_vo       # verifie le checkpoint et l'interpreteur
robotsnap-baseline run     drl_vo
tmux attach -t drl_vo
```

Six fenetres : `unity`, `bridge`, `drl_vo(2)` (l'observation et la politique),
`control` (passe le robot sous ROS), `rviz(2)` et `goal`. La fenetre `rviz` a
deux vues empilees : la vue partagee de toutes les baselines
(`../../tools/scene.rviz`), et les marqueurs de `../../tools/scene_markers.py`
qui y placent les pietons et le but. Les deux viennent maintenant de
`window_rviz` dans `lib/common.sh`, comme pour toutes les methodes : cette
integration lancait le nœud de marqueurs elle-meme, et c'est pour cela qu'elle
etait la seule a montrer la foule et le but.

La fenetre `goal` n'a ici qu'un pane, et c'est voulu : l'observation lit le but
du scenario directement dans `/simulation/state`, donc le robot part des le
debut de l'episode et il n'y a rien qu'un second expediteur pourrait ajouter.
Les methodes qui ecoutent une action ou un topic, elles, recoivent un second
pane qui leur remet ce meme but (`../../tools/scenario_goal.py`).

La vue est la meme pour toutes les methodes, a dessein : deux methodes qui
apportent chacune la leur obligent a comparer des images qui ne montrent pas la
meme chose. Ses affichages sont ranges en dossiers — `RobotSNAP`, `Crowd`,
`Planner`, `Costmaps` — et chaque methode allume ce qu'elle publie. Sans ce
fichier, `rviz2` rechargerait `~/.rviz2/default.rviz`, souvent regle pour un
autre montage, et afficherait un ecran vide avec une frame fixe qui n'existe pas
ici.

Puis, dans la fenetre `goal` ou depuis n'importe quel terminal :

```bash
python send_goal.py X Y [YAW_DEG] --robot robot_1
```

`X` et `Y` sont dans le repere ROS, celui de RViz2 : x vers l'avant, y vers la
gauche, en metres, sur la carte. Le but est transmis a Unity, donc l'objectif
affiche, celui que les metriques evaluent et celui que la politique suit sont le
meme.

Options : `--headless` (application sans fenetre), `--robot NAME` (le robot
conduit), `--stop` (arrete la session).

## Topics

| cote | topic | type |
| ---- | ----- | ---- |
| RobotSNAP | `/scan`, `/odom`, `/map`, `/tf` | laser, odometrie, grille, reperes |
| RobotSNAP | `/simulation/agents` | `std_msgs/String`, JSON dans le repere robot |
| RobotSNAP | `/simulation/state` | `std_msgs/String`, JSON, but et humains |
| DRL-VO | `/drl_vo/observation` | `std_msgs/Float64MultiArray` (19202 flottants) |
| DRL-VO | `/cmd_vel` | `geometry_msgs/Twist` (publie) |

## Depannage

- **Le robot ne bouge pas.** Le mode de controle doit etre `ROS`. La fenetre
  `control` s'en charge et reessaie jusqu'a ce que le simulateur accepte, parce
  que la demande arrive souvent avant le robot. A la main :
  `python set_ros_control.py --robot robot_1`.
- **Rien n'arrive dans le graphe.** Le bridge doit tourner avec `--ros2`, et
  Unity doit s'y connecter.
- **Les nœuds ne rejoignent pas le graphe.** Le transport memoire partagee de
  cette machine refuse d'ouvrir ses ports ; `run.sh` fixe
  `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, a garder si vous lancez les nœuds a la
  main.
- **La politique tourne mais le robot reste immobile.** Verifiez `/cmd_vel` :
  `ros2 topic info /cmd_vel -v` doit annoncer un abonne. Sans abonne, le bridge
  n'a pas encore vu Unity s'enregistrer ; il le fait dans la seconde.
- **Le checkpoint ne se charge pas.** Il date de 2021 : `drl_vo_network.py`
  reconstruit les tetes a la main et `vendor/gym` fournit le `gym` de l'epoque.
  Rien d'autre que `torch` et `gymnasium` n'est requis.

## Fichiers

| fichier | role |
| ------- | ---- |
| `drl_vo_common.py` | noms de topics et petites conversions |
| `drl_vo_lidar.py` | reechantillonnage du laser sur la grille du modele |
| `drl_vo_agents.py` | carte de vitesse des pietons, dans le repere robot |
| `drl_vo_subgoal.py` | le sous-but a 2 m sur la ligne du but |
| `drl_vo_network.py` | chargement et execution du checkpoint |
| `drl_vo_command.py` | de l'observation a la commande, sans ROS |
| `drl_vo_observation.py` | noeud : construit l'observation |
| `drl_vo_policy.py` | noeud : politique et `/cmd_vel` |
| `send_goal.py` | envoie un but |
| `selftest.py` | verifications hors simulation |

Le passage du robot sous ROS n'est pas ici : c'est `../../tools/set_ros_control.py`,
que la fenetre `control` appelle, et que partage toute methode ROS 2.
