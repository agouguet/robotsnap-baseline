# RobotSNAP baselines

Faire tourner des méthodes de navigation sociale d'origine extérieure contre
[RobotSNAP](../robotsnap-unity), sans que le cœur de RobotSNAP ait à connaître
aucune d'elles.

Ce dépôt ne contient pas le simulateur. Il contient les **intégrations** : pour
chaque méthode, les quelques nœuds et fichiers de configuration qui la relient à
RobotSNAP, plus les deux scripts qui l'installent et la lancent. Le simulateur,
le pont et le package Python restent dans leurs dépôts ; ici, on les branche.

## Pourquoi un dépôt à part

RobotSNAP doit se cloner et s'installer sans PyTorch, sans Gymnasium et sans les
images Docker d'aucune méthode. Quelqu'un qui veut seulement brancher sa propre
solution ROS 2 n'a rien à faire de DRL-VO ni de CoHAN-Nav2, et l'inverse est
vrai : une méthode vit à son rythme, change de version, casse parfois. Séparer
les deux garde le cœur stable et les intégrations jetables.

## Ce qui est dedans

Comparer des méthodes de navigation sociale n'a de sens que si elles ne se
ressemblent pas. Trois familles sont représentées, et c'est volontaire : une
méthode qui apprend, une méthode qui optimise, une méthode qui réagit.

| méthode | famille | ce que c'est | comment elle tourne |
| ------- | ------- | ------------ | ------------------- |
| `cohan_nav2` | apprentissage | [CoHAN-Nav2](https://github.com/sphanit/CoHAN-Nav2), la version ROS 2 / nav2 de CoHAN, HATEB comme planificateur local | un espace de travail `colcon` construit sur la machine, sans conteneur |
| `drl_vo` | apprentissage | [DRL-VO](https://github.com/TempleRAIL/drl_vo_nav) (IEEE T-RO 2023), porté en ROS 2 : même réseau, même *checkpoint* | des nœuds Python sur le pont RobotSNAP, sans conteneur ni `ros1_bridge` |
| `brne` | apprentissage | [BRNE](https://github.com/ShuijingLiu1/BRNE), politique d'évitement de foule pré-entraînée, à partir d'une image | l'image Docker de ses auteurs, non modifiée |
| `cadrl` | apprentissage | [CADRL](https://github.com/mit-acl/cadrl_ros), le réseau de valeur pré-entraîné des auteurs, ROS 1 | l'image Docker des auteurs, non modifiée, derrière un pont ROS 1 ↔ ROS 2 |
| `sarl_star` | apprentissage | [SARL*](https://github.com/LeeKeyu/sarl_star), l'attention sociale pré-entraînée des auteurs, ROS 1 | l'image Docker des auteurs, non modifiée, derrière un pont ROS 1 ↔ ROS 2 |
| `mpc_ebm` | automatique | un contrôleur prédictif (MPC) : il optimise une trajectoire sous contraintes, il n'apprend rien | l'image Docker de ses auteurs, ROS 2 Humble |
| `nav2_dwb` | heuristique | le planificateur local DWB de nav2 : il échantillonne des trajectoires et les note, sans aucun modèle des personnes | les paquets `nav2` de la machine, sans conteneur |

Toutes parlent à la même chose : `robotsnap bridge --ros2`, qui sert Unity sur
le graphe ROS 2. La méthode publie `/cmd_vel`, le robot la suit.

## Ce que veut dire « non modifiée »

Pour une méthode publiée, ce dépôt ne touche pas à son code. Ce qui change de
nom ou de forme entre elle et RobotSNAP est adapté **à côté**, par des nœuds du
répertoire `tools/`, jamais dans son dépôt ni dans son image :

| ce qui diffère | qui le traduit |
| -------------- | -------------- |
| `/odom` s'appelle `/robot_odom` | `tools/odom_relay.py`, `tools/ros1/robotsnap_ros1_adapter.py` |
| la foule est en JSON, dans la frame du robot | `tools/agents_adapter.py`, `tools/ros1/robotsnap_ros1_adapter.py` |
| la méthode est en ROS 1 | `tools/ros1/compose.yaml` (maître, pont, vue ROS 1) |
| `base_link`, `map`, `odom` ne veulent rien dire pareil | `tools/frames_alias.py` |
| le plan, la foule et le but ne s'affichent pas | `tools/scene_markers.py` |

Ainsi une méthode peut changer de version, ou casser, sans que RobotSNAP ni les
autres méthodes s'en aperçoivent.

## Démarrage

Pré-requis : Ubuntu 22.04, ROS 2 Humble, et un espace de travail RobotSNAP déjà
construit (l'application, `robotsnap/src`, `venv_robotSNAP`). Le dépôt est posé
à côté des autres, dans cet espace de travail.

```bash
./robotsnap-baseline doctor            # ce qui manque sur cette machine
./robotsnap-baseline list              # les méthodes disponibles

./robotsnap-baseline install cohan_nav2
./robotsnap-baseline check   cohan_nav2
./robotsnap-baseline run     cohan_nav2

# dans une autre fenêtre, une fois la pile démarrée :
#   tmux attach -t cohan
#   robot => goal :  python tools/send_goal_nav2.py X Y [YAW_DEG]

./robotsnap-baseline stop    cohan_nav2
```

`run` ouvre une session `tmux`, une fenêtre par responsabilité : le simulateur,
le pont, la méthode, la mainmise sur le robot, `rviz`, et une fenêtre prête à
envoyer un but. `run --headless` lance le simulateur en `-batchmode -nographics`.

## Les commandes

Toutes passent par le même point d'entrée, quelle que soit la méthode :

```
list                les méthodes de ce dépôt
install <méthode>   préparer (construire un espace de travail, extraire un modèle, ...)
check <méthode>     vérifier sans lancer de simulation
run <méthode> [...] lancer une session : simulateur, pont et méthode
stop <méthode>      arrêter cette session
doctor              ce qui manque sur cette machine
pins                toutes les versions épinglées
env <méthode>       l'environnement dans lequel la méthode tourne
```

## Architecture

```
robotsnap-baseline       le point d'entrée unique
lib/common.sh            ce que toutes les méthodes partagent
pins.env                 toutes les versions, à un seul endroit
tools/                   les outils partagés (le robot sous ROS, la vue RViz)
    ros1/                ce qu'une méthode ROS 1 a besoin qu'on lui monte autour
methods/<méthode>/
    method.env           ce que la méthode est et ce dont elle a besoin
    install.sh           la préparer
    check.sh             la vérifier sans simulation
    run.sh               la lancer
    ...                  ses nœuds, ses paramètres, son modèle
work/                    les espaces de travail construits (ignoré par git)
campaigns/               les suites de scénarios à rejouer
```

Une méthode ROS 1 ne recopie pas son pont : elle dit quelle image elle est et
quelle commande la lance, et `tools/ros1/compose.yaml` monte autour d'elle le
maître ROS 1, le pont vers ROS 2 et la vue ROS 1 de RobotSNAP. Les mêmes quatre
services servent CADRL et SARL*, et serviront la prochaine.

Une méthode ne partage rien avec une autre : elle a son dossier, son manifeste
et ses trois scripts. Tout ce qui est vrai de toutes les méthodes vit dans
`lib/common.sh` : la création de la session `tmux`, l'environnement ROS, les
fenêtres que chacune reçoit. Ajouter une méthode, c'est écrire un manifeste et
un `run.sh`, pas recopier un lanceur.

Deux choses en particulier sont vraies de toutes les méthodes, et vivent donc à
un seul endroit plutôt que dans chaque `run.sh` :

* **La vue.** `window_rviz` ouvre RViz2 sur la configuration partagée *et* le
  panneau de marqueurs qui y place la foule et le but, que RViz ne sait pas lire
  dans le JSON de RobotSNAP. Sans lui une méthode montrait une carte et un robot,
  et le but restait invisible ; c'était une ligne dans un seul `run.sh`, donc une
  différence entre méthodes et non une propriété du benchmark.
* **Le but.** Un scénario dit où le robot doit aller, mais une méthode n'écoute
  ni le scénario ni l'application : elle écoute une action ou un topic. Le
  manifeste déclare donc son `GOAL_INTERFACE`, et `window_goal` ouvre le panneau
  qui lit `/simulation/state` et lui remet le but du scénario — `tools/
  scenario_goal.py`. Le robot part tout seul au lancement de l'épisode, et le
  shell reste là pour essayer un but que le scénario ne porte pas.
* **Le robot sous ROS.** `set_ros_control.py` ne se contente pas de régler le
  mode de contrôle au démarrage : il le repose à chaque `/reset_done`. Arrêter
  une session détruit ses robots et appliquer un scénario en construit d'autres,
  en mode clavier — sans ce réarmement le mode ROS était une propriété d'un seul
  épisode, et chaque méthode pilotait dans le vide à partir du deuxième. Le
  même raisonnement vaut pour le but : les deux outils surveillent la session
  au lieu de la lire une fois.

Les versions ne sont jamais devinées. `pins.env` tient les commits, les
empreintes et les distributions. `install.sh` les lit, `doctor` les confronte à
ce qui est réellement installé.

## Ajouter une méthode

1. `mkdir methods/ma_methode`, avec un `method.env` (`MODE`, `ROS`, `SUMMARY`, et
   ce qui suit : `WORKSPACE`, `SOURCES`, `APT`, `GOAL_HINT`).
2. Un `run.sh` qui source `../../lib/common.sh` et enchaîne les fenêtres ; voir
   `methods/drl_vo/run.sh`, le plus court des deux.
3. Un `install.sh` et un `check.sh` si la méthode a quelque chose à préparer ou à
   vérifier ; sans eux, `run` se contente de ce qui est déjà là.
4. Une ligne dans le tableau ci-dessus, et un `pins.env` complété.

Une méthode **ROS 1** suit la même recette et une ligne de plus : ses fenêtres
`method` et son `--stop` passent par `ros1_up_command` et `ros1_down_command` de
`lib/common.sh`. Voir `methods/cadrl/run.sh`.

## Licence

MIT. Chaque méthode garde la licence de son amont, rappelée dans son dossier.
