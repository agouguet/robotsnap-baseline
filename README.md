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

| méthode | ce que c'est | comment elle tourne |
| ------- | ------------ | ------------------- |
| `cohan_nav2` | [CoHAN-Nav2](https://github.com/sphanit/CoHAN-Nav2), la version ROS 2 / nav2 de CoHAN, HATEB comme planificateur local | un espace de travail `colcon` construit sur la machine, sans conteneur |
| `drl_vo` | [DRL-VO](https://github.com/TempleRAIL/drl_vo_nav) (IEEE T-RO 2023), porté en ROS 2 : même réseau, même *checkpoint* | des nœuds Python sur le pont RobotSNAP, sans conteneur ni `ros1_bridge` |

Les deux parlent à la même chose : `robotsnap bridge --ros2`, qui sert Unity sur
le graphe ROS 2. La méthode publie `/cmd_vel`, le robot la suit.

## Démarrage

Pré-requis : Ubuntu 22.04, ROS 2 Humble, et un espace de travail RobotSNAP déjà
construit (l'application, `robotsnap/src`, `venv_robotSNAP`). Le dépôt est posé
à côté des autres, dans cet espace de travail.

```bash
./bin/robotsnap-baseline doctor            # ce qui manque sur cette machine
./bin/robotsnap-baseline list              # les méthodes disponibles

./bin/robotsnap-baseline install cohan_nav2
./bin/robotsnap-baseline check   cohan_nav2
./bin/robotsnap-baseline run     cohan_nav2

# dans une autre fenêtre, une fois la pile démarrée :
#   tmux attach -t cohan
#   robot => goal :  python send_goal.py X Y [YAW_DEG]

./bin/robotsnap-baseline stop    cohan_nav2
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
bin/robotsnap-baseline   le point d'entrée unique
lib/common.sh            ce que toutes les méthodes partagent
pins.env                 toutes les versions, à un seul endroit
tools/                   les outils partagés (le passage du robot sous ROS)
methods/<méthode>/
    method.env           ce que la méthode est et ce dont elle a besoin
    install.sh           la préparer
    check.sh             la vérifier sans simulation
    run.sh               la lancer
    ...                  ses nœuds, ses paramètres, son modèle
work/                    les espaces de travail construits (ignoré par git)
campaigns/               les suites de scénarios à rejouer
```

Une méthode ne partage rien avec une autre : elle a son dossier, son manifeste
et ses trois scripts. Tout ce qui est vrai de toutes les méthodes vit dans
`lib/common.sh` : la création de la session `tmux`, l'environnement ROS, les
fenêtres que chacune reçoit. Ajouter une méthode, c'est écrire un manifeste et
un `run.sh`, pas recopier un lanceur.

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

## Licence

MIT. Chaque méthode garde la licence de son amont, rappelée dans son dossier.
