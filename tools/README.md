# tools

Les outils qu'aucune méthode ne possède, parce qu'ils parlent à RobotSNAP et non
à une méthode.

## `set_ros_control.py`

```bash
python set_ros_control.py [--robot robot_1] [--timeout 120] [--once]
```

Un robot ne lit `/cmd_vel` que lorsque son contrôleur d'entrée est en mode ROS,
et ce mode ne peut se régler qu'une fois le robot existant. Une session qui
démarre le simulateur et la politique en même temps tombe sur cette course : la
première demande arrive avant qu'il y ait un robot pour y répondre. L'outil
réessaie jusqu'à ce que le simulateur accepte, et dit à chaque essai ce qu'il a
reçu.

Il **reste ensuite en veille**, et c'est la moitié qui compte le plus. Arrêter
une session détruit ses robots (`GameManager.ClearAgents`) et appliquer un
scénario en construit d'autres, à partir du prefab — dont le contrôleur d'entrée
est en mode clavier. Le mode ROS était donc une propriété d'**un seul épisode** :
chaque méthode marchait au premier, puis pilotait dans le vide au suivant, la
pile publiant `/cmd_vel` vers un robot qui l'ignorait, sans qu'aucune fenêtre ne
dise pourquoi. Le mode est maintenant reposé à chaque `/reset_done`, que
l'application publie chaque fois qu'elle applique un scénario — premier
chargement, autre scénario, ou stop puis start du même.

Seule une application de scénario le réarme : un robot qu'un opérateur a remis
au clavier à la main reste au clavier, la main gagne, comme partout ailleurs
dans le projet. `--once` rend l'ancien comportement — régler le mode pour un
épisode puis sortir — pour un script qui n'en veut pas plus.

Il n'est pas propre à DRL-VO : la fenêtre `control` de chaque méthode l'appelle.

## `scenario_goal.py`

```bash
python scenario_goal.py --interface action [--robot robot_1]
python scenario_goal.py --interface topic --topic /global_goal
```

Un scénario porte la mission : il place le robot et il dit où le robot doit
aller. L'application lit ce but elle-même, mais une méthode du benchmark non :
elle écoute une action ou un topic, et elle n'a aucun moyen d'apprendre un but
que personne ne lui envoie. Sans ce nœud, il fallait taper le but à la main dans
la fenêtre `goal` de chaque méthode, alors que DRL-VO, qui lit l'état de la
session, partait tout seul dès le lancement du scénario.

Le nœud lit `/simulation/state`, prend le but du robot demandé, et le remet à la
méthode sous la forme que son manifeste déclare :

| `GOAL_INTERFACE` | ce qu'il fait | méthodes |
| --- | --- | --- |
| `topic` | publie un `PoseStamped` sur `GOAL_TOPIC` (défaut `/global_goal`) | `cadrl`, `sarl_star`, `brne` |
| `action` | envoie un but `navigate_to_pose` | `nav2_dwb`, `cohan_nav2` |
| *(vide)* | rien : la méthode lit la session elle-même | `drl_vo`, `mpc_ebm` |

Il envoie une fois par mission, puis surveille. Un but qui *change* — autre
scénario, but modifié — repart. `/reset_done` est la frontière d'épisode
principale : un `std_msgs/Bool` à `True` permet de renvoyer le même but dans le
nouvel épisode, et `False` est ignoré. Un retour en arrière de
`sim_time_seconds` reste seulement un fallback pour une session qui n'aurait pas
publié `/reset_done` ; une régression proche du signal principal est absorbée
par une courte garde pour ne pas compter deux fois le même reset.

Rien n'y bloque : le nœud est tourné par `rclpy.spin`, donc chaque attente — la
méthode qui s'abonne, le serveur d'action qui apparaît, le planificateur qui
répond — est interrogée depuis un minuteur, jamais dormie.

Pour l'interface `action`, un reset ne renvoie pas seulement le but. Avant de
réarmer quoi que ce soit, `scenario_goal.py` annule le goal actif, efface les
deux costmaps, publie une foule vide quand `cohan_msgs` est disponible, puis
signale le reset au bridge CoHAN. Le goal du nouvel épisode n'est armé qu'après
la fin de cette séquence.

### `nav2_episode_reset.py`

Le coordinateur de cette séquence est séparé du nœud ROS pour rester testable.
Ses états sont `cancel`, `clears`, `empty`, `finish` et il avance uniquement par
appels à `step()` depuis le minuteur. Une annulation sans réponse ou un service
`ClearEntireCostmap` absent est retenté à intervalle court, puis abandonné au
plus tard au timeout global ; le signal de reset et le passage à l'épisode
suivant ne restent donc jamais bloqués.

Il publie toujours `std_msgs/Empty` sur `/robotsnap/episode_reset`. Le bridge
CoHAN efface alors sa dernière odométrie et publie lui aussi un
`TrackedAgents` vide, ce qui couvre aussi le cas où `cohan_msgs` n'est pas
importable dans `scenario_goal.py`.

### `selftest_nav2_episode_reset.py`

```bash
source /opt/ros/humble/setup.bash && python selftest_nav2_episode_reset.py
```

Le selftest pilote le coordinateur avec une horloge, des services, des futures
et des publishers factices. Il vérifie l'ordre annulation, clears local et
global, foule vide, signal bridge, puis goal du nouvel épisode. Il vérifie aussi
que `/reset_done` compte une seule fois, que la régression d'horloge qui suit
est ignorée, que le fallback fonctionne sans `/reset_done`, et qu'une
annulation ou des services qui ne répondent jamais finissent par être bornés
sans bloquer.

## `scene_markers.py`

```bash
python scene_markers.py [--robot robot_1]
```

RViz dessine la carte, le laser et l'odométrie tout seul, mais pas les deux
choses qui rendent la navigation sociale *sociale* : où sont les gens, et où le
robot veut aller. Les deux voyagent en JSON, que RViz ne sait pas lire. Ce nœud
les lit et republie ce qu'ils disent en un `MarkerArray` sur `/scene/markers`,
dans la frame `map` :

* la foule, lue dans les `humans` de `/simulation/state` — déjà dans la frame du
  monde, donc dessinés là où le scénario les a mis, et à la même place que dans
  le viewer Python — avec repli, quand cet instantané n'en nomme aucun, sur les
  agents de `/simulation/agents`, que Unity publie dans la frame du **robot** et
  qui sont alors placés une fois, avec la pose de l'instant où ils ont été
  mesurés ;
* le robot, en flèche orientée par son cap ;
* le but, tant que la tâche en a un.

La foule n'est **pas** reconstruite à chaque message d'odométrie : les agents de
`/simulation/agents` ne valent que contre la pose qui les a mesurés, si bien que
les replacer à chaque tick d'`/odom` — plusieurs fois par instantané — faisait
tourner toute la foule avec le robot entre deux instantanés, puis la remettait
d'un coup à sa vraie place. C'est ce va-et-vient qui tremblait, le plus fort
pendant les rotations. La foule est donc gelée quand elle est mesurée et
seulement réestampillée ensuite.

Il envoie un `DELETEALL` en tête de chaque tableau : RViz ajoute et met à jour
les marqueurs d'un tableau mais ne retire jamais celui qu'on cesse d'envoyer,
donc un piéton qui s'éloigne garderait son cylindre à l'écran indéfiniment.

Rien de ce qu'il publie n'est relu par un contrôleur : c'est une vue, pas une
entrée.

Ce nœud n'est plus démarré méthode par méthode : `window_rviz` de
`lib/common.sh` le lance pour toutes. C'était une ligne dans le `run.sh` de
DRL-VO, donc la seule méthode qui montrait la foule et le but ; les autres
affichaient une carte et un robot, et rien d'autre.

### `selftest_scene_markers.py`

```bash
source /opt/ros/humble/setup.bash && python selftest_scene_markers.py
```

Le nœud précédent, vérifié sans simulation, sans RViz et sans build Unity : il
rejoue à la main des messages d'odométrie, d'agents et d'état et relit la foule
après chacun. Le premier cas est la régression elle-même — un robot qui tourne
d'un radian pendant que la foule d'un agent ne doit pas bouger d'un millimètre.
Sans argument, sans ROS 2 sourcé : le nœud importe `rclpy`.

## `ros1/`

Tout ce qu'une méthode ROS 1 a besoin qu'on lui monte autour : un maître ROS 1,
un pont vers ROS 2, et la vue ROS 1 de RobotSNAP (`/robot_odom` et
`/social_sim/agents`). C'est un sous-dossier parce que c'est un montage complet —
quatre services, un adaptateur — et non un outil isolé. Voir
[`ros1/README.md`](ros1/README.md).

Une méthode ROS 1 n'en connaît que deux fonctions de `lib/common.sh` :
`ros1_up_command` et `ros1_down_command`.
