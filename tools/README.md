# tools

Les outils qu'aucune méthode ne possède, parce qu'ils parlent à RobotSNAP et non
à une méthode.

## `set_ros_control.py`

```bash
python set_ros_control.py [--robot robot_1] [--timeout 120]
```

Un robot ne lit `/cmd_vel` que lorsque son contrôleur d'entrée est en mode ROS,
et ce mode ne peut se régler qu'une fois le robot existant. Une session qui
démarre le simulateur et la politique en même temps tombe sur cette course : la
première demande arrive avant qu'il y ait un robot pour y répondre. L'outil
réessaie jusqu'à ce que le simulateur accepte, et dit à chaque essai ce qu'il a
reçu. Il sort en 0 quand c'est accepté, en 1 après `--timeout`.

Il n'est pas propre à DRL-VO : la fenêtre `control` de chaque méthode l'appelle.

## `scene_markers.py`

```bash
python scene_markers.py [--robot robot_1]
```

RViz dessine la carte, le laser et l'odométrie tout seul, mais pas les deux
choses qui rendent la navigation sociale *sociale* : où sont les gens, et où le
robot veut aller. Les deux voyagent en JSON, que RViz ne sait pas lire. Ce nœud
les lit et republie ce qu'ils disent en un `MarkerArray` sur `/scene/markers`,
dans la frame `map` :

* les agents de `/simulation/agents`, que Unity publie dans la frame du **robot**,
  ramenés dans `map` par la pose de `/odom` — la même transformation que
  `agents_bridge.py` de CoHAN-Nav2, pour que les deux vues concordent ;
* le robot, en flèche orientée par son cap ;
* le but, tant que la tâche en a un.

Il envoie un `DELETEALL` en tête de chaque tableau : RViz ajoute et met à jour
les marqueurs d'un tableau mais ne retire jamais celui qu'on cesse d'envoyer,
donc un piéton qui s'éloigne garderait son cylindre à l'écran indéfiniment.

Rien de ce qu'il publie n'est relu par un contrôleur : c'est une vue, pas une
entrée.
