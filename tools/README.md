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
