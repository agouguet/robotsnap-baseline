#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""La vue ROS 1 de RobotSNAP : ce qu'une methode ROS 1 doit voir pour tourner.

    python robotsnap_ros1_adapter.py [--robot robot_1]

RobotSNAP publie deja tout ce dont une methode a besoin, mais sous des formes
que les methodes ROS 1 de ce benchmark ne lisent pas. Ce nœud en republie
quatre :

* l'odometrie est sur ``/odom`` alors qu'elles attendent ``/robot_odom`` ;
* la foule voyage en JSON (``std_msgs/String``) dans la frame du **robot**,
  alors qu'elles lisent ``/social_sim/agents`` (``agents_msgs/AgentArray``) dans
  la frame du **monde**.
* une pile ROS 1 de l'epoque du turtlebot prend sa pose dans ``/amcl_pose``, que
  RobotSNAP n'a pas - la pose de ``/odom`` la remplace ;
* la meme pile lit un costmap global sur
  ``/move_base/global_costmap/costmap``, et RobotSNAP publie cette grille sur
  ``/map``.

Les quatre sont la meme idee : ce qu'un robot ROS 1 aurait sous la main, ce
nœud le met sous la main d'une methode qui n'a rien a savoir de RobotSNAP. Il
ne fait rien d'autre. Il tourne *dans le conteneur de la
methode* - c'est le seul endroit ou ``agents_msgs`` version ROS 1 est construit -
mais il ne lit que des types standard, ceux que ``ros1_bridge`` sait faire
passer : ``std_msgs/String`` et ``nav_msgs/Odometry``. C'est ce qui permet de
garder le code de la methode intact et de mettre l'adaptation ici.

Trois choses a savoir sur ce fichier :

* il doit rester compatible **Python 2**. Les images CADRL et SARL* embarquent
  ROS Kinetic, dont ``rospy`` est un ``rospy`` Python 2 ; un import de ``yaml``
  echoue des que l'on lance le meme fichier sous ``python3``. Pas de f-strings,
  pas d'annotations, pas de ``print`` sans parentheses.
* il ne suppose rien de l'ordre d'arrivee : sans odometrie il ne publie rien,
  un JSON invalide ou une liste vide sont ignores.
* il republie l'odometrie *avant* de reconstruire la foule, donc la foule est
  toujours placee avec la pose courante.
"""

from __future__ import print_function

import argparse
import json
import math

import rospy
import tf
from agents_msgs.msg import Agent, AgentArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String

try:  # Python 2
    _STRING_TYPES = (str, unicode)  # noqa: F821
except NameError:  # Python 3
    _STRING_TYPES = (str,)

#: Le robot dont les topics sont sans prefixe, comme dans le reste du projet.
PRIMARY_ROBOT = "robot_1"
#: La frame ou toute la scene est dessinee ; ``map`` et ``odom`` y sont confondus.
WORLD_FRAME = "map"
#: La grille de RobotSNAP, et le nom qu'une pile ROS 1 lui donne.
TOPIC_MAP = "/map"
TOPIC_COSTMAP = "/move_base/global_costmap/costmap"
#: Le corps du robot. ``map`` et ``odom`` sont confondus dans RobotSNAP, donc
#: ``odom -> base_link`` porte la pose et ``map -> odom`` est l'identite.
#:
#: ``base_footprint`` est le meme corps sous le nom que les piles de l'epoque du
#: turtlebot ecrivent en dur - exactement ce que ``tools/frames_alias.py`` fait
#: cote ROS 2. Les deux liens sont identiques, donc rien ne peut les contredire.
ROBOT_FRAME = "base_link"
ROBOT_FRAME_ALIASES = ("base_footprint",)
ODOM_FRAME = "odom"


def robot_topic(robot, topic):
    """Le topic d'un robot : nu pour le primaire, prefixe pour les autres."""
    if robot == PRIMARY_ROBOT:
        return "/" + topic
    return "/" + robot + "/" + topic


def yaw_from_quaternion(q):
    """Le lacet plan d'un quaternion, en radians."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def number(value):
    """``float`` pour un vrai nombre, ``None`` pour tout le reste (bools compris)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def agent_id(entry):
    """L'identifiant numerique d'un agent, ecrit nombre ou chaine par RobotSNAP."""
    raw = entry.get("id")
    if isinstance(raw, bool) or raw is None:
        return None
    if isinstance(raw, int):
        return raw
    if isinstance(raw, float):
        return int(raw) if raw == int(raw) else None
    if isinstance(raw, _STRING_TYPES):
        try:
            return int(raw.strip())
        except ValueError:
            return None
    return None


class RobotSnapRos1Adapter(object):
    """Odometrie, foule, pose et grille : la vue ROS 1 de RobotSNAP."""

    def __init__(self, robot):
        self._robot = robot
        self._odom = None
        self._agents = None

        self._odom_pub = rospy.Publisher("/robot_odom", Odometry, queue_size=10)
        # La pose que lit une pile ROS 1 localisee : meme pose que l'odometrie,
        # sous le nom et le type d'AMCL.
        self._amcl_pub = rospy.Publisher(
            "/amcl_pose", PoseWithCovarianceStamped, queue_size=10
        )
        self._agents_pub = rospy.Publisher(
            "/social_sim/agents", AgentArray, queue_size=10
        )
        self._costmap_pub = rospy.Publisher(TOPIC_COSTMAP, OccupancyGrid, queue_size=1)
        # ``tf2_msgs`` ne fait pas partie des types que ce pont connait, donc
        # ``/tf`` ne traverse pas depuis ROS 2 : la chaine est reconstruite ici,
        # du cote ROS 1, exactement comme la pose qu'elle porte.
        self._tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber(robot_topic(robot, "odom"), Odometry, self._on_odom)
        rospy.Subscriber(
            robot_topic(robot, "simulation/agents"), String, self._on_agents
        )
        rospy.Subscriber(TOPIC_MAP, OccupancyGrid, self._on_map)

    # -- entrees -----------------------------------------------------------

    def _on_odom(self, message):
        self._odom = message
        self._odom_pub.publish(message)
        self._broadcast_tf(message)
        self._amcl_pub.publish(self._as_amcl_pose(message))
        self._republish_agents()

    def _on_agents(self, message):
        self._agents = message
        self._republish_agents()

    def _on_map(self, message):
        # Meme grille, meme entete : seul le nom change, et il est absolu comme
        # celui qu'attend la pile.
        self._costmap_pub.publish(message)

    # -- sortie ------------------------------------------------------------

    @staticmethod
    def _as_amcl_pose(odom):
        """L'odometrie vue comme une pose d'AMCL : meme entete, meme covariance."""
        pose = PoseWithCovarianceStamped()
        pose.header = odom.header
        pose.pose = odom.pose
        return pose

    def _broadcast_tf(self, odom):
        """``map -> odom`` (identite) et ``odom -> base_link`` (la pose).

        Une entree d'horodatage nulle est remplacee par l'heure courante : ``tf``
        la lirait comme « le dernier transform disponible », ce qui laisserait la
        chaine vide pour un lecteur qui demande un instant precis.
        """
        stamp = odom.header.stamp
        if stamp.secs == 0 and stamp.nsecs == 0:
            stamp = rospy.Time.now()
        self._tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            stamp,
            ODOM_FRAME,
            WORLD_FRAME,
        )
        pose = odom.pose.pose
        translation = (pose.position.x, pose.position.y, pose.position.z)
        rotation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        for frame in (ROBOT_FRAME,) + ROBOT_FRAME_ALIASES:
            self._tf_broadcaster.sendTransform(
                translation, rotation, stamp, frame, ODOM_FRAME
            )

    def _republish_agents(self):
        odom = self._odom
        payload = self._agents
        if odom is None or payload is None:
            return
        try:
            body = json.loads(payload.data)
        except (TypeError, ValueError):
            return
        entries = body.get("agents") if isinstance(body, dict) else None
        if not isinstance(entries, list) or not entries:
            return

        pose = odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        robot_x, robot_y = pose.position.x, pose.position.y

        message = AgentArray()
        message.header.stamp = odom.header.stamp
        message.header.frame_id = WORLD_FRAME

        for entry in entries:
            if not isinstance(entry, dict):
                continue
            x, y = number(entry.get("x")), number(entry.get("y"))
            identifier = agent_id(entry)
            if x is None or y is None or identifier is None:
                continue
            vx, vy = number(entry.get("vx")), number(entry.get("vy"))
            if vx is None:
                vx = 0.0
            if vy is None:
                vy = 0.0

            agent = Agent()
            agent.id = identifier
            # Frame du robot (x avant, y gauche) -> frame du monde.
            agent.pose.position.x = robot_x + cos_yaw * x - sin_yaw * y
            agent.pose.position.y = robot_y + sin_yaw * x + cos_yaw * y
            agent.pose.orientation.w = 1.0
            agent.velocity.linear.x = cos_yaw * vx - sin_yaw * vy
            agent.velocity.linear.y = sin_yaw * vx + cos_yaw * vy
            visible = entry.get("visible")
            agent.visible_by_robot = True if visible is None else bool(visible)
            message.agents.append(agent)

        self._agents_pub.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot",
        default=PRIMARY_ROBOT,
        help="robot dont l'odometrie et la foule sont adaptees (defaut: %(default)s)",
    )
    arguments = parser.parse_args()

    rospy.init_node("robotsnap_ros1_adapter", anonymous=False)
    RobotSnapRos1Adapter(arguments.robot)
    rospy.spin()


if __name__ == "__main__":
    main()
