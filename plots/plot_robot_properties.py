"""Plot per-link mass, link length, and diagonal inertia from the robot's MJCF model.

Unlike the other scripts here, this reads the model XML directly (models/tron/biped.xml
by default) rather than a simulation log -- these are static design properties, not a
time trace. Every body in that file carries an explicit <inertial mass=... diaginertia=.../>,
and each actuated body has exactly one <joint> driven by one <motor> in <actuator>, so
"actuator mass" below means the mass of the link the corresponding motor/joint is attached to.
"Link length" is the norm of a body's pos offset from its parent body frame, i.e. the
translation spanned by that link in the kinematic chain.

Usage:
    python3 plots/plot_robot_properties.py
    python3 plots/plot_robot_properties.py --xml models/tron/biped.xml --show
"""

from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt


@dataclass
class LinkInfo:
    name: str
    mass: Optional[float] = None
    diaginertia: Optional[Tuple[float, float, float]] = None
    link_length: float = 0.0
    joint_name: Optional[str] = None
    joint_class: Optional[str] = None
    motor_name: Optional[str] = None
    forcerange: Optional[Tuple[float, float]] = None


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", type=Path, default=repo_root / "models" / "tron" / "biped.xml")
    parser.add_argument(
        "--out", type=Path, default=Path(__file__).resolve().parent / "robot_properties.png"
    )
    parser.add_argument(
        "--csv-out",
        type=Path,
        default=Path(__file__).resolve().parent / "robot_properties.csv",
        help="Also write the extracted per-link table to this CSV",
    )
    parser.add_argument("--show", action="store_true", help="Also display the plot interactively")
    parser.add_argument("--figwidth", type=float, default=13.0)
    parser.add_argument("--figheight", type=float, default=8.0)
    return parser.parse_args()


def _parse_floats(text: Optional[str]) -> List[float]:
    if text is None:
        return []
    return [float(token) for token in text.split()]


def parse_default_forceranges(root: ET.Element) -> Dict[str, Tuple[float, float]]:
    """Map a <default class="..."> name to the (lo, hi) forcerange of its <motor>, if any."""
    forceranges: Dict[str, Tuple[float, float]] = {}
    top_default = root.find("default")
    if top_default is None:
        return forceranges

    for default_node in top_default.findall("default"):
        class_name = default_node.get("class")
        motor_node = default_node.find("motor")
        if class_name is None or motor_node is None:
            continue
        values = _parse_floats(motor_node.get("forcerange") or motor_node.get("ctrlrange"))
        if len(values) == 2:
            forceranges[class_name] = (values[0], values[1])

    return forceranges


def parse_actuators(root: ET.Element) -> Dict[str, Tuple[str, Optional[str]]]:
    """Map joint name -> (motor name, motor class)."""
    joint_to_motor: Dict[str, Tuple[str, Optional[str]]] = {}
    actuator_node = root.find("actuator")
    if actuator_node is None:
        return joint_to_motor

    for motor_node in actuator_node.findall("motor"):
        joint_name = motor_node.get("joint")
        motor_name = motor_node.get("name")
        motor_class = motor_node.get("class")
        if joint_name is not None and motor_name is not None:
            joint_to_motor[joint_name] = (motor_name, motor_class)

    return joint_to_motor


def collect_links(root: ET.Element) -> List[LinkInfo]:
    forceranges = parse_default_forceranges(root)
    joint_to_motor = parse_actuators(root)

    worldbody = root.find("worldbody")
    if worldbody is None:
        return []

    links: List[LinkInfo] = []

    def walk(body: ET.Element) -> None:
        name = body.get("name", "<unnamed>")
        pos_values = _parse_floats(body.get("pos"))
        link_length = math.sqrt(sum(v * v for v in pos_values)) if pos_values else 0.0

        info = LinkInfo(name=name, link_length=link_length)

        inertial_node = body.find("inertial")
        if inertial_node is not None:
            mass_str = inertial_node.get("mass")
            if mass_str is not None:
                info.mass = float(mass_str)
            diag_values = _parse_floats(inertial_node.get("diaginertia"))
            if len(diag_values) == 3:
                info.diaginertia = (diag_values[0], diag_values[1], diag_values[2])

        joint_node = body.find("joint")
        if joint_node is not None:
            info.joint_name = joint_node.get("name")
            info.joint_class = joint_node.get("class")
            if info.joint_name in joint_to_motor:
                info.motor_name, motor_class = joint_to_motor[info.joint_name]
                lookup_class = motor_class or info.joint_class
                if lookup_class in forceranges:
                    info.forcerange = forceranges[lookup_class]

        links.append(info)
        for child_body in body.findall("body"):
            walk(child_body)

    for top_level_body in worldbody.findall("body"):
        walk(top_level_body)

    return links


def write_csv(links: List[LinkInfo], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["link", "mass_kg", "link_length_m", "Ixx", "Iyy", "Izz",
             "joint", "motor", "force_or_torque_min", "force_or_torque_max"]
        )
        for link in links:
            ixx, iyy, izz = link.diaginertia if link.diaginertia else ("", "", "")
            lo, hi = link.forcerange if link.forcerange else ("", "")
            writer.writerow(
                [link.name, link.mass or "", link.link_length, ixx, iyy, izz,
                 link.joint_name or "", link.motor_name or "", lo, hi]
            )
    print(f"Wrote {out_path}")


def plot_links(links: List[LinkInfo], out_path: Path, figwidth: float, figheight: float) -> None:
    names = [link.name for link in links]
    masses = [link.mass or 0.0 for link in links]
    lengths = [link.link_length for link in links]

    fig, axes = plt.subplots(3, 1, figsize=(figwidth, figheight))
    fig.suptitle("Robot link properties (from MJCF model)")

    axes[0].bar(names, masses, color="tab:blue")
    axes[0].set_ylabel("mass [kg]")
    axes[0].set_title("Link mass (actuator + housing lumped into each link's body)")
    axes[0].grid(True, axis="y", alpha=0.3)

    axes[1].bar(names, lengths, color="tab:green")
    axes[1].set_ylabel("length [m]")
    axes[1].set_title("Link length (offset from parent body frame)")
    axes[1].grid(True, axis="y", alpha=0.3)

    width = 0.25
    x = range(len(links))
    ixx = [link.diaginertia[0] if link.diaginertia else 0.0 for link in links]
    iyy = [link.diaginertia[1] if link.diaginertia else 0.0 for link in links]
    izz = [link.diaginertia[2] if link.diaginertia else 0.0 for link in links]
    axes[2].bar([i - width for i in x], ixx, width=width, label="Ixx")
    axes[2].bar(list(x), iyy, width=width, label="Iyy")
    axes[2].bar([i + width for i in x], izz, width=width, label="Izz")
    axes[2].set_xticks(list(x))
    axes[2].set_xticklabels(names)
    axes[2].set_ylabel("inertia [kg*m^2]")
    axes[2].set_title("Principal (diagonal) inertia about each link's own COM")
    axes[2].legend()
    axes[2].grid(True, axis="y", alpha=0.3)

    for ax in axes:
        ax.tick_params(axis="x", rotation=45)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"Wrote {out_path}")


def print_summary(links: List[LinkInfo]) -> None:
    total_mass = sum(link.mass or 0.0 for link in links)
    print(f"\n{len(links)} links, total mass = {total_mass:.3f} kg\n")
    header = f"{'link':<16}{'mass[kg]':>10}{'length[m]':>12}{'joint':>16}{'motor':>16}{'force/torque range':>22}"
    print(header)
    print("-" * len(header))
    for link in links:
        mass_str = f"{link.mass:.3f}" if link.mass is not None else "-"
        force_str = f"{link.forcerange[0]:.1f}..{link.forcerange[1]:.1f}" if link.forcerange else "-"
        print(
            f"{link.name:<16}{mass_str:>10}{link.link_length:>12.4f}"
            f"{(link.joint_name or '-'):>16}{(link.motor_name or '-'):>16}{force_str:>22}"
        )


def main() -> None:
    args = parse_args()
    root = ET.parse(args.xml).getroot()
    links = collect_links(root)

    print_summary(links)
    write_csv(links, args.csv_out)
    plot_links(links, args.out, args.figwidth, args.figheight)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
