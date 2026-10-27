"""
Utility functions for working with Mujoco models.
copied from https://github.com/kevinzakka/mink/blob/main/mink/utils.py
"""

from typing import List

import mujoco


def get_body_body_ids(model: mujoco.MjModel, body_id: int) -> List[int]:
    return [
        i
        for i in range(model.nbody)
        if model.body_parentid[i] == body_id and body_id != i
    ]


def get_subtree_body_ids(model: mujoco.MjModel, body_id: int) -> List[int]:
    body_ids: List[int] = []
    stack = [body_id]
    while stack:
        body_id = stack.pop()
        body_ids.append(body_id)
        stack += get_body_body_ids(model, body_id)
    return body_ids


def get_subtree_body_names(model: mujoco.MjModel, body_id: int) -> List[str]:
    return [model.body(i).name for i in get_subtree_body_ids(model, body_id)]


def get_body_geom_ids(model: mujoco.MjModel, body_id: int) -> List[int]:
    geom_start = model.body_geomadr[body_id]
    geom_end = geom_start + model.body_geomnum[body_id]
    return list(range(geom_start, geom_end))


def get_subtree_geom_ids(model: mujoco.MjModel, body_id: int) -> List[int]:
    geom_ids: List[int] = []
    stack = [body_id]
    while stack:
        body_id = stack.pop()
        geom_ids.extend(get_body_geom_ids(model, body_id))
        stack += get_body_body_ids(model, body_id)
    return geom_ids
