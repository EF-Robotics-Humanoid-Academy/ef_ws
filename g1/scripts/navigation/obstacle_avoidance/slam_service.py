#!/usr/bin/env python3
"""
slam_service.py
===============

Client wrapper for the G1 SLAM/navigation service interface via unitree_sdk2py RPC.
Service name: "slam_operate"
Version: 1.0.0.1
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Optional

from unitree_sdk2py.rpc.client import Client


SERVICE_NAME = "slam_operate"
SERVICE_VERSION = "1.0.0.1"

API_START_MAPPING = 1801
API_END_MAPPING = 1802
API_INIT_POSE = 1804
API_POSE_NAV = 1102
API_PAUSE_NAV = 1201
API_RESUME_NAV = 1202
API_CLOSE_SLAM = 1901


@dataclass
class SlamResponse:
    code: int
    raw: Any


class SlamOperateClient(Client):
    def __init__(self, enable_lease: bool = False) -> None:
        super().__init__(SERVICE_NAME, enable_lease)

    def Init(self) -> None:
        self._RegistApi(API_START_MAPPING, 0)
        self._RegistApi(API_END_MAPPING, 0)
        self._RegistApi(API_INIT_POSE, 0)
        self._RegistApi(API_POSE_NAV, 0)
        self._RegistApi(API_PAUSE_NAV, 0)
        self._RegistApi(API_RESUME_NAV, 0)
        self._RegistApi(API_CLOSE_SLAM, 0)
        self._SetApiVerson(SERVICE_VERSION)

    def _call(self, api_id: int, payload: dict) -> SlamResponse:
        param = json.dumps(payload, ensure_ascii=True)
        code, data = self._Call(api_id, param)
        return SlamResponse(code=code, raw=data)

    def start_mapping(self, slam_type: str = "indoor") -> SlamResponse:
        return self._call(API_START_MAPPING, {"data": {"slam_type": slam_type}})

    def end_mapping(self, address: str) -> SlamResponse:
        return self._call(API_END_MAPPING, {"data": {"address": address}})

    def init_pose(
        self,
        x: float,
        y: float,
        z: float,
        q_x: float,
        q_y: float,
        q_z: float,
        q_w: float,
        address: str,
    ) -> SlamResponse:
        return self._call(
            API_INIT_POSE,
            {
                "data": {
                    "x": x,
                    "y": y,
                    "z": z,
                    "q_x": q_x,
                    "q_y": q_y,
                    "q_z": q_z,
                    "q_w": q_w,
                    "address": address,
                }
            },
        )

    def pose_nav(
        self,
        x: float,
        y: float,
        z: float,
        q_x: float,
        q_y: float,
        q_z: float,
        q_w: float,
        mode: int = 1,
    ) -> SlamResponse:
        return self._call(
            API_POSE_NAV,
            {
                "data": {
                    "targetPose": {
                        "x": x,
                        "y": y,
                        "z": z,
                        "q_x": q_x,
                        "q_y": q_y,
                        "q_z": q_z,
                        "q_w": q_w,
                    },
                    "mode": mode,
                }
            },
        )

    def pause_nav(self) -> SlamResponse:
        return self._call(API_PAUSE_NAV, {"data": {}})

    def resume_nav(self) -> SlamResponse:
        return self._call(API_RESUME_NAV, {"data": {}})

    def close_slam(self) -> SlamResponse:
        return self._call(API_CLOSE_SLAM, {"data": {}})
