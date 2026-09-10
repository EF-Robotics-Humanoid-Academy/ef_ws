#!/usr/bin/env python3
"""
Phone-first Dash web-app version of ik_pose_cli_v3.py

Run:  python3 ik_pose_dash_v2.py [same CLI flags as ik_pose_cli_v3.py]
Then open  http://localhost:8050  in a browser on the same network.

Extra flags (not in CLI v3):
  --port PORT    HTTP port (default 8050)
  --host HOST    bind address (default 0.0.0.0)
  --debug        enable Dash debug mode
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
MODULES_DIR = os.path.join(ROOT_DIR, "modules")
for _p in reversed((SCRIPT_DIR, os.path.join(SCRIPT_DIR, "modules"),
                    ROOT_DIR, MODULES_DIR)):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

# ── Import robot logic ────────────────────────────────────────────────────────
from ik_pose_cli_v3 import (
    IKPoseCLI, _parse_args, ControllerLockError,
    ARM_CONTROL_MODES, HAND_CONTROL_MODES,
    DOF_NAMES, DOF_UNITS, N_DOFS,
    ARM_JOINTS, JOINT_LABELS, UPPER_BODY_JOINTS, WAIST_JOINTS,
    LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS,
    _rpy_from_R,
)

import dash
from dash import dcc, html, Input, Output, State, callback_context, no_update, ALL
import dash_bootstrap_components as dbc

# ── Globals ───────────────────────────────────────────────────────────────────
_ctrl: Optional[IKPoseCLI] = None
_lock = threading.RLock()

# Background tick thread
_tick_thread: Optional[threading.Thread] = None
_tick_running = False

# Hand action thread (blocking open/close calls run here)
_hand_thread: Optional[threading.Thread] = None
_hand_status = "idle"          # read by refresh callback for display


# ── Tick loop ─────────────────────────────────────────────────────────────────

def _tick_loop() -> None:
    global _tick_running
    while _tick_running:
        with _lock:
            c = _ctrl
            if c is not None and not c._closed:
                try:
                    c.tick()
                except Exception as exc:
                    c.status = f"Tick error: {exc}"
        # Sleep outside the lock
        hz = _ctrl.rate_hz if _ctrl else 25.0
        time.sleep(1.0 / hz)


# ── Hand action helpers ────────────────────────────────────────────────────────

def _run_hand_action(fn, *args, **kwargs) -> None:
    """Run a blocking hand call in a daemon thread so Dash stays responsive."""
    global _hand_thread, _hand_status

    def _worker():
        global _hand_status
        try:
            _hand_status = "running…"
            fn(*args, **kwargs)
            _hand_status = "done"
        except Exception as exc:
            _hand_status = f"error: {exc}"

    _hand_thread = threading.Thread(target=_worker, daemon=True)
    _hand_thread.start()


def _dex3_action(hand: str, action: str, hold_s: float, ramp_s: Optional[float]) -> None:
    """Call open/close on the Dex3HandController(s) already owned by the IK controller."""
    global _hand_status
    with _lock:
        c = _ctrl
        if c is None:
            return
        # make sure controllers exist for the requested side(s)
        sides = ["left", "right"] if hand == "both" else [hand]
        orig_mode = c.hand_control_mode
        # Temporarily widen to ensure the controller is initialised
        if hand == "both":
            c.hand_control_mode = "both"
        elif hand not in c.hand_control_mode:
            c.hand_control_mode = hand
        c._init_hand_controllers()
        controllers = {s: c.hand_controllers.get(s) for s in sides}
        c.hand_control_mode = orig_mode   # restore

    for side, ctrl in controllers.items():
        if ctrl is None:
            _hand_status = f"No Dex3 controller for {side}"
            continue
        ramp = ramp_s if ramp_s and ramp_s > 0 else None
        if action == "open":
            ctrl.open(hold_s=hold_s, ramp_s=ramp)
        else:
            ctrl.close(hold_s=hold_s, ramp_s=ramp)


def _inspire_action(hand: str, action: str, speed: int, force: int) -> None:
    """Call inspire_sdk open_hand / close_hand."""
    from inspire_sdk import open_hand, close_hand
    fn = open_hand if action == "open" else close_hand
    sides = ["left", "right"] if hand == "both" else [hand]
    for side in sides:
        fn(side, speed=speed, force=force, hold=0.0)


# ── Layout helpers ─────────────────────────────────────────────────────────────

def _card(title: str, children, **kwargs) -> dbc.Card:
    return dbc.Card([
        dbc.CardHeader(html.B(title)),
        dbc.CardBody(children),
    ], className="mb-2", **kwargs)


def _touch_button(label: str, component_id, color: str = "secondary",
                  outline: bool = False, className: str = "",
                  **kwargs) -> dbc.Button:
    return dbc.Button(
        label,
        id=component_id,
        color=color,
        outline=outline,
        className=f"touch-btn {className}".strip(),
        **kwargs,
    )


def _axis_readout(i: int) -> html.Div:
    return html.Div([
        html.Div(DOF_NAMES[i], className="axis-name"),
        html.Div([
            html.Span("live ", className="text-muted"),
            html.Span("—", id={"type": "live-dof", "index": i},
                      className="font-monospace"),
        ], className="axis-value"),
        html.Div([
            html.Span("target ", className="text-muted"),
            html.Span("—", id={"type": "tgt-dof", "index": i},
                      className="font-monospace text-info"),
        ], className="axis-value"),
        html.Div([
            html.Span("step ", className="text-muted"),
            html.Span("—", id={"type": "step-dof", "index": i},
                      className="font-monospace text-warning"),
        ], className="axis-value"),
    ], className="axis-card")


def _position_pad() -> html.Div:
    return html.Div([
        html.Div([
            _touch_button("X-", {"type": "dec-dof", "index": 0}, "primary",
                          className="x-btn"),
            _touch_button("X+", {"type": "inc-dof", "index": 0}, "primary",
                          className="x-btn"),
        ], className="x-rail"),
        html.Div([
            html.Div(),
            _touch_button("Z+", {"type": "inc-dof", "index": 2}, "info",
                          className="dpad-btn"),
            html.Div(),
            _touch_button("Y-", {"type": "dec-dof", "index": 1}, "info",
                          className="dpad-btn"),
            html.Div("Y/Z", className="dpad-center"),
            _touch_button("Y+", {"type": "inc-dof", "index": 1}, "info",
                          className="dpad-btn"),
            html.Div(),
            _touch_button("Z-", {"type": "dec-dof", "index": 2}, "info",
                          className="dpad-btn"),
            html.Div(),
        ], className="dpad-grid"),
    ], className="position-pad")


def _orientation_pad() -> html.Div:
    return html.Div([
        html.Div([
            html.Div("Roll", className="orient-label"),
            _touch_button("-", {"type": "dec-dof", "index": 3}, "secondary"),
            _touch_button("+", {"type": "inc-dof", "index": 3}, "secondary"),
        ], className="orient-row"),
        html.Div([
            html.Div("Pitch", className="orient-label"),
            _touch_button("-", {"type": "dec-dof", "index": 4}, "secondary"),
            _touch_button("+", {"type": "inc-dof", "index": 4}, "secondary"),
        ], className="orient-row"),
        html.Div([
            html.Div("Yaw", className="orient-label"),
            _touch_button("-", {"type": "dec-dof", "index": 5}, "secondary"),
            _touch_button("+", {"type": "inc-dof", "index": 5}, "secondary"),
        ], className="orient-row"),
    ], className="orientation-pad")


def _sticky_actions() -> html.Div:
    return dbc.Row([
        dbc.Col(dbc.Button("Sync", id="btn-sync", color="info",
                           className="top-action w-100"), width=3),
        dbc.Col(dbc.Button("Release", id="btn-release", color="warning",
                           className="top-action w-100"), width=3),
        dbc.Col(dbc.Button("Reengage", id="btn-reengage", color="success",
                           className="top-action w-100"), width=3),
        dbc.Col(dbc.Button("Zero", id="btn-zero-gain", color="danger",
                           className="top-action w-100"), width=3),
    ], className="sticky-actions g-1")


def _settings_panel() -> html.Div:
    return html.Div([
        dbc.Row([
            dbc.Col([
                html.Small("Position step", className="d-block text-muted"),
                dbc.ButtonGroup([
                    dbc.Button("÷2", id={"type": "halve-step", "index": 0},
                               color="outline-secondary", size="sm"),
                    dbc.Button("×2", id={"type": "double-step", "index": 0},
                               color="outline-secondary", size="sm"),
                ]),
            ], width=6),
            dbc.Col([
                html.Small("Rotation step", className="d-block text-muted"),
                dbc.ButtonGroup([
                    dbc.Button("÷2", id={"type": "halve-step", "index": 3},
                               color="outline-secondary", size="sm"),
                    dbc.Button("×2", id={"type": "double-step", "index": 3},
                               color="outline-secondary", size="sm"),
                ]),
            ], width=6),
        ], className="g-2 mb-2"),
        dbc.Row([
            dbc.Col([
                html.Small("Ramp r/s", className="d-block text-muted"),
                dbc.InputGroup([
                    dbc.Input(id="ramp-input", type="number", value=0.2,
                              step=0.01, min=0.01, size="sm"),
                    dbc.Button("Set", id="ramp-set", size="sm", color="primary"),
                ]),
            ], width=12, className="mb-2"),
            dbc.Col([
                html.Small("max_dq rad", className="d-block text-muted"),
                dbc.InputGroup([
                    dbc.Input(id="maxdq-input", type="number", value=0.2,
                              step=0.005, min=0.005, size="sm"),
                    dbc.Button("Set", id="maxdq-set", size="sm", color="primary"),
                ]),
            ], width=12, className="mb-2"),
            dbc.Col([
                html.Small("Waist PR kp", className="d-block text-muted"),
                dbc.InputGroup([
                    dbc.Input(id="waist-kp-input", type="number", value=200.0,
                              step=10, min=0, size="sm"),
                    dbc.Button("Set", id="waist-kp-set", size="sm", color="primary"),
                ]),
            ], width=12),
        ], className="g-2"),
    ])


TAB_LABEL_STYLE = {
    "display": "block",
    "width": "100%",
    "minHeight": "44px",
    "padding": "10px 6px",
    "color": "#f8f9fa",
    "backgroundColor": "#30343b",
    "border": "1px solid #6c757d",
    "borderRadius": "8px",
    "fontWeight": "700",
    "textAlign": "center",
}

TAB_ACTIVE_LABEL_STYLE = {
    **TAB_LABEL_STYLE,
    "backgroundColor": "#0d6efd",
    "borderColor": "#86b7fe",
    "color": "#ffffff",
}


def _dof_table() -> html.Div:
    header = dbc.Row([
        dbc.Col(html.Small("DOF"),    width=1),
        dbc.Col(html.Small("Live FK"), width=2),
        dbc.Col(html.Small("Target"),  width=2),
        dbc.Col(html.Small("Step"),    width=2),
        dbc.Col(html.Small("± Adjust"), width=2),
        dbc.Col(html.Small("Step ÷/×"), width=3),
    ], className="fw-bold mb-1 px-1")

    rows = []
    for i in range(N_DOFS):
        rows.append(dbc.Row([
            dbc.Col(html.Code(DOF_NAMES[i]), width=1),
            dbc.Col(html.Span("—", id={"type": "live-dof",  "index": i},
                              className="font-monospace small"), width=2),
            dbc.Col(html.Span("—", id={"type": "tgt-dof",   "index": i},
                              className="font-monospace small text-info"), width=2),
            dbc.Col(html.Span("—", id={"type": "step-dof",  "index": i},
                              className="font-monospace small text-muted"), width=2),
            dbc.Col(dbc.ButtonGroup([
                dbc.Button("−", id={"type": "dec-dof", "index": i},
                           color="secondary", size="sm", className="px-2"),
                dbc.Button("+", id={"type": "inc-dof", "index": i},
                           color="secondary", size="sm", className="px-2"),
            ]), width=2),
            dbc.Col(dbc.ButtonGroup([
                dbc.Button("÷2", id={"type": "halve-step",  "index": i},
                           color="outline-secondary", size="sm"),
                dbc.Button("×2", id={"type": "double-step", "index": i},
                           color="outline-secondary", size="sm"),
            ]), width=3),
        ], className="mb-1 align-items-center px-1"))
    return html.Div([header] + rows)


def _dex3_card() -> dbc.Card:
    return _card("Dex3 Hand Control (sdk_client / sdk_hand)", [
        dbc.Row([
            dbc.Col([
                html.Small("Tick-loop mode:", className="d-block text-muted"),
                dbc.RadioItems(
                    id="hand-mode",
                    options=[{"label": m, "value": m} for m in HAND_CONTROL_MODES],
                    value="off",
                    inline=True,
                    className="small",
                ),
            ], width=12, className="mb-2"),
        ]),
        dbc.Row([
            dbc.Col([
                html.Small("Grip %:", className="me-1"),
                dbc.InputGroup([
                    dbc.Input(id="grip-input", type="number", value=0,
                              step=5, min=0, max=100, style={"width": "70px"}),
                    dbc.Button("Set", id="grip-set", size="sm", color="primary"),
                ], size="sm"),
            ], width="auto"),
        ], className="mb-2"),
        dbc.Row([
            dbc.Col([
                html.Small("One-shot open/close:", className="d-block text-muted"),
                dbc.Row([
                    dbc.Col([
                        html.Small("Side:"),
                        dbc.RadioItems(
                            id="dex3-side",
                            options=[
                                {"label": "Right", "value": "right"},
                                {"label": "Left",  "value": "left"},
                                {"label": "Both",  "value": "both"},
                            ],
                            value="right",
                            inline=True,
                            className="small",
                        ),
                    ], width="auto"),
                    dbc.Col([
                        html.Small("hold s:"),
                        dbc.Input(id="dex3-hold-s", type="number",
                                  value=0.6, step=0.1, min=0,
                                  style={"width": "65px"}, size="sm"),
                    ], width="auto"),
                    dbc.Col([
                        html.Small("ramp s (0=default):"),
                        dbc.Input(id="dex3-ramp-s", type="number",
                                  value=0, step=0.1, min=0,
                                  style={"width": "65px"}, size="sm"),
                    ], width="auto"),
                ], className="mb-1 align-items-end"),
                dbc.ButtonGroup([
                    dbc.Button("✋ Open",  id="dex3-open",  color="success", size="sm"),
                    dbc.Button("✊ Close", id="dex3-close", color="warning", size="sm"),
                ]),
            ], width=12),
        ]),
    ])


def _inspire_card() -> dbc.Card:
    return _card("Inspire Hand Control (inspire_sdk, Modbus TCP)", [
        dbc.Row([
            dbc.Col([
                html.Small("Side:"),
                dbc.RadioItems(
                    id="inspire-side",
                    options=[
                        {"label": "Right", "value": "right"},
                        {"label": "Left",  "value": "left"},
                        {"label": "Both",  "value": "both"},
                    ],
                    value="right",
                    inline=True,
                    className="small",
                ),
            ], width="auto"),
            dbc.Col([
                html.Small("Speed:"),
                dbc.Input(id="inspire-speed", type="number",
                          value=200, step=50, min=1, max=1000,
                          style={"width": "70px"}, size="sm"),
            ], width="auto"),
            dbc.Col([
                html.Small("Force:"),
                dbc.Input(id="inspire-force", type="number",
                          value=200, step=50, min=1, max=1000,
                          style={"width": "70px"}, size="sm"),
            ], width="auto"),
        ], className="mb-2 align-items-end"),
        dbc.ButtonGroup([
            dbc.Button("✋ Open",  id="inspire-open",  color="success", size="sm"),
            dbc.Button("✊ Close", id="inspire-close", color="warning", size="sm"),
        ]),
        html.Div(id="hand-status-bar",
                 className="mt-2 small font-monospace text-muted"),
    ])


def make_layout() -> html.Div:
    return dbc.Container([
        html.Div([
            dbc.Row([
                dbc.Col([
                    html.H5("IK Pose Dash v2", className="mb-0"),
                    html.Div("Phone control", className="text-muted small"),
                ], className="min-w-0"),
                dbc.Col(html.Span(id="conn-badge"), width="auto"),
                dbc.Col(html.Span(id="armed-badge"), width="auto"),
            ], className="align-items-center g-2"),
            _sticky_actions(),
        ], className="phone-header"),

        dbc.Nav([
            dbc.NavItem(dbc.NavLink("Control", id="tab-control", href="#",
                                    active=True, className="phone-tab-link")),
            dbc.NavItem(dbc.NavLink("Poses", id="tab-poses", href="#",
                                    active=False, className="phone-tab-link")),
            dbc.NavItem(dbc.NavLink("Status", id="tab-status", href="#",
                                    active=False, className="phone-tab-link")),
        ], pills=True, fill=True, className="phone-tabs mb-2"),

        html.Div([
            html.Div(
                id="pane-control",
                style={"display": "block"},
                children=[
                _card("Arm", [
                    html.Small("Arm selection", className="d-block text-muted"),
                    dbc.RadioItems(
                        id="arm-mode",
                        options=[{"label": m, "value": m} for m in ARM_CONTROL_MODES],
                        value="right",
                        inline=True,
                        className="mb-2",
                    ),
                    dbc.Switch(id="orient-stiff", label="Fix EE orientation",
                               value=True, className="mb-1"),
                    dbc.Switch(id="waist-toggle", label="Waist hold",
                               value=True),
                ]),

                _card("Position", [
                    html.Div([_axis_readout(i) for i in range(3)],
                             className="axis-grid mb-2"),
                    _position_pad(),
                ]),

                _card("Orientation", [
                    html.Div([_axis_readout(i) for i in range(3, 6)],
                             className="axis-grid mb-2"),
                    _orientation_pad(),
                ]),

                _card("Save Pose", [
                    dbc.InputGroup([
                        dbc.Input(id="pose-name-input",
                                  placeholder="Pose name", size="lg"),
                        dbc.Button("Save", id="pose-save", color="success"),
                    ]),
                ]),

                dbc.Accordion([
                    dbc.AccordionItem(_settings_panel(), title="Motion settings"),
                    dbc.AccordionItem(_dex3_card().children[1], title="Dex3 hand"),
                    dbc.AccordionItem(_inspire_card().children[1], title="Inspire hand"),
                ], start_collapsed=True, className="mb-2"),
                ],
            ),

            html.Div(
                id="pane-poses",
                style={"display": "none"},
                children=[
                _card("Saved Poses", [
                    dbc.RadioItems(id="pose-selector", options=[], value=None,
                                   className="pose-list mb-2"),
                    dbc.ButtonGroup([
                        dbc.Button("Load", id="pose-load", color="primary"),
                        dbc.Button("Delete", id="pose-delete", color="danger"),
                        dbc.Button("Add to Seq", id="pose-add-seq", color="secondary"),
                    ], className="wide-group"),
                    dbc.Switch(id="include-waist-new",
                               label="Include waist in new sequence steps",
                               value=True, className="mt-2 small"),
                ]),
                _card("Sequence", [
                    dbc.RadioItems(id="seq-selector", options=[], value=None,
                                   className="pose-list mb-2"),
                    dbc.Row([
                        dbc.Col([
                            html.Small("Gap / speed s", className="d-block text-muted"),
                            dbc.InputGroup([
                                dbc.Input(id="seq-gap-input", type="number",
                                          value=2.0, step=0.5, min=0, size="sm"),
                                dbc.Button("Set", id="seq-gap-set",
                                           size="sm", color="primary"),
                            ]),
                        ], width=12),
                    ], className="g-2 mb-2"),
                    dbc.ButtonGroup([
                        dbc.Button("Up", id="seq-up", color="secondary"),
                        dbc.Button("Down", id="seq-down", color="secondary"),
                        dbc.Button("Remove", id="seq-remove", color="danger"),
                    ], className="wide-group mb-2"),
                    dbc.ButtonGroup([
                        dbc.Button("Run", id="seq-run", color="success"),
                        dbc.Button("Stop", id="seq-stop", color="warning"),
                    ], className="wide-group"),
                ]),
                ],
            ),

            html.Div(
                id="pane-status",
                style={"display": "none"},
                children=[
                _card("IK Status", [
                    html.Div(id="ik-status-left", className="mb-1"),
                    html.Div(id="ik-status-right"),
                ]),
                _card("Joint Readout",
                      html.Div(id="joint-readout", className="font-monospace small")),
                ],
            ),
        ]),

        dbc.Alert(id="status-bar", color="dark", className="status-bar mb-2"),

        dcc.Interval(id="interval", interval=100, n_intervals=0),
        dcc.Store(id="active-tab", data="control"),
        dcc.Store(id="_act"),
        dcc.Store(id="_hand-act"),
    ], fluid=True, className="phone-shell")


# ── App ───────────────────────────────────────────────────────────────────────
app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.DARKLY],
    title="IK Pose Dash v2",
    suppress_callback_exceptions=True,
)
app.index_string = """
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <style>
            body { overscroll-behavior: none; }
            .phone-shell {
                max-width: 560px;
                padding-left: 10px;
                padding-right: 10px;
                padding-bottom: 22px;
            }
            .phone-header {
                position: sticky;
                top: 0;
                z-index: 1030;
                background: #222;
                border-bottom: 1px solid #444;
                padding: 8px 0 10px;
            }
            .sticky-actions {
                margin-top: 8px;
            }
            .top-action {
                min-height: 44px;
                padding-left: 4px;
                padding-right: 4px;
                font-weight: 700;
                white-space: normal;
            }
            .status-bar {
                margin: 8px 0;
                font-size: 0.82rem;
                min-height: 38px;
                max-height: 56px;
                overflow: auto;
                word-break: break-word;
            }
            .phone-tabs {
                margin-top: 8px;
                position: sticky;
                top: 92px;
                z-index: 1020;
                display: grid;
                grid-template-columns: repeat(3, minmax(0, 1fr));
                gap: 6px;
                background: #222;
                padding-bottom: 8px;
                border-bottom: 1px solid #444;
            }
            .phone-tabs .nav-item {
                min-width: 0;
            }
            .phone-tab-link {
                min-height: 44px;
                font-weight: 700;
                width: 100%;
                border: 1px solid #555;
                border-radius: 8px;
                color: #f1f3f5;
                background: #30343b;
                padding-left: 4px;
                padding-right: 4px;
                text-align: center;
                display: flex;
                align-items: center;
                justify-content: center;
            }
            .phone-tab-link.active {
                color: #fff;
                background: #0d6efd;
                border-color: #66a3ff;
            }
            .card {
                border-radius: 8px;
            }
            .card-header {
                padding: 0.5rem 0.75rem;
            }
            .card-body {
                padding: 0.75rem;
            }
            .axis-grid {
                display: grid;
                grid-template-columns: repeat(3, minmax(0, 1fr));
                gap: 6px;
            }
            .axis-card {
                border: 1px solid #444;
                border-radius: 8px;
                padding: 6px;
                min-width: 0;
                background: #181b20;
            }
            .axis-name {
                font-weight: 700;
                text-transform: uppercase;
                margin-bottom: 2px;
            }
            .axis-value {
                font-size: 0.7rem;
                line-height: 1.25;
                overflow-wrap: anywhere;
            }
            .position-pad {
                display: grid;
                grid-template-columns: 0.85fr 1.15fr;
                gap: 10px;
                align-items: stretch;
            }
            .x-rail {
                display: grid;
                grid-template-rows: repeat(2, minmax(78px, 1fr));
                gap: 8px;
            }
            .dpad-grid {
                display: grid;
                grid-template-columns: repeat(3, minmax(0, 1fr));
                grid-template-rows: repeat(3, minmax(56px, 1fr));
                gap: 6px;
            }
            .dpad-center {
                display: flex;
                align-items: center;
                justify-content: center;
                border: 1px solid #444;
                border-radius: 8px;
                color: #bbb;
                font-weight: 700;
                background: #181b20;
            }
            .orientation-pad {
                display: grid;
                gap: 8px;
            }
            .orient-row {
                display: grid;
                grid-template-columns: 1fr 1fr 1fr;
                gap: 8px;
                align-items: center;
            }
            .orient-label {
                font-weight: 700;
                color: #ddd;
            }
            .touch-btn {
                min-height: 56px;
                font-size: 1.05rem;
                font-weight: 800;
                touch-action: manipulation;
            }
            .x-btn {
                min-height: 78px;
            }
            .wide-group {
                display: grid;
                grid-auto-flow: column;
                grid-auto-columns: 1fr;
                width: 100%;
                gap: 0;
            }
            .wide-group .btn {
                min-height: 44px;
                white-space: normal;
            }
            .pose-list label {
                display: block;
                border: 1px solid #444;
                border-radius: 8px;
                padding: 8px 10px;
                margin-bottom: 6px;
                background: #181b20;
                overflow-wrap: anywhere;
            }
            .form-check-inline {
                margin-right: 0.75rem;
                margin-bottom: 0.25rem;
            }
            @media (max-width: 390px) {
                .phone-shell { padding-left: 6px; padding-right: 6px; }
                .top-action { font-size: 0.78rem; }
                .axis-value { font-size: 0.64rem; }
                .touch-btn { min-height: 52px; font-size: 0.98rem; }
                .position-pad { grid-template-columns: 0.8fr 1.2fr; gap: 6px; }
                .phone-tabs { top: 88px; }
            }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
    </body>
</html>
"""
app.layout = make_layout()


# ── State snapshot ────────────────────────────────────────────────────────────

def _snap() -> dict:
    """Return a JSON-safe snapshot of controller state. Empty dict if offline."""
    with _lock:
        c = _ctrl
        if c is None or c._closed:
            return {}

        arm = c._display_arm()
        T_live = c._fk_live(arm) if c.seeded else np.eye(4)
        T_tgt  = c.target_T[arm]
        rpy_live = _rpy_from_R(T_live[:3, :3])
        rpy_tgt  = _rpy_from_R(T_tgt[:3, :3])

        dof_live = [float(T_live[i, 3]) for i in range(3)] + list(map(float, rpy_live))
        dof_tgt  = [float(T_tgt[i, 3])  for i in range(3)] + list(map(float, rpy_tgt))
        steps    = [c.pos_step if i < 3 else c.rot_step for i in range(N_DOFS)]

        joint_tgt  = {a: [c.desired_targets[j]  for j in jl]
                      for a, jl in ARM_JOINTS.items()}
        joint_live = {a: [c.latest_positions.get(j, c.current_targets[j]) for j in jl]
                      for a, jl in ARM_JOINTS.items()}

        running_step = c.sequence_step_index - 1 if c.sequence_running else -1

        return {
            "seeded": c.seeded,
            "armed":  c.armed,
            "status": c.status,
            "arm_control_mode":  c.arm_control_mode,
            "waist_enabled":     c.waist_enabled,
            "orient_stiff":      c.orient_stiff,
            "hand_control_mode": c.hand_control_mode,
            "hand_grip_percent": c.hand_grip_percent,
            "max_speed":   c.max_speed,
            "max_dq":      c.max_dq,
            "waist_pr_kp": c.waist_pr_kp,
            "sequence_gap_s":    c.sequence_gap_s,
            "sequence_running":  c.sequence_running,
            "dof_live": dof_live,
            "dof_tgt":  dof_tgt,
            "steps":    steps,
            "ik_info": {k: dict(v) for k, v in c.ik_info.items()},
            "joint_tgt":  joint_tgt,
            "joint_live": joint_live,
            "saved_poses": [
                {"name": p.get("name", f"pose_{i}"),
                 "saved_at": str(p.get("saved_at", ""))[:19]}
                for i, p in enumerate(c.saved_poses)
            ],
            "sequence_steps": [
                {
                    "pose_name": (
                        c.saved_poses[s["pose_index"]].get("name", "?")
                        if 0 <= s.get("pose_index", -1) < len(c.saved_poses)
                        else "<missing>"
                    ),
                    "include_waist": s.get("include_waist", True),
                    "pose_index": s.get("pose_index", -1),
                }
                for s in c.sequence_steps
            ],
            "running_step": running_step,
        }


# ── Manual tab switching ──────────────────────────────────────────────────────

@app.callback(
    Output("active-tab", "data"),
    Input("tab-control", "n_clicks"),
    Input("tab-poses", "n_clicks"),
    Input("tab-status", "n_clicks"),
    prevent_initial_call=True,
)
def switch_tab(*_):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    if trig == "tab-poses":
        return "poses"
    if trig == "tab-status":
        return "status"
    return "control"


@app.callback(
    Output("tab-control", "active"),
    Output("tab-poses", "active"),
    Output("tab-status", "active"),
    Output("pane-control", "style"),
    Output("pane-poses", "style"),
    Output("pane-status", "style"),
    Input("active-tab", "data"),
)
def render_active_tab(active):
    active = active or "control"
    hidden = {"display": "none"}
    shown = {"display": "block"}
    return (
        active == "control",
        active == "poses",
        active == "status",
        shown if active == "control" else hidden,
        shown if active == "poses" else hidden,
        shown if active == "status" else hidden,
    )


# ── Refresh callback ──────────────────────────────────────────────────────────

@app.callback(
    Output("conn-badge",  "children"),
    Output("armed-badge", "children"),
    Output("status-bar",  "children"),
    Output("status-bar",  "style"),
    Output({"type": "live-dof",  "index": ALL}, "children"),
    Output({"type": "tgt-dof",   "index": ALL}, "children"),
    Output({"type": "step-dof",  "index": ALL}, "children"),
    Output("ik-status-left",  "children"),
    Output("ik-status-right", "children"),
    Output("joint-readout",   "children"),
    Output("pose-selector",   "options"),
    Output("seq-selector",    "options"),
    Output("hand-status-bar", "children"),
    Input("interval", "n_intervals"),
)
def refresh(_n):
    s = _snap()
    offline_style = {"background": "#1a0000", "color": "#888",
                     "padding": "8px", "borderRadius": "4px",
                     "border": "1px solid #333", "fontFamily": "monospace"}
    empty = ["—"] * N_DOFS
    if not s:
        return (
            dbc.Badge("OFFLINE", color="secondary"),
            dbc.Badge("—",       color="secondary"),
            "Controller not running", offline_style,
            empty, empty, empty,
            "", "", "",
            [], [],
            _hand_status,
        )

    conn_color  = "success" if s["seeded"] else "danger"
    armed_color = "success" if s["armed"]  else "warning"
    conn_badge  = dbc.Badge("CONNECTED" if s["seeded"] else "WAITING", color=conn_color)
    armed_badge = dbc.Badge("ARMED"     if s["armed"]  else "RELEASED", color=armed_color)

    ok_color = "#00d26a" if (s["armed"] and s["seeded"]) else "#ff6b6b"
    status_style = {
        "background": "#0f1117", "color": ok_color,
        "padding": "8px", "borderRadius": "4px",
        "border": "1px solid #333", "fontFamily": "monospace",
    }

    def _fv(v, u):
        return f"{v:+.4f} {u}"

    live_v = [_fv(s["dof_live"][i], DOF_UNITS[i]) for i in range(N_DOFS)]
    tgt_v  = [_fv(s["dof_tgt"][i],  DOF_UNITS[i]) for i in range(N_DOFS)]
    step_v = [_fv(s["steps"][i],    DOF_UNITS[i]) for i in range(N_DOFS)]

    def _ik_div(arm):
        info = s["ik_info"].get(arm, {})
        ok = info.get("success")
        mode = s.get("arm_control_mode", "both")
        if mode not in ("both", arm):
            return html.Div()
        if ok is None:
            return html.Div(f"IK {arm}: pending", className="text-muted small")
        extra = ""
        m = info.get("mode", "")
        if m == "pos_shoulder_elbow":
            extra = "  shoulder+elbow"
        elif m == "pos_axis_clamped":
            extra = f"  axis-clamped ({info.get('axis_error_m', 0):.4f}m)"
        if ok:
            txt = (f"IK {arm}: ✓ OK  "
                   f"pos={info['error_pos_m']:.4f}m  "
                   f"rot={info['error_rot_rad']:.4f}rad  "
                   f"{info['iterations']}it{extra}")
            return html.Div(txt, className="text-success small font-monospace")
        txt = (f"IK {arm}: ✗ FAIL  "
               f"pos={info['error_pos_m']:.4f}m  "
               f"rot={info['error_rot_rad']:.4f}rad")
        return html.Div(txt, className="text-danger small font-monospace")

    ik_left  = _ik_div("left")
    ik_right = _ik_div("right")

    # Joint readout
    arm_mode = s.get("arm_control_mode", "both")
    rows = []
    for arm2, jlist in ARM_JOINTS.items():
        if arm_mode not in ("both", arm2):
            continue
        lbl = "  ".join(f"{n:<7}" for n in JOINT_LABELS)
        tgt = "  ".join(f"{v:+.3f}" for v in s["joint_tgt"][arm2])
        liv = "  ".join(f"{v:+.3f}" for v in s["joint_live"][arm2])
        rows += [
            html.Div([html.Span(f"{arm2.upper()}: ", className="text-info"),
                      html.Span(lbl)]),
            html.Div([html.Span("target: ", className="text-muted"),
                      html.Span(tgt),
                      html.Span("   live: ", className="text-warning ms-3"),
                      html.Span(liv, className="text-warning")]),
        ]
    joint_div = html.Div(rows)

    pose_opts = [
        {"label": f"{i}: {p['name']}  {p['saved_at']}", "value": i}
        for i, p in enumerate(s["saved_poses"])
    ]
    seq_opts = []
    for i, step in enumerate(s["sequence_steps"]):
        w = "waist" if step["include_waist"] else "arms"
        indicator = " ▶ ACTIVE" if i == s["running_step"] else ""
        seq_opts.append({
            "label": f"{i+1}: {step['pose_name']} [{w}]{indicator}",
            "value": i,
        })

    return (
        conn_badge, armed_badge,
        s["status"], status_style,
        live_v, tgt_v, step_v,
        ik_left, ik_right,
        joint_div,
        pose_opts, seq_opts,
        _hand_status,
    )


# ── DOF adjust ────────────────────────────────────────────────────────────────

@app.callback(
    Output("_act", "data", allow_duplicate=True),
    Input({"type": "inc-dof",    "index": ALL}, "n_clicks"),
    Input({"type": "dec-dof",    "index": ALL}, "n_clicks"),
    Input({"type": "halve-step", "index": ALL}, "n_clicks"),
    Input({"type": "double-step","index": ALL}, "n_clicks"),
    prevent_initial_call=True,
)
def dof_buttons(*_):
    ctx = callback_context
    if not ctx.triggered:
        return no_update
    try:
        id_dict  = json.loads(ctx.triggered[0]["prop_id"].rsplit(".", 1)[0])
        btn_type = id_dict["type"]
        idx      = int(id_dict["index"])
    except Exception:
        return no_update

    with _lock:
        c = _ctrl
        if c is None or not c.seeded or not c.armed:
            return no_update
        prev = c.dof_idx
        c.dof_idx = idx
        try:
            if btn_type == "inc-dof":
                c._adjust_dof(+c._ee_step())
            elif btn_type == "dec-dof":
                c._adjust_dof(-c._ee_step())
            elif btn_type == "halve-step":
                c._set_ee_step(max(0.0001, c._ee_step() / 2.0))
            elif btn_type == "double-step":
                hi = 1.0 if idx < 3 else math.pi
                c._set_ee_step(min(hi, c._ee_step() * 2.0))
        finally:
            c.dof_idx = prev
    return no_update


# ── Arm mode / waist / stiffness ──────────────────────────────────────────────

@app.callback(
    Output("_act", "data", allow_duplicate=True),
    Input("arm-mode",     "value"),
    Input("waist-toggle", "value"),
    Input("orient-stiff", "value"),
    prevent_initial_call=True,
)
def settings_changed(arm_mode, waist_val, stiff_val):
    with _lock:
        c = _ctrl
        if c is None:
            return no_update
        if arm_mode is not None:
            c.arm_control_mode = arm_mode
        prev_waist = c.waist_enabled
        c.waist_enabled = bool(waist_val)
        if c.waist_enabled and not prev_waist:
            for j in WAIST_JOINTS:
                c.desired_targets[j] = c.latest_positions.get(j, c.desired_targets[j])
                c.current_targets[j] = c.desired_targets[j]
            c._sync_ee_from_joints()
        c.orient_stiff = bool(stiff_val)
    return no_update


# ── Parameter set ─────────────────────────────────────────────────────────────

@app.callback(
    Output("_act", "data", allow_duplicate=True),
    Input("ramp-set",      "n_clicks"),
    Input("maxdq-set",     "n_clicks"),
    Input("waist-kp-set",  "n_clicks"),
    State("ramp-input",     "value"),
    State("maxdq-input",    "value"),
    State("waist-kp-input", "value"),
    prevent_initial_call=True,
)
def param_set(r, d, w, ramp, maxdq, wkp):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    with _lock:
        c = _ctrl
        if c is None:
            return no_update
        if trig == "ramp-set"     and ramp  is not None:
            c.max_speed   = max(0.01, float(ramp))
            c.status = f"Ramp speed → {c.max_speed:.4f} r/s"
        elif trig == "maxdq-set"  and maxdq is not None:
            c.max_dq      = max(0.005, min(math.pi, float(maxdq)))
            c.status = f"max_dq → {c.max_dq:.4f} rad"
        elif trig == "waist-kp-set" and wkp is not None:
            c.waist_pr_kp = max(0.0, float(wkp))
            c.status = f"Waist PR kp → {c.waist_pr_kp:.1f}"
    return no_update


# ── Action buttons ────────────────────────────────────────────────────────────

@app.callback(
    Output("_act", "data", allow_duplicate=True),
    Input("btn-sync",      "n_clicks"),
    Input("btn-release",   "n_clicks"),
    Input("btn-reengage",  "n_clicks"),
    Input("btn-zero-gain", "n_clicks"),
    prevent_initial_call=True,
)
def action_buttons(*_):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    with _lock:
        c = _ctrl
        if c is None:
            return no_update
        if trig == "btn-sync":
            c._sync_targets_to_live()
            c.status = "EE targets resynced to current hand pose"
        elif trig == "btn-release":
            def _do():
                with _lock:
                    try:
                        c._release_arms()
                        c.armed = False
                        c.status = "Arms released"
                    except Exception as exc:
                        c.status = f"Release failed: {exc}"
            threading.Thread(target=_do, daemon=True).start()
        elif trig == "btn-reengage":
            def _do():
                with _lock:
                    try:
                        c._unrelease_arms()
                        c.armed = True
                        c.status = "Reengaged — holding current arm pose"
                    except Exception as exc:
                        c.status = f"Reengage failed: {exc}"
            threading.Thread(target=_do, daemon=True).start()
        elif trig == "btn-zero-gain":
            c.pub.publish_zero_gains(c.current_targets)
            c.status = "Zero-gain hold sent"
    return no_update


# ── Dex3 hand mode + grip ─────────────────────────────────────────────────────

@app.callback(
    Output("_act", "data", allow_duplicate=True),
    Input("hand-mode", "value"),
    Input("grip-set",  "n_clicks"),
    State("grip-input","value"),
    prevent_initial_call=True,
)
def dex3_mode_grip(hand_mode, _g, grip):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    with _lock:
        c = _ctrl
        if c is None:
            return no_update
        if trig == "hand-mode" and hand_mode is not None:
            c.hand_control_mode = hand_mode
            c._init_hand_controllers()
            c.status = f"Dex3 mode → {hand_mode}"
        elif trig == "grip-set" and grip is not None:
            c.hand_grip_percent = max(0.0, min(100.0, float(grip)))
            c._ensure_hand_control_for_grip()
            c._publish_hand_targets_once()
            c.status = f"Dex3 grip → {c.hand_grip_percent:.0f}%"
    return no_update


# ── Dex3 open / close ─────────────────────────────────────────────────────────

@app.callback(
    Output("_hand-act", "data", allow_duplicate=True),
    Input("dex3-open",  "n_clicks"),
    Input("dex3-close", "n_clicks"),
    State("dex3-side",   "value"),
    State("dex3-hold-s", "value"),
    State("dex3-ramp-s", "value"),
    prevent_initial_call=True,
)
def dex3_open_close(_o, _c, side, hold_s, ramp_s):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    action = "open" if trig == "dex3-open" else "close"
    hs  = max(0.0, float(hold_s or 0.6))
    rs  = float(ramp_s or 0) or None
    _run_hand_action(_dex3_action, side or "right", action, hs, rs)
    return no_update


# ── Inspire open / close ──────────────────────────────────────────────────────

@app.callback(
    Output("_hand-act", "data", allow_duplicate=True),
    Input("inspire-open",  "n_clicks"),
    Input("inspire-close", "n_clicks"),
    State("inspire-side",  "value"),
    State("inspire-speed", "value"),
    State("inspire-force", "value"),
    prevent_initial_call=True,
)
def inspire_open_close(_o, _c, side, speed, force):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    action = "open" if trig == "inspire-open" else "close"
    sp = int(speed or 200)
    fo = int(force or 200)
    _run_hand_action(_inspire_action, side or "right", action, sp, fo)
    return no_update


# ── Pose actions ──────────────────────────────────────────────────────────────

@app.callback(
    Output("_act", "data", allow_duplicate=True),
    Input("pose-save",    "n_clicks"),
    Input("pose-load",    "n_clicks"),
    Input("pose-delete",  "n_clicks"),
    Input("pose-add-seq", "n_clicks"),
    State("pose-name-input",  "value"),
    State("pose-selector",    "value"),
    State("include-waist-new","value"),
    prevent_initial_call=True,
)
def pose_actions(*_):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    states = callback_context.states
    name     = states.get("pose-name-input.value")
    selected = states.get("pose-selector.value")
    waist    = bool(states.get("include-waist-new.value", True))

    with _lock:
        c = _ctrl
        if c is None:
            return no_update
        if trig == "pose-save":
            if name:
                c.saved_poses.append(c._pose_payload(name))
                c._write_pose_file()
                c.status = f"Saved pose '{name}'"
        elif trig == "pose-load":
            idx = selected
            if idx is not None and 0 <= idx < len(c.saved_poses):
                try:
                    c._apply_joint_pose(c.saved_poses[idx], include_waist=True)
                    c.status = f"Loaded '{c.saved_poses[idx].get('name','?')}'"
                except Exception as exc:
                    c.status = f"Load failed: {exc}"
        elif trig == "pose-delete":
            idx = selected
            if idx is not None and 0 <= idx < len(c.saved_poses):
                pname = c.saved_poses[idx].get("name", "?")
                new_steps = []
                for step in c.sequence_steps:
                    pi = step.get("pose_index", -1)
                    if pi == idx:
                        continue
                    new_steps.append({
                        "pose_index": pi - (1 if pi > idx else 0),
                        "include_waist": step.get("include_waist", True),
                    })
                c.sequence_steps = new_steps
                del c.saved_poses[idx]
                c.sequence_running = False
                c._write_pose_file()
                c.status = f"Deleted pose '{pname}'"
        elif trig == "pose-add-seq":
            idx = selected
            if idx is not None and 0 <= idx < len(c.saved_poses):
                c.include_waist_new = waist
                c.sequence_steps.append({"pose_index": idx, "include_waist": waist})
                c._write_pose_file()
                pname = c.saved_poses[idx].get("name", "?")
                c.status = f"Added '{pname}' to sequence"
    return no_update


# ── Sequence actions ──────────────────────────────────────────────────────────

@app.callback(
    Output("_act", "data", allow_duplicate=True),
    Input("seq-up",      "n_clicks"),
    Input("seq-down",    "n_clicks"),
    Input("seq-remove",  "n_clicks"),
    Input("seq-run",     "n_clicks"),
    Input("seq-stop",    "n_clicks"),
    Input("seq-gap-set", "n_clicks"),
    State("seq-selector",  "value"),
    State("seq-gap-input", "value"),
    prevent_initial_call=True,
)
def seq_actions(*_):
    trig = (callback_context.triggered or [{}])[0].get("prop_id", "").split(".")[0]
    states = callback_context.states
    idx = states.get("seq-selector.value")
    gap = states.get("seq-gap-input.value")

    with _lock:
        c = _ctrl
        if c is None:
            return no_update
        if trig == "seq-up":
            if idx is not None and 1 <= idx < len(c.sequence_steps):
                c.sequence_steps[idx-1], c.sequence_steps[idx] = (
                    c.sequence_steps[idx], c.sequence_steps[idx-1])
                c._write_pose_file()
        elif trig == "seq-down":
            if idx is not None and 0 <= idx < len(c.sequence_steps) - 1:
                c.sequence_steps[idx+1], c.sequence_steps[idx] = (
                    c.sequence_steps[idx], c.sequence_steps[idx+1])
                c._write_pose_file()
        elif trig == "seq-remove":
            if idx is not None and 0 <= idx < len(c.sequence_steps):
                del c.sequence_steps[idx]
                c.sequence_running = False
                c._write_pose_file()
                c.status = "Removed sequence step"
        elif trig == "seq-run":
            if not c.armed:
                c.status = "Reengage arms before running a sequence"
            elif not c.sequence_steps:
                c.status = "Add poses to the sequence first"
            else:
                c.sequence_running = True
                c.sequence_step_index = 0
                c.sequence_next_time_s = 0.0
                c.status = "Sequence started"
        elif trig == "seq-stop":
            c.sequence_running = False
            c.sequence_step_index = 0
            c.sequence_next_time_s = 0.0
            c.status = "Sequence stopped"
        elif trig == "seq-gap-set" and gap is not None:
            c.sequence_gap_s = max(0.0, float(gap))
            c._write_pose_file()
            c.status = f"Sequence gap → {c.sequence_gap_s:.1f} s"
    return no_update


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    global _ctrl, _tick_running, _tick_thread

    # Split our extra flags from the ik_pose_cli_v3 flags
    extra_parser = argparse.ArgumentParser(add_help=False)
    extra_parser.add_argument("--port",  type=int,  default=8050)
    extra_parser.add_argument("--host",  default="0.0.0.0")
    extra_parser.add_argument("--debug", action="store_true")
    extra_args, remaining = extra_parser.parse_known_args()

    # Parse remaining with the original CLI parser
    orig_argv = sys.argv
    sys.argv = [sys.argv[0]] + remaining
    args = _parse_args()
    sys.argv = orig_argv

    print("Initialising IK controller…")
    try:
        _ctrl = IKPoseCLI(args)
        _ctrl._last_tick = time.monotonic()
    except ControllerLockError as exc:
        print(f"ik_pose_dash: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc

    _tick_running = True
    _tick_thread = threading.Thread(target=_tick_loop, daemon=True)
    _tick_thread.start()

    print(f"Open  http://localhost:{extra_args.port}  in your browser.")

    import atexit, signal as _signal

    shutdown_done = False

    def _shutdown() -> None:
        nonlocal shutdown_done
        global _tick_running
        if shutdown_done:
            return
        shutdown_done = True
        _tick_running = False
        if _ctrl and not _ctrl._closed:
            _ctrl.close()
        if _tick_thread and _tick_thread.is_alive():
            _tick_thread.join(timeout=1.0)

    def _signal_stop(signum, _frame) -> None:
        _shutdown()
        raise SystemExit(128 + int(signum))

    atexit.register(_shutdown)
    for _sig in ("SIGINT", "SIGTERM"):
        try:
            _signal.signal(getattr(_signal, _sig), _signal_stop)
        except Exception:
            pass

    try:
        app.run(
            host=extra_args.host,
            port=extra_args.port,
            debug=extra_args.debug,
            use_reloader=False,
        )
    finally:
        _shutdown()


if __name__ == "__main__":
    main()
