from __future__ import annotations

from pathlib import Path

import dash
from dash import dcc, html
import dash_bootstrap_components as dbc


BASE_DIR = Path(__file__).resolve().parent
DAY_FILES = {
    "Day 1": BASE_DIR / "day_1_final.md",
    "Day 2": BASE_DIR / "day_2_final.md",
    "Day 3": BASE_DIR / "day_3_final.md",
    "Day 4": BASE_DIR / "day_4_final.md",
}


def load_md(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return f"# Missing file\n\nCould not find `{path.name}`."


app = dash.Dash(__name__, external_stylesheets=[dbc.themes.FLATLY])
server = app.server


tabs = []
for day, path in DAY_FILES.items():
    tabs.append(
        dbc.Tab(
            label=day,
            tab_id=day.lower().replace(" ", "-"),
            children=dbc.Card(
                dbc.CardBody(
                    dcc.Markdown(load_md(path), link_target="_blank"),
                ),
                class_name="shadow-sm",
            ),
        )
    )


app.layout = dbc.Container(
    [
        html.Style(
            """
            img {
                max-height: 420px;
                height: auto;
                width: auto;
                max-width: 100%;
                display: block;
            }
            """
        ),
        html.H1("Unitree G1 Academy - Course Days", className="mt-3"),
        html.P(
            "Browse each day of the course. Content is loaded from the local markdown files.",
            className="text-muted",
        ),
        dbc.Tabs(tabs, active_tab="day-1"),
    ],
    fluid=True,
)


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=8050)
